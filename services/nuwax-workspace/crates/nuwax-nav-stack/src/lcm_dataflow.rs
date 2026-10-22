//! Real-LCM adapters for in-process navigation graph nodes.

#[cfg(any(feature = "cmu-native", feature = "explore"))]
use std::collections::{HashMap, HashSet};
use std::future::Future;
use std::pin::Pin;
#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{mpsc as std_mpsc, Arc, Mutex};
use std::thread::JoinHandle;
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
pub(crate) use surface_dataflow::register_surface_owners;

use crate::error::{NavStackError, NavStackResult};
use crate::graph::ModuleNode;
use crate::runner::InProcessNode;

const DEFAULT_LCM_URL: &str = "udpm://239.255.76.67:7667?ttl=0";
const STARTUP_TIMEOUT: Duration = Duration::from_secs(10);
#[cfg(any(feature = "cmu-native", feature = "explore"))]
static FAR_READY_GENERATION: AtomicI64 = AtomicI64::new(0);
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
    if type_name == "nav_msgs.OccupancyGrid" && planar_navigation_enabled() {
        return planar_navigation_stream(transport,topic).await;
    }
    Ok(transport.subscribe(topic, Some(message_type)).await)
}

fn planar_navigation_enabled()->bool {
    std::env::var("NUWAX_GEOMETRY_PIPELINE").as_deref()==Ok("slam_2d")
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
                let now=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                    .map(|d|d.as_nanos().min(i64::MAX as u128) as i64).unwrap_or(0);
                match planar_navigation::decode_body(&frame.payload)
                    .map_err(|e|e.to_string())
                    .and_then(|map|admission.admit(&map,now).map(|changed|(map,changed))) {
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

#[cfg(all(
    feature = "native-runner",
    any(feature = "cmu-native", feature = "explore")
))]
#[derive(Clone)]
pub(crate) enum NativePlannerReadyContract {
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    Far(nuwax_nav_far_planner_bridge::worker::FarReadyContract),
}


#[cfg(all(
    feature = "native-runner",
    any(feature = "cmu-native", feature = "explore")
))]
fn far_ready_contract_matches(
    expected: &nuwax_nav_far_planner_bridge::worker::FarReadyContract,
    generation: i64,
    state: &schema::NavigationState,
) -> bool {
    let mut expected = expected.clone();
    expected.generation = generation;
    state.state == nuwax_nav_far_planner_bridge::worker::FAR_EVENT_READY
        && state.sink_ready == 1
        && state.robot_frame == expected.backend
        && state.runtime_generation == generation
        && nuwax_nav_far_planner_bridge::worker::decode_ready_contract_v2(&state.message)
            .is_ok_and(|contract| contract == expected)
}

#[cfg(all(
    feature = "native-runner",
    any(feature = "cmu-native", feature = "explore")
))]
pub(crate) fn native_planner_readiness(
    expected: NativePlannerReadyContract,
    restart_generation: u64,
) -> nuwax_nav_native_runner::TypedReadyFuture {
    Box::pin(async move {
        use nuwax_nav_native_runner::SupervisedReadyError;

        let generation = i64::try_from(restart_generation).map_err(|_| {
            SupervisedReadyError::Protocol("restart generation exceeds int64".into())
        })?;
        let event_channel = match &expected {
            #[cfg(any(feature = "cmu-native", feature = "explore"))]
            NativePlannerReadyContract::Far(contract) => {
                FAR_READY_GENERATION.store(0, Ordering::Release);
                contract.worker_event.clone()
            }

        };
        let provider = std::env::var("NUWAX_LCM_URL").unwrap_or_else(|_| DEFAULT_LCM_URL.into());
        let transport = LcmTransport::new_real(&provider)
            .map_err(|error| SupervisedReadyError::Transport(error.to_string()))?;
        let topic = protocol_topic(&event_channel)
            .map_err(|error| SupervisedReadyError::Protocol(error.to_string()))?;
        let message_type = LcmMessageType::new("nuwax_msgs.NavigationState")
            .map_err(|error| SupervisedReadyError::Protocol(error.to_string()))?;
        let mut events = transport
            .subscribe_owned(topic, Some(message_type))
            .await
            .map_err(|error| SupervisedReadyError::Transport(error.to_string()))?;

        loop {
            let frame = events
                .recv()
                .await
                .map_err(|error| SupervisedReadyError::Transport(error.to_string()))?;
            match &expected {

                #[cfg(any(feature = "cmu-native", feature = "explore"))]
                NativePlannerReadyContract::Far(expected_contract) => {
                    let state = match schema::decode_navigation_state_body(&frame.payload) {
                        Ok(state) => state,
                        Err(_) => continue,
                    };
                    if far_ready_contract_matches(expected_contract, generation, &state) {
                        FAR_READY_GENERATION.store(generation, Ordering::Release);
                        return Ok(());
                    }
                }
            }
        }
    })
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
pub(crate) fn map_pose_owner(node: &ModuleNode, body_reference_frame_id: Arc<str>) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 2, &[1])?;
    let node = node.clone();
    Ok(boxed_owner(
        "map-pose-adapter",
        Box::new(move |shutdown, ready| Box::pin(run_map_pose_adapter(node, body_reference_frame_id, shutdown, ready))),
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
fn fence_map_pose_source_timestamp(
    last_published_timestamp_ns: Option<i64>,
    observed: &crate::map_pose::MapPoseObservation,
) -> OwnerInputResult<i64> {
    let candidate_timestamp_ns =
        lcm_header_timestamp_ns(&observed.pose.header, "map_pose_adapter.output")?;
    if let Some(published_timestamp_ns) = last_published_timestamp_ns {
        if candidate_timestamp_ns <= published_timestamp_ns {
            return Ok(OwnerInput::Rejected(format!(
                "map_pose_adapter.output timestamp {candidate_timestamp_ns} is duplicate or older than the last published timestamp {published_timestamp_ns}"
            )));
        }
    }
    Ok(OwnerInput::Accepted(candidate_timestamp_ns))
}

#[cfg(feature = "guard-runtime")]
async fn publish_map_pose_observation(
    transport: &LcmTransport,
    channel: &str,
    last_published_timestamp_ns: &mut Option<i64>,
    diagnostics: &mut InputDiagnosticLimiter,
    observed: crate::map_pose::MapPoseObservation,
) -> NavStackResult<()> {
    let candidate_timestamp_ns =
        match fence_map_pose_source_timestamp(*last_published_timestamp_ns, &observed)? {
            OwnerInput::Accepted(timestamp_ns) => timestamp_ns,
            OwnerInput::Rejected(reason) => {
                diagnostics.reject("map_pose_adapter.output", &reason);
                return Ok(());
            }
        };
    let body = schema::encode_pose_stamped_body(&observed.pose)
        .map_err(|error| NavStackError::Internal(format!("map pose encode: {error}")))?;
    publish_body(transport, channel, "geometry_msgs.PoseStamped", body).await?;
    *last_published_timestamp_ns = Some(candidate_timestamp_ns);
    Ok(())
}

#[cfg(feature = "guard-runtime")]
fn discard_pending_odometry_through(
    pending_odometry: &mut Option<Box<schema::Odometry>>,
    observed: &crate::map_pose::MapPoseObservation,
) -> NavStackResult<()> {
    let Some(pending) = pending_odometry.as_ref() else {
        return Ok(());
    };
    let pending_timestamp =
        lcm_header_timestamp_ns(&pending.header, "map_pose_adapter.pending_odometry")?;
    let observed_timestamp =
        lcm_header_timestamp_ns(&observed.pose.header, "map_pose_adapter.atomic_tf")?;
    if pending_timestamp <= observed_timestamp {
        *pending_odometry = None;
    }
    Ok(())
}

#[cfg(feature = "guard-runtime")]
const MAX_REPLACEABLE_POSE_BATCH: usize = 64;

#[cfg(feature = "guard-runtime")]
fn retain_newest_map_pose(
    newest: &mut Option<crate::map_pose::MapPoseObservation>,
    candidate: crate::map_pose::MapPoseObservation,
) -> NavStackResult<()> {
    let stamp = lcm_header_timestamp_ns(&candidate.pose.header, "map_pose_adapter.output")?;
    let current = newest
        .as_ref()
        .map(|value| lcm_header_timestamp_ns(&value.pose.header, "map_pose_adapter.output"))
        .transpose()?
        .unwrap_or(0);
    if stamp > current {
        *newest = Some(candidate);
    }
    Ok(())
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
    node: ModuleNode,
    body_reference_frame_id: Arc<str>,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut odometry = typed_stream(&transport, &node.inputs[0], "nav_msgs.Odometry").await?;
    let mut transforms = typed_stream(&transport, &node.inputs[1], "tf2_msgs.TFMessage").await?;
    let mut adapter = bound_map_pose_adapter(
        configured_frame("NUWAX_GEOMETRY_MAP_FRAME", "map")?,
        configured_frame("NUWAX_GEOMETRY_ODOM_FRAME", "odom")?,
        &body_reference_frame_id,
    )?;
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut pending_odometry: Option<Box<schema::Odometry>> = None;
    let mut last_published_timestamp_ns = None;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => return Ok(()),
            message = transforms.next() => {
                let mut next = message;
                let mut newest = None;
                for index in 0..MAX_REPLACEABLE_POSE_BATCH {
                    let frame = required_frame(next, "map_pose_adapter.tf")?;
                    match ingest_map_pose_tf(&mut adapter, &frame.payload)? {
                        OwnerInput::Accepted(Some(observed)) => {
                            discard_pending_odometry_through(&mut pending_odometry, &observed)?;
                            retain_newest_map_pose(&mut newest, observed)?;
                        }
                        OwnerInput::Accepted(None) => {
                            if let Some(odometry) = pending_odometry.take() {
                                match transform_map_pose_odometry(&adapter, *odometry)? {
                                    MapPoseOdometryIngress::Accepted(observed) => {
                                        retain_newest_map_pose(&mut newest, observed)?;
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
                    if index + 1 == MAX_REPLACEABLE_POSE_BATCH { break; }
                    match transforms.next().now_or_never() {
                        Some(message) => next = message,
                        None => break,
                    }
                }
                if let Some(observed) = newest {
                    publish_map_pose_observation(&transport, &node.outputs[0],
                        &mut last_published_timestamp_ns, &mut diagnostics, observed).await?;
                }
            }
            message = odometry.next() => {
                let mut next = message;
                let mut newest = None;
                for index in 0..MAX_REPLACEABLE_POSE_BATCH {
                    let frame = required_frame(next, "map_pose_adapter.odometry")?;
                    match ingest_map_pose_odometry(&adapter, &frame.payload)? {
                        MapPoseOdometryIngress::Accepted(observed) => {
                            discard_pending_odometry_through(&mut pending_odometry, &observed)?;
                            retain_newest_map_pose(&mut newest, observed)?;
                        }
                        MapPoseOdometryIngress::AwaitingTransform(candidate, reason) => {
                            let candidate_stamp = lcm_header_timestamp_ns(&candidate.header, "map_pose_adapter.odometry")?;
                            let pending_stamp = pending_odometry.as_ref()
                                .map(|pending| lcm_header_timestamp_ns(&pending.header, "map_pose_adapter.pending"))
                                .transpose()?.unwrap_or(0);
                            if candidate_stamp > pending_stamp {
                                pending_odometry = Some(candidate);
                            }
                            diagnostics.reject("map_pose_adapter.odometry", &reason);
                        }
                        MapPoseOdometryIngress::Rejected(reason) => {
                            diagnostics.reject("map_pose_adapter.odometry", &reason);
                        }
                    }
                    if index + 1 == MAX_REPLACEABLE_POSE_BATCH { break; }
                    match odometry.next().now_or_never() {
                        Some(message) => next = message,
                        None => break,
                    }
                }
                if let Some(observed) = newest {
                    publish_map_pose_observation(&transport, &node.outputs[0],
                        &mut last_published_timestamp_ns, &mut diagnostics, observed).await?;
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

fn configured_frame_alias(
    preferred_key: &str,
    legacy_key: &str,
    fallback: &str,
) -> NavStackResult<String> {
    let preferred = std::env::var(preferred_key).ok();
    let legacy = std::env::var(legacy_key).ok();
    resolve_frame_alias(preferred.as_deref(), legacy.as_deref(), fallback).ok_or_else(|| {
        NavStackError::InvalidConfig(format!(
            "{preferred_key}, {legacy_key}, and fallback frame must not all be empty"
        ))
    })
}

fn resolve_frame_alias(
    preferred: Option<&str>,
    legacy: Option<&str>,
    fallback: &str,
) -> Option<String> {
    preferred
        .and_then(|value| (!value.trim().is_empty()).then(|| value.trim().to_string()))
        .or_else(|| {
            legacy.and_then(|value| (!value.trim().is_empty()).then(|| value.trim().to_string()))
        })
        .or_else(|| (!fallback.trim().is_empty()).then(|| fallback.trim().to_string()))
}

#[cfg(feature = "guard-runtime")]
pub(crate) fn guarded_nav_scheduler_owner(
    node: &ModuleNode, body_reference_frame_id: Arc<str>) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 5, &[4])?;
    let node = node.clone();
    Ok(boxed_owner(
        "guarded-nav-scheduler",
        Box::new(move |shutdown, ready| Box::pin(run_guarded_nav_scheduler(node, body_reference_frame_id, shutdown, ready))),
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
    // 旧线格式的第三项仍为yaw；三维高度在内部独立保留用于真实进度。
    current_z: f64,
    goal_z: f64,
    surface_navigation: bool,
    initial_distance: f64,
    progress: f64,
    path_revision: i64,
    producer_runtime_generation: i64,
    planner_runtime_generation: i64,
}

#[cfg(feature = "guard-runtime")]
fn guarded_goal_execution_identity(
    goal: &schema::GoalEnvelope,
) -> nuwax_nav_runtime::PathExecutionIdentity {
    nuwax_nav_runtime::PathExecutionIdentity {
        goal_key: nuwax_nav_types_messages::GoalKey { producer_runtime_generation: (goal.producer_runtime_generation) as u64, goal_generation: (goal.goal_generation) as u64 },
        session_id: goal.session_id.clone(),


        planner_runtime_generation: 0,
        path_revision: 0,
    }
}

#[cfg(feature = "guard-runtime")]
fn guarded_path_execution_identity(
    path: &schema::PathEnvelope,
) -> nuwax_nav_runtime::PathExecutionIdentity {
    nuwax_nav_runtime::PathExecutionIdentity {
        goal_key: nuwax_nav_types_messages::GoalKey { producer_runtime_generation: (path.producer_runtime_generation) as u64, goal_generation: (path.goal_generation) as u64 },
        session_id: path.session_id.clone(),


        planner_runtime_generation: path.planner_runtime_generation,
        path_revision: u64::try_from(path.path_revision).expect("validated positive path revision"),
    }
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone)]
struct GuardedCancelTerminal {
    goal: schema::GoalEnvelope,
    cancel_generation: u64,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: GuardedNavTelemetry,
}

#[cfg(feature = "guard-runtime")]
#[derive(Clone)]
struct GuardedCancelGenerationWatermark {
    goal: schema::GoalEnvelope,
    generation: u64,
}

#[cfg(feature = "guard-runtime")]
fn observe_guarded_cancel_generation(
    watermark: &mut Option<GuardedCancelGenerationWatermark>,
    goal: &schema::GoalEnvelope,
    generation: u64,
) -> Result<(), String> {
    if let Some(observed) = watermark.as_mut().filter(|observed| {
        planner_goal_identity_matches(&observed.goal, goal)
    }) {
        if generation < observed.generation {
            return Err(format!(
                "stale cancel generation {generation}; highest observed generation is {}",
                observed.generation
            ));
        }
        observed.generation = observed.generation.max(generation);
        return Ok(());
    }
    *watermark = Some(GuardedCancelGenerationWatermark {
        goal: goal.clone(),
        generation,
    });
    Ok(())
}

#[cfg(feature = "guard-runtime")]
struct GuardedPoseInput {
    pose: PoseStamped,
    frame_id: String,
    current: [f64; 3],
    source_deadline: Option<tokio::time::Instant>,
}

#[cfg(feature = "guard-runtime")]
fn drain_guarded_pose_input(
    first: Option<TransportResult<RawLcmFrame>>,
    poses: &mut LcmFrameStream,
    diagnostics: &mut InputDiagnosticLimiter,
    source_max_age: Option<Duration>,
) -> NavStackResult<Option<GuardedPoseInput>> {
    let wall_now = std::time::SystemTime::now();
    let mono_now = tokio::time::Instant::now();
    let mut next = first;
    let mut newest: Option<GuardedPoseInput> = None;
    for index in 0..MAX_REPLACEABLE_POSE_BATCH {
        let frame = required_frame(next, "guarded_nav_scheduler.map_pose")?;
        match decode_guarded_pose_input(&frame.payload)? {
            OwnerInput::Accepted(mut candidate) => {
                let valid = source_max_age
                    .map(|maximum| {
                        guarded_pose_source_deadline(
                            candidate.pose.stamp.nanos,
                            maximum,
                            wall_now,
                            mono_now,
                        )
                    })
                    .transpose();
                match valid {
                    Err(reason) => diagnostics.reject("guarded_nav_scheduler.map_pose", &reason),
                    Ok(deadline) => {
                        candidate.source_deadline = deadline;
                        if newest.as_ref().is_none_or(|value| candidate.pose.stamp.nanos > value.pose.stamp.nanos) {
                            newest = Some(candidate);
                        }
                    }
                }
            }
            OwnerInput::Rejected(reason) => {
                diagnostics.reject("guarded_nav_scheduler.map_pose", &reason)
            }
        }
        if index + 1 == MAX_REPLACEABLE_POSE_BATCH {
            break;
        }
        match poses.next().now_or_never() {
            Some(message) => next = message,
            None => break,
        }
    }
    Ok(newest)
}

#[cfg(feature = "guard-runtime")]
fn configured_pose_source_max_age() -> NavStackResult<Duration> {
    let raw = std::env::var("NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC").unwrap_or_else(|_| "0.5".into());
    let seconds: f64 = raw.parse().map_err(|_| {
        NavStackError::InvalidConfig("NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC必须为正有限秒数".into())
    })?;
    Duration::try_from_secs_f64(seconds)
        .ok()
        .filter(|value| !value.is_zero())
        .ok_or_else(|| {
            NavStackError::InvalidConfig("NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC必须为正有限秒数".into())
        })
}

#[cfg(feature = "guard-runtime")]
fn guarded_pose_source_deadline(
    source_timestamp_ns: i64,
    max_age: Duration,
    wall_now: std::time::SystemTime,
    mono_now: tokio::time::Instant,
) -> Result<tokio::time::Instant, String> {
    let unix_now = wall_now
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|_| "当前墙钟早于Unix纪元".to_string())?;
    let source = u64::try_from(source_timestamp_ns)
        .ok().filter(|value| *value > 0)
        .ok_or_else(|| "位姿源时间必须为正".to_string())?;
    let age = Duration::from_nanos(nuwax_nav_types_messages::source_time::age_ns_at(
        source, unix_now.as_nanos() as u64,
    )?);
    let remaining = max_age
        .checked_sub(age)
        .filter(|value| !value.is_zero())
        .ok_or_else(|| format!("位姿原始采样已过期：{}毫秒", age.as_millis()))?;
    mono_now
        .checked_add(remaining)
        .ok_or_else(|| "位姿有效期限溢出".into())
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
        source_deadline: None,
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
    node: ModuleNode,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let source_max_age = configured_pose_source_max_age()?;
    let transport = lcm_transport()?;
    let goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.GoalEnvelope").await?;
    let poses = typed_stream(&transport, &node.inputs[1], "geometry_msgs.PoseStamped").await?;
    let paths = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.PathEnvelope").await?;
    let costmap_type = if planar_navigation_enabled() {nuwax_codec::lcm::planar_navigation::TYPE_NAME} else {"nav_msgs.OccupancyGrid"};
    let costmaps = typed_stream(&transport, &node.inputs[3], costmap_type).await?;
    let cancels = typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let socket_path = std::env::var("NUWAX_MOTION_GUARD_SOCKET")
        .unwrap_or_else(|_| "/run/nuwax/motion-guard.sock".to_string());
    let mut scheduler_config = nuwax_nav_runtime::SchedulerConfig::default();
    if planar_navigation_enabled() {
        scheduler_config.costmap_ttl = Duration::from_nanos(nuwax_codec::lcm::planar_navigation::MAX_SOURCE_AGE_NS as u64);
    }
    let (scheduler, task) = nuwax_nav_runtime::spawn_production_scheduler(
        std::path::PathBuf::from(socket_path),
        scheduler_config,
    )
    .await
    .map_err(|error| module_error("guarded_nav_scheduler.connect", error))?;
    let status = scheduler.subscribe_status();
    signal_ready(ready)?;

    let loop_future = run_guarded_nav_loop(
        &node,
        &transport,
        &scheduler,
        shutdown,
        goals,
        poses,
        paths,
        costmaps,
        cancels,
        status,
        Some(source_max_age),
        planar_navigation_enabled(),
        &body_reference_frame_id,
        None,
        None,
    );
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GuardedNavSource {
    Pose,
    Costmap,
    Status,
    Goal,
    Path,
    Cancel,
}

#[cfg(feature = "guard-runtime")]
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

    fn successor(self) -> Self {
        Self::ORDER[(self.index() + 1) % Self::ORDER.len()]
    }
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
    committed_map: schema::CommittedMapIdentity,
    producer_runtime_generation: i64,
    map_frame_generation: i64,
    cloud_revision: i64,
    cloud_occupancy_revision: i64,
    cloud_commit_sequence: i64,
    cloud_source_timestamp_ns: i64,
    commit_boot_nonce: [u8; 16],
}

#[cfg(feature = "guard-runtime")]
impl GuardedMapAuthority {
    fn same_domain(&self, other: &Self) -> bool {
        self.producer_runtime_generation == other.producer_runtime_generation
            && self.committed_map.session_id == other.committed_map.session_id
            && self.committed_map.session_generation == other.committed_map.session_generation
            && self.committed_map.frame_id == other.committed_map.frame_id
            && self.map_frame_generation == other.map_frame_generation
            && self.commit_boot_nonce == other.commit_boot_nonce
    }

    fn permits_goal(&self, goal: &schema::GoalEnvelope) -> bool {
        self.producer_runtime_generation == goal.producer_runtime_generation
            && self.map_frame_generation == goal.map_frame_generation
            && self.committed_map.session_id == goal.committed_map_session_id
            && self.committed_map.session_generation == goal.committed_map_session_generation
            && self.committed_map.frame_id == goal.committed_map_frame_id
            && self.committed_map.commit_boot_nonce == goal.committed_map_boot_nonce
            && self.committed_map.revision >= goal.committed_map_revision
            && if self.committed_map.revision == goal.committed_map_revision {
                self.committed_map.commit_sequence == goal.committed_map_commit_sequence
                    && self.committed_map.source_timestamp_ns == goal.committed_map_source_timestamp_ns
            } else {
                self.committed_map.commit_sequence > goal.committed_map_commit_sequence
            }
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
impl GuardedNavEvent {
    fn source(&self) -> GuardedNavSource {
        match self {
            Self::Pose(_) => GuardedNavSource::Pose,
            Self::Costmap(_) => GuardedNavSource::Costmap,
            Self::Status(_) => GuardedNavSource::Status,
            Self::Goal(_) => GuardedNavSource::Goal,
            Self::Path(_) => GuardedNavSource::Path,
            Self::Cancel(_) => GuardedNavSource::Cancel,
        }
    }
}

#[cfg(feature = "guard-runtime")]
struct GuardedNavArbiter {
    cursor: GuardedNavSource,
}

#[cfg(feature = "guard-runtime")]
impl Default for GuardedNavArbiter {
    fn default() -> Self {
        Self {
            cursor: GuardedNavSource::Pose,
        }
    }
}

#[cfg(feature = "guard-runtime")]
impl GuardedNavArbiter {
    fn source_at(&self, offset: usize) -> GuardedNavSource {
        GuardedNavSource::ORDER[(self.cursor.index() + offset) % GuardedNavSource::ORDER.len()]
    }

    fn record_dispatch(&mut self, source: GuardedNavSource) {
        self.cursor = source.successor();
    }

    #[cfg(test)]
    fn choose_ready(&mut self, ready: [bool; 6]) -> Option<GuardedNavSource> {
        let winner = (0..GuardedNavSource::ORDER.len())
            .map(|offset| self.source_at(offset))
            .find(|source| ready[source.index()]);
        if let Some(source) = winner {
            self.record_dispatch(source);
        }
        winner
    }
}

#[cfg(feature = "guard-runtime")]
fn scan_guarded_nav_event<M, T>(
    arbiter: &GuardedNavArbiter,
    goals: &mut LcmFrameStream,
    poses: &mut LcmFrameStream,
    paths: &mut LcmFrameStream,
    costmaps: &mut M,
    cancels: &mut LcmFrameStream,
    status: &mut tokio::sync::watch::Receiver<nuwax_nav_runtime::SchedulerStatus>,
) -> Option<GuardedNavEvent>
where
    M: Stream<Item = TransportResult<T>> + Unpin,
    T: Into<GuardedMapFrame>,
{
    for offset in 0..GuardedNavSource::ORDER.len() {
        let event = match arbiter.source_at(offset) {
            GuardedNavSource::Pose => poses.next().now_or_never().map(GuardedNavEvent::Pose),
            GuardedNavSource::Costmap => costmaps.next().now_or_never().map(|message| {
                GuardedNavEvent::Costmap(message.map(|frame| frame.map(Into::into)))
            }),
            GuardedNavSource::Status => {
                status.changed().now_or_never().map(GuardedNavEvent::Status)
            }
            GuardedNavSource::Goal => goals.next().now_or_never().map(GuardedNavEvent::Goal),
            GuardedNavSource::Path => paths.next().now_or_never().map(GuardedNavEvent::Path),
            GuardedNavSource::Cancel => cancels.next().now_or_never().map(GuardedNavEvent::Cancel),
        };
        if event.is_some() {
            return event;
        }
    }
    None
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
fn guarded_nav_shutdown_requested(shutdown: &mut oneshot::Receiver<()>) -> bool {
    match shutdown.try_recv() {
        Ok(()) | Err(tokio::sync::oneshot::error::TryRecvError::Closed) => true,
        Err(tokio::sync::oneshot::error::TryRecvError::Empty) => false,
    }
}

#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn run_guarded_nav_loop<M, T>(
    node: &ModuleNode,
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
    body_reference_frame_id: &str,
    surface_support_z_m: Option<f64>,
    recent_source_max_age: Option<Duration>,
) -> NavStackResult<()>
where
    M: Stream<Item = TransportResult<T>> + Unpin + Send,
    T: Into<GuardedMapFrame> + Send,
{
    let process_runtime_generation = runtime_generation();
    let authoritative_planner_generation = saturating_i64(process_runtime_generation);
    let mut telemetry = GuardedNavTelemetry {
        runtime_generation: process_runtime_generation,
        surface_navigation: node.id == "surface_nav_scheduler",
        ..GuardedNavTelemetry::default()
    };
    let mut active_session: Option<schema::GoalEnvelope> = None;
    let mut accepted_planner_generation: Option<(GoalKey, i64)> = None;
    let mut goal_buffer = PlannerGoalBuffer::default();
    let mut surface_authority: Option<GuardedMapAuthority> = None;
    #[cfg(feature="mls")]
    let mut recent_source=surface_dataflow::RecentSourceState::new(recent_source_max_age);
    if recent_source_max_age.is_some() {
        scheduler.update_obstacle_source(None,false,None).await
            .map_err(|error|module_error("recent source initial health",error))?;
    }
    let mut planar_admission=nuwax_codec::lcm::planar_navigation::Admission::default();
    let mut publications = GuardedStatusPublications::default();
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    { publications.qualification_timing=scheduler.qualification_timing().cloned(); }
    let mut diagnostics = InputDiagnosticLimiter::default();
    let mut last_cancel_terminal: Option<GuardedCancelTerminal> = None;
    // This watermark rejects generation rollback but never grants replay or
    // reacquisition eligibility; those remain owned by the active, pending,
    // and terminal goal records.
    let mut cancel_generation_watermark: Option<GuardedCancelGenerationWatermark> = None;
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

    let mut arbiter = GuardedNavArbiter::default();
    loop {
        if guarded_nav_shutdown_requested(&mut shutdown) {
            return Ok(());
        }
        let event = if let Some(event) = scan_guarded_nav_event(
            &arbiter,
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        ) {
            event
        } else {
            tokio::select! {
                biased;
                _ = &mut shutdown => return Ok(()),
                event = wait_guarded_nav_event(
                    &mut goals,
                    &mut poses,
                    &mut paths,
                    &mut costmaps,
                    &mut cancels,
                    &mut status,
                ) => event,
            }
        };
        let source = event.source();
        arbiter.record_dispatch(source);
        match event {
            GuardedNavEvent::Status(changed) => {
                changed.map_err(|_| {
                    NavStackError::Internal("guard scheduler status stream closed".into())
                })?;
                let snapshot = status.borrow().clone();
                publish_guarded_tail_observation(
                    transport, node, scheduler, snapshot, &mut telemetry, None, &mut publications,
                    &mut cancels, &mut active_session, &mut goal_buffer, &mut last_cancel_terminal,
                    &mut cancel_generation_watermark, &mut arbiter,
                ).await?;
            }
            GuardedNavEvent::Pose(message) => {
                #[cfg(all(test,feature="surface-recovery-qualification"))]
                if let Some(t)=scheduler.qualification_timing() {t.mark("bridge.pose.selected",0);}
                let Some(input) = drain_guarded_pose_input(
                    message,
                    &mut poses,
                    &mut diagnostics,
                    source_max_age,
                )?
                else {
                    continue;
                };
                let source_deadline = input.source_deadline;
                #[cfg(all(test,feature="surface-recovery-qualification"))]
                if let Some(t)=scheduler.qualification_timing() {t.mark("bridge.pose.drained",input.pose.stamp.nanos);}
                let mut candidate_telemetry = telemetry.clone();
                commit_guarded_pose_telemetry(&mut candidate_telemetry, &input);
                #[cfg(all(test,feature="surface-recovery-qualification"))]
                let pose_ack=scheduler.qualification_timing().map(|t|t.span("bridge.pose.scheduler_ack",input.pose.stamp.nanos));
                let acquired_at = source_deadline.map(|deadline| {
                    deadline.checked_sub(source_max_age.expect("已配置来源期限"))
                        .ok_or_else(|| NavStackError::InvalidConfig("位姿采集单调时刻下溢".into()))
                }).transpose()?;
                let settlement = await_guarded_active_input(async {
                    match (source_deadline, acquired_at) {
                        (Some(deadline), Some(acquired_at)) => scheduler
                            .update_odometry_with_acquisition(input.pose, acquired_at, deadline).await,
                        _ => scheduler.update_odometry(input.pose).await,
                    }
                }, node, transport, scheduler, &mut cancels, active_session.as_ref(),
                    &mut cancel_generation_watermark, &mut arbiter, &telemetry).await;
                #[cfg(all(test,feature="surface-recovery-qualification"))]
                drop(pose_ack);
                let accepted = settlement.input.is_ok();
                let ordinary = match settlement.input {
                    Ok(()) => { telemetry = candidate_telemetry; Ok(()) }
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.map_pose", error) {
                        Ok(OwnerInput::Rejected(reason)) => {
                            diagnostics.reject("guarded_nav_scheduler.map_pose", &reason); Ok(())
                        }
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                };
                let cancelled = settle_guarded_input_cancel(ordinary, settlement.cancel,
                    transport, node, scheduler, &mut telemetry, &mut active_session,
                    &mut goal_buffer, &mut last_cancel_terminal, &mut publications).await?;
                if !accepted || cancelled { continue; }
                if let Some(goal) = goal_buffer.take_ready_goal() {
                    activate_or_defer_guarded_goal(
                        node,
                        transport,
                        scheduler,
                        &mut telemetry,
                        &mut active_session,
                        &mut goal_buffer,
                        *goal,
                        surface_authority.as_ref(),
                        planar_mode,
                    )
                    .await?;
                }
                let snapshot = scheduler.status();
                let planner_inputs_fresh = snapshot.planner_inputs_fresh;
                publish_guarded_tail_observation(
                    transport, node, scheduler, snapshot, &mut telemetry, Some(planner_inputs_fresh),
                    &mut publications, &mut cancels, &mut active_session, &mut goal_buffer,
                    &mut last_cancel_terminal, &mut cancel_generation_watermark, &mut arbiter,
                ).await?;
            }
            GuardedNavEvent::Costmap(message) => {
                let frame = message
                    .ok_or_else(|| NavStackError::Internal("guarded map stream closed".into()))?
                    .map_err(|error| module_error("guarded map input", error))?;
                let (decoded, received_at) = match frame {
                    #[cfg(feature="mls")]
                    GuardedMapFrame::RecentCloud(frame) => {
                        let settlement = await_guarded_active_input(
                            recent_source.consume(&frame.payload, surface_authority.as_ref(), scheduler),
                            node, transport, scheduler, &mut cancels, active_session.as_ref(),
                            &mut cancel_generation_watermark, &mut arbiter, &telemetry,
                        ).await;
                        if let Err(reason) = settlement.input {
                            diagnostics.reject("recent obstacle source", &reason);
                        }
                        settle_guarded_input_cancel(Ok(()), settlement.cancel,
                            transport, node, scheduler, &mut telemetry, &mut active_session,
                            &mut goal_buffer, &mut last_cancel_terminal, &mut publications).await?;
                        continue;
                    }
                    GuardedMapFrame::Planar(frame) => {
                        let mut source_received_at = None;
                        let body = if planar_mode {
                            let value=match nuwax_codec::lcm::planar_navigation::decode_body(&frame.payload) {
                                Ok(value)=>value,
                                Err(error)=>{diagnostics.reject("planar_navigation",&error.to_string());continue;}
                            };
                            let now=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                                .map(|d|d.as_nanos().min(i64::MAX as u128) as i64).unwrap_or(0);
                            let changed=match planar_admission.admit(&value,now) {
                                Ok(changed)=>changed,
                                Err(error)=>{diagnostics.reject("planar_navigation",&error);continue;}
                            };
                            source_received_at = Instant::now().checked_sub(Duration::from_nanos(now.saturating_sub(value.source_timestamp_ns).max(0) as u64));
                            let authority=GuardedMapAuthority {
                                committed_map:schema::CommittedMapIdentity {
                                    session_id:value.session_id.clone(),session_generation:value.session_generation,
                                    revision:value.map_revision,commit_boot_nonce:value.map_boot_nonce,
                                    commit_sequence:value.map_commit_sequence,source_timestamp_ns:value.map_source_timestamp_ns,
                                    frame_id:value.grid.header.frame_id.clone(),
                                },
                                producer_runtime_generation:value.runtime_generation,
                                map_frame_generation:value.map_frame_generation,cloud_revision:0,
                                cloud_occupancy_revision:0,cloud_commit_sequence:0,cloud_source_timestamp_ns:0,
                                commit_boot_nonce:value.map_boot_nonce,
                            };
                            if changed || value.cleared {
                                // 切域的同一输入共享来源时间基准，避免首帧被自己的处理时刻拒绝。
                                // 显式清空继续建立当前水位，清空前采样不能迟到恢复执行。
                                let invalidated_at=if value.cleared {Instant::now()} else {source_received_at.unwrap_or_else(Instant::now)};
                                scheduler.invalidate_map_received("二维导航来源已撤销或切换",tokio::time::Instant::from_std(invalidated_at),
                                    nuwax_nav_runtime::MapInvalidationKind::Invalidated).await
                                    .map_err(|e|module_error("planar map invalidation",e))?;
                                goal_buffer.costmap_observed=false;
                                goal_buffer.active_goal_applied = false;
                                if value.cleared || !goal_buffer.pending_goal.as_ref().is_some_and(|goal|authority.permits_goal(goal)) {
                                    goal_buffer.pending_invalidated = true;
                                }
                                let snapshot=scheduler.status();
                                publish_guarded_nav_observation(transport,node,snapshot.clone(),&telemetry,
                                    Some(false),&mut publications).await?;
                                clear_terminal_session_after_publication(&mut active_session,&mut telemetry,&snapshot);
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
                        #[cfg(all(test,feature="surface-recovery-qualification"))]
                        if let Some(t)=scheduler.qualification_timing() {t.mark("bridge.map.prepared_received",update.source_timestamp_ns);}
                        let changed_domain = surface_authority
                            .as_ref()
                            .is_some_and(|previous| !previous.same_domain(&update.authority));
                        if changed_domain || update.snapshot.is_err() {
                            scheduler
                                .invalidate_map_received(
                                    update
                                        .snapshot
                                        .as_ref()
                                        .err()
                                        .cloned()
                                        .unwrap_or_else(|| "surface map domain changed".into()),
                                    tokio::time::Instant::from_std(update.received_at),
                                    if changed_domain || !update.awaiting_build {
                                        nuwax_nav_runtime::MapInvalidationKind::Invalidated
                                    } else {
                                        nuwax_nav_runtime::MapInvalidationKind::TemporaryUnavailable
                                    },
                                )
                                .await
                                .map_err(|error| module_error("surface map invalidation", error))?;
                            goal_buffer.costmap_observed = false;
                            goal_buffer.active_goal_applied = false;
                            // 精细源失败或换域时仍保留待执行目标；否则之后的取消
                            // 找不到原目标，几何拥有方会永久等待无法关联的停车回执。
                            if (update.snapshot.is_err() && !update.awaiting_build)
                                || (changed_domain && !goal_buffer.pending_goal.as_ref()
                                    .is_some_and(|pending| update.authority.permits_goal(pending)))
                            {
                                goal_buffer.pending_invalidated = true;
                            }
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
                            clear_terminal_session_after_publication(
                                &mut active_session,
                                &mut telemetry,
                                &snapshot,
                            );
                        }
                        if changed_domain || update.snapshot.is_err() {
                            let settlement = await_guarded_active_input(recent_source.reset(scheduler),
                                node, transport, scheduler, &mut cancels, active_session.as_ref(),
                                &mut cancel_generation_watermark, &mut arbiter, &telemetry).await;
                            let ordinary = settlement.input.map_err(|error|module_error("recent source domain reset",error));
                            settle_guarded_input_cancel(ordinary, settlement.cancel,
                                transport, node, scheduler, &mut telemetry, &mut active_session,
                                &mut goal_buffer, &mut last_cancel_terminal, &mut publications).await?;
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
                                map_frame: update.authority.committed_map.frame_id.clone(),
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
                #[cfg(all(test,feature="surface-recovery-qualification"))]
                let map_ack=scheduler.qualification_timing().map(|t|t.span("bridge.map.scheduler_ack",input.observation.source_timestamp_ns));
                let settlement = await_guarded_active_input(
                    scheduler.update_costmap_received(input.observation,
                        tokio::time::Instant::from_std(received_at.unwrap_or_else(Instant::now))),
                    node, transport, scheduler, &mut cancels, active_session.as_ref(),
                    &mut cancel_generation_watermark, &mut arbiter, &telemetry,
                ).await;
                #[cfg(all(test,feature="surface-recovery-qualification"))]
                drop(map_ack);
                let accepted = settlement.input.is_ok();
                let ordinary = match settlement.input {
                    Ok(()) => {
                        telemetry = candidate_telemetry;
                        // Observe the accepted map without taking/activating a
                        // buffered goal before an early cancellation settles.
                        goal_buffer.costmap_observed = true;
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
                let cancelled = settle_guarded_input_cancel(ordinary, settlement.cancel,
                    transport, node, scheduler, &mut telemetry, &mut active_session,
                    &mut goal_buffer, &mut last_cancel_terminal, &mut publications).await?;
                if !accepted || cancelled { continue; }
                let ready_goal = goal_buffer.take_ready_goal();
                if let Some(goal) = ready_goal {
                    activate_or_defer_guarded_goal(
                        node,
                        transport,
                        scheduler,
                        &mut telemetry,
                        &mut active_session,
                        &mut goal_buffer,
                        *goal,
                        surface_authority.as_ref(),
                        planar_mode,
                    )
                    .await?;
                }
                let snapshot = scheduler.status();
                let planner_inputs_fresh = snapshot.planner_inputs_fresh;
                publish_guarded_tail_observation(
                    transport, node, scheduler, snapshot, &mut telemetry, Some(planner_inputs_fresh),
                    &mut publications, &mut cancels, &mut active_session, &mut goal_buffer,
                    &mut last_cancel_terminal, &mut cancel_generation_watermark, &mut arbiter,
                ).await?;
            }
            GuardedNavEvent::Goal(message) => {
                let frame = required_frame(message, "guarded_nav_scheduler.goal")?;
                let goal_envelope = match decode_configured_goal_envelope(&frame.payload, body_reference_frame_id, surface_support_z_m) {
                    Ok(envelope) => envelope,
                    Err(error) => {
                        // 契约完整但本机配置拒绝的目标仍属于原请求。保留不可执行
                        // 的取消身份，避免上层已受理后永远得不到对应的归零回执。
                        let mut rejected_telemetry = telemetry.clone();
                        if active_session.is_none() {
                            if let Ok(goal) = schema::decode_goal_envelope_body(&frame.payload) {
                                if pose_from_schema(&goal.goal)
                                    .and_then(|pose| require_map_pose(&pose, "rejected goal")).is_ok()
                                    && !matches!(goal_buffer.observe_goal(goal.clone()), PlannerGoalIngress::Rejected(_))
                                {
                                    goal_buffer.pending_goal = Some(goal.clone());
                                    goal_buffer.pending_invalidated = true;
                                    last_cancel_terminal = None;
                                    rejected_telemetry = guarded_buffered_cancel_telemetry(&telemetry, &goal, 0);
                                }
                            }
                        }
                        publish_guarded_rejection(
                            transport,
                            &node.outputs[1],
                            scheduler.status(),
                            &rejected_telemetry,
                            "goal",
                            &format!("GoalEnvelope rejected: {error}"),
                        )
                        .await?;
                        continue;
                    }
                };
                if let Err(error) = pose_from_schema(&goal_envelope.goal)
                    .and_then(|goal| require_map_pose(&goal, "guarded_nav_scheduler.goal"))
                {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "goal",
                        &error.to_string(),
                    )
                    .await?;
                    continue;
                }
                let goal_envelope = match goal_buffer.observe_goal(goal_envelope) {
                    PlannerGoalIngress::Ready(goal) => {
                        last_cancel_terminal = None;
                        *goal
                    }
                    PlannerGoalIngress::Buffered => {
                        last_cancel_terminal = None;
                        if preempt_guarded_active_session(
                            node,
                            transport,
                            scheduler,
                            &mut telemetry,
                            &mut active_session,
                        )
                        .await?
                        {
                            publish_guarded_nav_observation(
                                transport,
                                node,
                                scheduler.status(),
                                &telemetry,
                                None,
                                &mut publications,
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
                        )
                        .await?;
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
                    surface_authority.as_ref(),
                    planar_mode,
                )
                .await?;
            }
            GuardedNavEvent::Path(message) => {
                let frame = required_frame(message, "guarded_nav_scheduler.path")?;
                let Some(active_goal) = active_session.as_ref() else {
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
                if !goal_buffer.has_applied_goal() {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "active goal lease is fenced by a newer goal",
                    )
                    .await?;
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
                if !planner_generation_matches_authority(
                    path_envelope.planner_runtime_generation,
                    authoritative_planner_generation,
                ) {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "PathEnvelope planner generation does not match the active nav-stack runtime",
                    )
                    .await?;
                    continue;
                }
                if planner_generation_is_stale(
                    accepted_planner_generation,
                    active_goal.goal_key(),
                    path_envelope.planner_runtime_generation,
                ) {
                    publish_guarded_rejection(
                        transport,
                        &node.outputs[1],
                        scheduler.status(),
                        &telemetry,
                        "path",
                        "PathEnvelope planner generation is stale for the active goal",
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
                let active_goal_key = active_goal.goal_key();
                let settlement = await_guarded_active_input(
                    scheduler
                    .submit_path(nuwax_nav_runtime::VersionedPath {
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
                    }),
                    node, transport, scheduler, &mut cancels, Some(active_goal),
                    &mut cancel_generation_watermark, &mut arbiter, &telemetry,
                ).await;
                let rejected = settlement.input.is_err();
                let ordinary = match settlement.input {
                    Err(error) => match runtime_input_error::<()>("guarded_nav_scheduler.path", error) {
                        Ok(OwnerInput::Rejected(reason)) => publish_guarded_rejection(
                            transport, &node.outputs[1], scheduler.status(), &telemetry, "path", &reason,
                        ).await,
                        Ok(OwnerInput::Accepted(())) => unreachable!("error classifier accepted an error"),
                        Err(error) => Err(error),
                    },
                    Ok(()) => {
                        if matches!(&settlement.cancel, Ok(None)) {
                            accepted_planner_generation = Some((active_goal_key,
                                path_envelope.planner_runtime_generation));
                            telemetry.path_revision = path_envelope.path_revision;
                            telemetry.planner_runtime_generation = path_envelope.planner_runtime_generation;
                        }
                        Ok(())
                    }
                };
                let cancelled = settle_guarded_input_cancel(ordinary, settlement.cancel,
                    transport, node, scheduler, &mut telemetry, &mut active_session,
                    &mut goal_buffer, &mut last_cancel_terminal, &mut publications).await?;
                if rejected && !cancelled {
                    let snapshot = scheduler.status();
                    clear_terminal_session_after_publication(
                        &mut active_session, &mut telemetry, &snapshot,
                    );
                }
            }
            GuardedNavEvent::Cancel(message) => {
                let (cancel, cancel_generation) = match decode_guarded_cancel(message)? {
                    OwnerInput::Accepted(value) => value,
                    OwnerInput::Rejected(reason) => {
                        publish_guarded_rejection(
                            transport, &node.outputs[1], scheduler.status(), &telemetry,
                            "cancel", &reason,
                        ).await?;
                        continue;
                    }
                };
                if let Some(active_goal) = active_session.clone() {
                    if let Err(reason) = admit_guarded_active_cancel(
                        &cancel, cancel_generation, &active_goal, &mut cancel_generation_watermark,
                    ) {
                        publish_guarded_rejection(
                            transport, &node.outputs[1], scheduler.status(), &telemetry,
                            "cancel", &reason,
                        ).await?;
                        continue;
                    }
                    let result = execute_guarded_active_cancel(scheduler, &cancel).await;
                    finish_guarded_active_cancel(
                        transport, node, scheduler,
                        GuardedActiveCancelCompletion { active_goal, cancel, cancel_generation, result },
                        &mut telemetry, &mut active_session, &mut goal_buffer,
                        &mut last_cancel_terminal, &mut publications,
                    ).await?;
                } else {
                    let buffered_goal = match goal_buffer.cancelled_pending_goal(&cancel) {
                        Ok(goal) => goal,
                        Err(reason) => {
                            publish_guarded_rejection(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                "cancel",
                                &reason,
                            )
                            .await?;
                            continue;
                        }
                    };
                    let (goal_to_fence, goal_was_pending) = if let Some(buffered_goal) =
                        buffered_goal
                    {
                        if let Err(reason) = observe_guarded_cancel_generation(
                            &mut cancel_generation_watermark,
                            &buffered_goal,
                            cancel_generation,
                        ) {
                            publish_guarded_rejection(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                "cancel",
                                &reason,
                            )
                            .await?;
                            continue;
                        }
                        (Some(*buffered_goal), true)
                    } else if let Some(terminal) = last_cancel_terminal.as_ref() {
                        if let Err(reason) =
                            validate_cancel_goal_correlation(&cancel, &terminal.goal)
                        {
                            publish_guarded_rejection(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                "cancel",
                                &reason,
                            )
                            .await?;
                            continue;
                        }
                        if cancel_generation < terminal.cancel_generation {
                            publish_guarded_rejection(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                "cancel",
                                &format!(
                                    "stale cancel generation {cancel_generation}; latest terminal generation is {}",
                                    terminal.cancel_generation
                                ),
                            ).await?;
                            continue;
                        }
                        if let Err(reason) = observe_guarded_cancel_generation(
                            &mut cancel_generation_watermark,
                            &terminal.goal,
                            cancel_generation,
                        ) {
                            publish_guarded_rejection(
                                transport,
                                &node.outputs[1],
                                scheduler.status(),
                                &telemetry,
                                "cancel",
                                &reason,
                            )
                            .await?;
                            continue;
                        }
                        if cancel_generation == terminal.cancel_generation {
                            publish_bool(transport, &node.outputs[3], true).await?;
                            publish_guarded_nav_state(
                                transport,
                                &node.outputs[1],
                                terminal.status.clone(),
                                &terminal.telemetry,
                                None,
                            )
                            .await?;
                            continue;
                        }
                        (Some(terminal.goal.clone()), false)
                    } else {
                        (None, false)
                    };
                    if let Some(buffered_goal) = goal_to_fence {
                        let buffered_telemetry = guarded_buffered_cancel_telemetry(
                            &telemetry,
                            &buffered_goal,
                            cancel_generation,
                        );
                        let session_id = buffered_goal.session_id.clone();
                        let session = nuwax_nav_runtime::NavSession {
            boundary: buffered_goal.boundary_points.iter().map(|p|[p[0],p[1]]).collect(),
            options: buffered_goal.options,
                            goal: nuwax_nav_runtime::ExecutionGoal::Body(pose_from_schema(&buffered_goal.goal)?),
                            session_id: session_id.clone(),
                            execution_identity: guarded_goal_execution_identity(&buffered_goal),

                            motion_admission: nuwax_nav_runtime::NavMotionAdmission {

                                guard_boot_nonce: buffered_goal.motion_guard_boot_nonce,
                                motion_stop_generation: u64::try_from(
                                    buffered_goal.motion_stop_generation,
                                )
                                .expect("validated non-negative motion_stop_generation"),
                            },
                        };
                        if let Err(error) = scheduler.start_session(session).await {
                            if matches!(error, nuwax_nav_runtime::RuntimeError::LeaseNotGranted(_)) {
                                publish_guarded_nav_state(
                                    transport, &node.outputs[1], scheduler.status(),
                                    &buffered_telemetry, None,
                                ).await?;
                                goal_buffer.clear_pending_goal_for(&buffered_goal);
                                continue;
                            }
                            match runtime_input_error::<()>(
                                "guarded_nav_scheduler.buffered_cancel_acquire",
                                error,
                            )? {
                                OwnerInput::Rejected(reason) => {
                                    publish_guarded_rejection(
                                        transport,
                                        &node.outputs[1],
                                        scheduler.status(),
                                        &buffered_telemetry,
                                        "cancel",
                                        &reason,
                                    )
                                    .await?;
                                    continue;
                                }
                                OwnerInput::Accepted(()) => {
                                    unreachable!("error classifier accepted an error")
                                }
                            }
                        }
                        if goal_was_pending && !goal_buffer.clear_pending_goal_for(&buffered_goal) {
                            return Err(NavStackError::Internal(
                                "guarded scheduler acquired a superseded buffered cancellation goal"
                                    .into(),
                            ));
                        }
                        active_session = Some(buffered_goal.clone());
                        telemetry = buffered_telemetry;
                        if let Err(error) = scheduler.cancel(&session_id).await {
                            match runtime_input_error::<()>(
                                "guarded_nav_scheduler.buffered_cancel",
                                error,
                            )? {
                                OwnerInput::Rejected(reason) => {
                                    publish_guarded_rejection(
                                        transport,
                                        &node.outputs[1],
                                        scheduler.status(),
                                        &telemetry,
                                        "cancel",
                                        &reason,
                                    )
                                    .await?;
                                    let snapshot = scheduler.status();
                                    clear_terminal_session_after_publication(
                                        &mut active_session,
                                        &mut telemetry,
                                        &snapshot,
                                    );
                                    continue;
                                }
                                OwnerInput::Accepted(()) => {
                                    unreachable!("error classifier accepted an error")
                                }
                            }
                        }
                        publish_bool(transport, &node.outputs[3], true).await?;
                        let snapshot = scheduler.status();
                        publish_guarded_nav_observation(
                            transport,
                            node,
                            snapshot.clone(),
                            &telemetry,
                            None,
                            &mut publications,
                        )
                        .await?;
                        last_cancel_terminal = Some(GuardedCancelTerminal {
                            goal: buffered_goal,
                            cancel_generation,
                            status: snapshot.clone(),
                            telemetry: telemetry.clone(),
                        });
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
                            "no active or buffered scheduler session",
                        )
                        .await?;
                    }
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
    surface_authority: Option<&GuardedMapAuthority>,
    planar_mode:bool,
) -> NavStackResult<()> {
    if (node.id == "surface_nav_scheduler" || planar_mode)
        && !surface_authority.is_some_and(|map| map.permits_goal(&goal_envelope))
    {
        goal_buffer.costmap_observed = false;
        goal_buffer.defer_ready_goal(goal_envelope);
        return Ok(());
    }

    if !guarded_goal_inputs_ready(telemetry, scheduler, &goal_envelope) {
        if !goal_buffer.defer_ready_goal(goal_envelope) {
            return Err(NavStackError::Internal(
                "guarded scheduler could not retain a not-yet-ready GoalEnvelope".into(),
            ));
        }
        return Ok(());
    }

    let applied_goal = goal_envelope.clone();
    match activate_guarded_goal(
        node,
        transport,
        scheduler,
        telemetry,
        active_session,
        goal_envelope,
    )
    .await? {
        GuardedGoalActivation::Activated => {
            if !goal_buffer.mark_applied(&applied_goal) {
                return Err(NavStackError::Internal(
                    "guarded scheduler applied a superseded GoalEnvelope".into(),
                ));
            }
        }
        GuardedGoalActivation::Deferred => {
            if !goal_buffer.defer_ready_goal(applied_goal) {
                return Err(NavStackError::Internal(
                    "guarded scheduler lost a GoalEnvelope after deferred activation".into(),
                ));
            }
        }
        GuardedGoalActivation::LeaseRejected => {
            goal_buffer.clear_pending_goal_for(&applied_goal);
        }
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
enum GuardedGoalActivation {
    Activated,
    Deferred,
    LeaseRejected,
}

#[cfg(feature = "guard-runtime")]
async fn activate_guarded_goal(
    node: &ModuleNode,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
    goal_envelope: schema::GoalEnvelope,
) -> NavStackResult<GuardedGoalActivation> {
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
            return Ok(GuardedGoalActivation::Deferred);
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
        return Ok(GuardedGoalActivation::Deferred);
    }

    if !preempt_guarded_active_session(node, transport, scheduler, telemetry, active_session)
        .await?
    {
        return Ok(GuardedGoalActivation::Deferred);
    }

    let mut candidate_telemetry = telemetry.clone();
    candidate_telemetry.goal = [
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.orientation.yaw(),
    ];
    candidate_telemetry.goal_z = goal.pose.position.z;
    candidate_telemetry.public_session_id = goal_envelope.session_id.clone();
    candidate_telemetry.owner_kind = goal_envelope.owner_kind;
    candidate_telemetry.parent_request_id = goal_envelope.parent_request_id.clone();
    candidate_telemetry.goal_generation = goal_envelope.goal_generation;
    candidate_telemetry.producer_runtime_generation = goal_envelope.producer_runtime_generation;
    candidate_telemetry.cancel_generation = 0;
    candidate_telemetry.path_revision = 0;
    candidate_telemetry.planner_runtime_generation = 0;
    candidate_telemetry.initial_distance = telemetry_distance(&candidate_telemetry);
    candidate_telemetry.progress = 0.0;
    if let Err(error) = scheduler
        .start_session(nuwax_nav_runtime::NavSession {
            boundary: goal_envelope.boundary_points.iter().map(|p|[p[0],p[1]]).collect(),
            options: goal_envelope.options,
            goal: nuwax_nav_runtime::ExecutionGoal::Body(pose_from_schema(&goal_envelope.goal)?),
            session_id: goal_envelope.session_id.clone(),
            execution_identity: guarded_goal_execution_identity(&goal_envelope),

            motion_admission: nuwax_nav_runtime::NavMotionAdmission {

                guard_boot_nonce: goal_envelope.motion_guard_boot_nonce,
                motion_stop_generation: u64::try_from(goal_envelope.motion_stop_generation)
                    .expect("validated non-negative motion_stop_generation"),
            },
        })
        .await
    {
        if matches!(error, nuwax_nav_runtime::RuntimeError::LeaseNotGranted(_)) {
            // 明确未授予的拒绝属于本次候选目标，不能发布旧目标或空闲心跳。
            publish_guarded_nav_state(
                transport, &node.outputs[1], scheduler.status(), &candidate_telemetry, None,
            ).await?;
            return Ok(GuardedGoalActivation::LeaseRejected);
        }
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
                return Ok(GuardedGoalActivation::Deferred);
            }
            OwnerInput::Accepted(()) => unreachable!("error classifier accepted an error"),
        }
    }
    *telemetry = candidate_telemetry;
    *active_session = Some(goal_envelope);
    Ok(GuardedGoalActivation::Activated)
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
    telemetry.goal_z = 0.0;
    telemetry.producer_runtime_generation = 0;
    telemetry.path_revision = 0;
    telemetry.planner_runtime_generation = 0;
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
fn planner_generation_matches_authority(candidate: i64, authoritative: i64) -> bool {
    candidate == authoritative
}

#[cfg(feature = "guard-runtime")]
fn planner_generation_is_stale(
    accepted: Option<(GoalKey, i64)>,
    active_goal_key: GoalKey,
    candidate_generation: i64,
) -> bool {
    accepted.is_some_and(|(goal_key, generation)| {
        goal_key == active_goal_key && candidate_generation < generation
    })
}

#[cfg(feature = "guard-runtime")]
fn validate_path_goal_correlation(path: &schema::PathEnvelope, goal: &schema::GoalEnvelope) -> Result<(), String> {
    if path.session_id != goal.session_id || path.goal_key() != goal.goal_key() {
        return Err("PathEnvelope goal identity does not match the active GoalEnvelope".into());
    }
    if path.path.header.frame_id != goal.goal.header.frame_id {
        return Err("PathEnvelope frame does not match the active GoalEnvelope".into());
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

#[cfg(feature = "guard-runtime")]
fn guarded_buffered_cancel_telemetry(
    telemetry: &GuardedNavTelemetry,
    goal: &schema::GoalEnvelope,
    cancel_generation: u64,
) -> GuardedNavTelemetry {
    let goal_pose =
        pose_from_schema(&goal.goal).expect("buffered goal pose was validated on ingress");
    let mut candidate = telemetry.clone();
    candidate.public_session_id = goal.session_id.clone();
    candidate.owner_kind = goal.owner_kind;
    candidate.parent_request_id = goal.parent_request_id.clone();
    candidate.goal_generation = goal.goal_generation;
    candidate.cancel_generation = cancel_generation;
    candidate.goal = [
        goal_pose.pose.position.x,
        goal_pose.pose.position.y,
        goal_pose.pose.orientation.yaw(),
    ];
    candidate.producer_runtime_generation = goal.producer_runtime_generation;
    candidate.path_revision = 0;
    candidate.planner_runtime_generation = 0;
    candidate.goal_z = goal_pose.pose.position.z;
    candidate.initial_distance = telemetry_distance(&candidate);
    candidate.progress = 0.0;
    candidate
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
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    qualification_timing: Option<nuwax_nav_runtime::qualification_timing::TraceSink>,
    goal: Option<(String, i32, String, i64, i64)>,
    applied_commands: u64,
    last_zero_ack_id: Option<String>,
    reached_published: bool,
}

#[cfg(feature = "guard-runtime")]
impl GuardedStatusPublications {
    fn frames(
        &mut self,
        node: &ModuleNode,
        status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry,
        planner_observation: Option<bool>,
    ) -> NavStackResult<Vec<RawLcmFrame>> {
        let mut frames = Vec::with_capacity(4);
        if !telemetry.public_session_id.is_empty() {
            let goal = (
                telemetry.public_session_id.clone(),
                telemetry.owner_kind,
                telemetry.parent_request_id.clone(),
                telemetry.goal_generation,
                telemetry.producer_runtime_generation,
            );
            if self.goal.as_ref() != Some(&goal) {
                self.goal = Some(goal);
                self.applied_commands = 0;
                self.last_zero_ack_id = None;
                self.reached_published = false;
            }
            if status.applied_commands > self.applied_commands {
                if let Some(twist) = status.last_applied_twist {
                    frames.push(build_twist2_frame(&node.outputs[0], twist)?);
                }
                self.applied_commands = status.applied_commands;
            }
            if status.last_zero_ack_id != self.last_zero_ack_id {
                if status.last_zero_ack_id.is_some() {
                    frames.push(build_twist2_frame(&node.outputs[0], Twist2::ZERO)?);
                }
                self.last_zero_ack_id = status.last_zero_ack_id.clone();
            }
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
fn decode_guarded_cancel(
    message: Option<TransportResult<RawLcmFrame>>,
) -> OwnerInputResult<(schema::CancelNavigation, u64)> {
    let frame = required_frame(message, "guarded_nav_scheduler.cancel")?;
    let cancel = match schema::decode_cancel_navigation_body(&frame.payload) {
        Ok(cancel) => cancel,
        Err(error) => return Ok(OwnerInput::Rejected(format!("CancelNavigation rejected: {error}"))),
    };
    #[cfg(all(test, feature = "surface-recovery-qualification"))]
    guarded_tests::record_ack_return_cancel("bridge_ack_test.cancel_decoded", &cancel);
    let Some(generation) = u64::try_from(cancel.cancel_generation).ok().filter(|value| *value > 0) else {
        return Ok(OwnerInput::Rejected("CancelNavigation.cancel_generation must be positive".into()));
    };
    Ok(OwnerInput::Accepted((cancel, generation)))
}

#[cfg(feature = "guard-runtime")]
fn admit_guarded_active_cancel(
    cancel: &schema::CancelNavigation,
    generation: u64,
    active_goal: &schema::GoalEnvelope,
    watermark: &mut Option<GuardedCancelGenerationWatermark>,
) -> Result<(), String> {
    validate_cancel_goal_correlation(cancel, active_goal)?;
    observe_guarded_cancel_generation(watermark, active_goal, generation)
}

#[cfg(feature = "guard-runtime")]
async fn execute_guarded_active_cancel(
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    cancel: &schema::CancelNavigation,
) -> Result<(), nuwax_nav_runtime::RuntimeError> {
    #[cfg(all(test, feature = "surface-recovery-qualification"))]
    guarded_tests::record_ack_return_cancel("bridge_ack_test.cancel_api_begin", cancel);
    let result = scheduler.cancel(&cancel.session_id).await;
    #[cfg(all(test, feature = "surface-recovery-qualification"))]
    guarded_tests::record_ack_return_cancel("bridge_ack_test.cancel_api_done", cancel);
    #[cfg(all(test, feature = "surface-recovery-qualification"))]
    if result.is_ok() {
        guarded_tests::record_ack_return_cancel("bridge_ack_test.cancel_api_ok", cancel);
    }
    result
}

#[cfg(feature = "guard-runtime")]
struct GuardedActiveCancelCompletion {
    active_goal: schema::GoalEnvelope,
    cancel: schema::CancelNavigation,
    cancel_generation: u64,
    result: Result<(), nuwax_nav_runtime::RuntimeError>,
}

#[cfg(feature = "guard-runtime")]
struct GuardedInputSettlement<T> {
    input: T,
    cancel: NavStackResult<Option<GuardedActiveCancelCompletion>>,
}

// One ordinary request remains owned from its bounded send through its ACK.
// Only an already-active goal is eligible: a queued StartSession could acquire
// after a cancellation and must never be passed to this helper.
#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn await_guarded_active_input<P, T>(
    input: P,
    node: &ModuleNode,
    transport: &LcmTransport,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    cancels: &mut LcmFrameStream,
    active_goal: Option<&schema::GoalEnvelope>,
    watermark: &mut Option<GuardedCancelGenerationWatermark>,
    arbiter: &mut GuardedNavArbiter,
    telemetry: &GuardedNavTelemetry,
) -> GuardedInputSettlement<T>
where
    P: Future<Output = T>,
{
    let eligible = {
        // Borrow the existing watch value; a pose ACK must not clone local_path
        // merely to inspect these two scalar/session fields.
        let status = scheduler.subscribe_status();
        let snapshot = status.borrow();
        active_goal.filter(|goal| snapshot.last_terminal.is_none()
            && snapshot.session_id.as_deref() == Some(goal.session_id.as_str()))
    };
    let Some(active_goal) = eligible else {
        return GuardedInputSettlement { input: input.await, cancel: Ok(None) };
    };
    tokio::pin!(input);
    loop {
        let message = tokio::select! {
            biased;
            result = &mut input => return GuardedInputSettlement { input: result, cancel: Ok(None) },
            message = cancels.next() => message,
        };
        arbiter.record_dispatch(GuardedNavSource::Cancel);
        let decoded = decode_guarded_cancel(message).map(|decoded| match decoded {
            OwnerInput::Accepted((cancel, generation)) => {
                match admit_guarded_active_cancel(&cancel, generation, active_goal, watermark) {
                    Ok(()) => OwnerInput::Accepted((cancel, generation)),
                    Err(reason) => OwnerInput::Rejected(reason),
                }
            }
            OwnerInput::Rejected(reason) => OwnerInput::Rejected(reason),
        });
        let fatal = match decoded {
            Ok(OwnerInput::Accepted((cancel, cancel_generation))) => {
                let (input, result) = tokio::join!(
                    &mut input, execute_guarded_active_cancel(scheduler, &cancel),
                );
                return GuardedInputSettlement {
                    input,
                    cancel: Ok(Some(GuardedActiveCancelCompletion {
                        active_goal: active_goal.clone(), cancel, cancel_generation, result,
                    })),
                };
            }
            Ok(OwnerInput::Rejected(reason)) => {
                // Preserve the original diagnostic and its transport backpressure.
                // A rejected head must not leave a later valid Cancel waiting
                // for the blocked ordinary ACK.
                publish_guarded_rejection(
                    transport, &node.outputs[1], scheduler.status(), telemetry, "cancel", &reason,
                ).await.err()
            }
            Err(error) => Some(error),
        };
        if let Some(error) = fatal {
            // Returning early would discard a possibly-enqueued request; merely
            // awaiting it could prevent outer cleanup behind a pending renew.
            // The original urgent shutdown closes the actor and its input queue.
            let (input, shutdown) = tokio::join!(&mut input, scheduler.shutdown());
            if let Err(shutdown_error) = shutdown {
                tracing::warn!(ingress_error = %error, shutdown_error = %shutdown_error,
                    "guarded input cancellation error during shutdown settlement");
            }
            return GuardedInputSettlement { input, cancel: Err(error) };
        }
    }
}

// Input bookkeeping/diagnostics have completed, but no goal has been activated
// and no terminal identity has been cleared. Keep both completed results even
// when the ordinary input failed; never let an early `?` discard cancellation.
#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn settle_guarded_input_cancel(
    ordinary: NavStackResult<()>,
    cancel: NavStackResult<Option<GuardedActiveCancelCompletion>>,
    transport: &LcmTransport,
    node: &ModuleNode,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
    goal_buffer: &mut PlannerGoalBuffer,
    last_cancel_terminal: &mut Option<GuardedCancelTerminal>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<bool> {
    let completion = match cancel {
        Ok(completion) => completion,
        Err(error) => {
            if let Err(input_error) = ordinary {
                tracing::warn!(ingress_error = %error, input_error = %input_error,
                    "guarded input also failed during fatal ingress settlement");
            }
            return Err(error);
        }
    };
    let interrupted = completion.is_some();
    let cancelled = match completion {
        Some(completion) => finish_guarded_active_cancel(
            transport, node, scheduler, completion, telemetry, active_session,
            goal_buffer, last_cancel_terminal, publications,
        ).await,
        None => Ok(()),
    };
    if let Err(error) = ordinary {
        if let Err(cancel_error) = cancelled {
            tracing::warn!(input_error = %error, cancel_error = %cancel_error,
                "ordinary guarded input failed with cancellation settlement failure");
        }
        return Err(error);
    }
    cancelled?;
    Ok(interrupted)
}

// This is one publication tail, not a second ingress owner. An eligible active
// goal is borrowed until a Cancel is actually read; the no-Cancel path clones none.
#[cfg(feature = "guard-runtime")]
async fn await_guarded_tail_publication<P>(
    publication: P,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    cancels: &mut LcmFrameStream,
    active_goal: Option<&schema::GoalEnvelope>,
    watermark: &mut Option<GuardedCancelGenerationWatermark>,
    arbiter: &mut GuardedNavArbiter,
) -> NavStackResult<Option<OwnerInput<GuardedActiveCancelCompletion>>>
where
    P: Future<Output = NavStackResult<()>>,
{
    tokio::pin!(publication);
    let (publication_result, pending) = if let Some(active_goal) = active_goal {
        tokio::select! {
            biased;
            result = &mut publication => (result, None),
            message = cancels.next() => {
                arbiter.record_dispatch(GuardedNavSource::Cancel);
                match decode_guarded_cancel(message) {
                    Err(error) => (publication.await, Some(Err(error))),
                    Ok(OwnerInput::Rejected(reason)) => (
                        publication.await, Some(Ok(OwnerInput::Rejected(reason))),
                    ),
                    Ok(OwnerInput::Accepted((cancel, cancel_generation))) => {
                        if let Err(reason) = admit_guarded_active_cancel(
                            &cancel, cancel_generation, active_goal, watermark,
                        ) {
                            (publication.await, Some(Ok(OwnerInput::Rejected(reason))))
                        } else {
                            // Preserve both futures and both Results, whichever completes first.
                            // No shutdown branch may discard an already admitted cancellation.
                            let (published, result) = tokio::join!(
                                &mut publication,
                                execute_guarded_active_cancel(scheduler, &cancel),
                            );
                            (published, Some(Ok(OwnerInput::Accepted(GuardedActiveCancelCompletion {
                                active_goal: active_goal.clone(), cancel, cancel_generation, result,
                            }))))
                        }
                    }
                }
            }
        }
    } else {
        (publication.await, None)
    };
    if let Err(error) = publication_result {
        match &pending {
            Some(Ok(OwnerInput::Accepted(completion))) => {
                // One failure event, never the rate-limited rejection diagnostic.
                // The unchanged outer coordinator still owns shutdown/task error precedence.
                tracing::warn!(
                    publication_error = %error, cancel_result = ?completion.result,
                    session_id = %completion.cancel.session_id,
                    owner_kind = completion.cancel.owner_kind,
                    parent_request_id = %completion.cancel.parent_request_id,
                    goal_generation = completion.cancel.goal_generation,
                    producer_runtime_generation = completion.cancel.producer_runtime_generation,
                    cancel_generation = completion.cancel_generation,
                    "ordinary publication failed after early active cancellation",
                );
            }
            Some(Ok(OwnerInput::Rejected(reason))) => {
                tracing::warn!(
                    publication_error = %error, cancel_rejection = %reason,
                    "ordinary publication failed after consuming rejected cancellation input",
                );
            }
            Some(Err(input_error)) => {
                tracing::warn!(
                    publication_error = %error, cancel_input_error = %input_error,
                    "ordinary publication failed after consuming failed cancellation input",
                );
            }
            None => {}
        }
        return Err(error);
    }
    pending.transpose()
}

#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn publish_guarded_tail_observation(
    transport: &LcmTransport,
    node: &ModuleNode,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    snapshot: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &mut GuardedNavTelemetry,
    planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
    cancels: &mut LcmFrameStream,
    active_session: &mut Option<schema::GoalEnvelope>,
    goal_buffer: &mut PlannerGoalBuffer,
    last_cancel_terminal: &mut Option<GuardedCancelTerminal>,
    watermark: &mut Option<GuardedCancelGenerationWatermark>,
    arbiter: &mut GuardedNavArbiter,
) -> NavStackResult<()> {
    let pending = {
        let eligible = active_session.as_ref().filter(|goal| {
            snapshot.last_terminal.is_none()
                && snapshot.session_id.as_deref() == Some(goal.session_id.as_str())
        });
        let publication = publish_guarded_nav_observation(
            transport, node, snapshot.clone(), telemetry, planner_observation, publications,
        );
        await_guarded_tail_publication(
            publication, scheduler, cancels, eligible, watermark, arbiter,
        ).await?
    };
    // Keep the original snapshot. An early C may have made the scheduler terminal,
    // but clearing with that newer status here would erase the Cancel's identity.
    clear_terminal_session_after_publication(active_session, telemetry, &snapshot);
    match pending {
        None => Ok(()),
        Some(OwnerInput::Rejected(reason)) => {
            publish_guarded_rejection(
                transport, &node.outputs[1], scheduler.status(), telemetry, "cancel", &reason,
            ).await
        }
        Some(OwnerInput::Accepted(completion)) => {
            finish_guarded_active_cancel(
                transport, node, scheduler, completion, telemetry, active_session,
                goal_buffer, last_cancel_terminal, publications,
            ).await
        }
    }
}

#[cfg(feature = "guard-runtime")]
#[allow(clippy::too_many_arguments)]
async fn finish_guarded_active_cancel(
    transport: &LcmTransport,
    node: &ModuleNode,
    scheduler: &nuwax_nav_runtime::NavSchedulerHandle,
    completion: GuardedActiveCancelCompletion,
    telemetry: &mut GuardedNavTelemetry,
    active_session: &mut Option<schema::GoalEnvelope>,
    goal_buffer: &mut PlannerGoalBuffer,
    last_cancel_terminal: &mut Option<GuardedCancelTerminal>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    let GuardedActiveCancelCompletion { active_goal, cancel, cancel_generation, result } = completion;
    let mut candidate_telemetry = telemetry.clone();
    candidate_telemetry.cancel_generation = cancel_generation;
    candidate_telemetry.public_session_id = cancel.session_id.clone();
    candidate_telemetry.owner_kind = cancel.owner_kind;
    candidate_telemetry.parent_request_id = cancel.parent_request_id.clone();
    candidate_telemetry.goal_generation = cancel.goal_generation;
    candidate_telemetry.path_revision = 0;
    candidate_telemetry.planner_runtime_generation = 0;
    if let Err(error) = result {
        match runtime_input_error::<()>("guarded_nav_scheduler.cancel", error)? {
            OwnerInput::Rejected(reason) => {
                publish_guarded_rejection(
                    transport,
                    &node.outputs[1],
                    scheduler.status(),
                    telemetry,
                    "cancel",
                    &reason,
                )
                .await?;
                let snapshot = scheduler.status();
                clear_terminal_session_after_publication(
                    active_session,
                    telemetry,
                    &snapshot,
                );
                return Ok(());
            }
            OwnerInput::Accepted(()) => {
                unreachable!("error classifier accepted an error")
            }
        }
    }
    *telemetry = candidate_telemetry;
    publish_bool(transport, &node.outputs[3], true).await?;
    let snapshot = scheduler.status();
    publish_guarded_nav_observation(
        transport,
        node,
        snapshot.clone(),
        telemetry,
        None,
        publications,
    )
    .await?;
    goal_buffer.clear_pending_goal_for(&active_goal);
    *last_cancel_terminal =
        goal_buffer
            .is_newest_goal(&active_goal)
            .then(|| GuardedCancelTerminal {
                goal: active_goal,
                cancel_generation,
                status: snapshot.clone(),
                telemetry: telemetry.clone(),
            });
    clear_terminal_session_after_publication(
        active_session,
        telemetry,
        &snapshot,
    );
    Ok(())
}

#[cfg(feature = "guard-runtime")]
async fn publish_guarded_nav_observation(
    transport: &LcmTransport,
    node: &ModuleNode,
    status: nuwax_nav_runtime::SchedulerStatus,
    telemetry: &GuardedNavTelemetry,
    planner_observation: Option<bool>,
    publications: &mut GuardedStatusPublications,
) -> NavStackResult<()> {
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    let encode=publications.qualification_timing.as_ref().map(|t|t.span("bridge.observation.encode",status.costmap_source_timestamp_ns));
    let frames=publications.frames(node, &status, telemetry, planner_observation)?;
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    drop(encode);
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    if let Some(t)=publications.qualification_timing.as_ref() {t.count("bridge.observation.bytes",status.costmap_source_timestamp_ns,frames.iter().map(|f|f.payload.len() as u64).sum());}
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    let fixture_copy=publications.qualification_timing.as_ref().map(|t|t.span("bridge.observation.fixture_receipt_copy",status.costmap_source_timestamp_ns));
    #[cfg(test)]
    let receipts=frames.iter().map(|frame|frame.payload.to_vec()).collect::<Vec<_>>();
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    drop(fixture_copy);
    #[cfg(all(test, feature = "surface-recovery-qualification"))]
    guarded_tests::hold_test_publication_before_delegate(
        &status, telemetry, planner_observation, &frames,
    ).await;
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    let publish=publications.qualification_timing.as_ref().map(|t|t.span("bridge.observation.publish_ack",status.costmap_source_timestamp_ns));
    transport
        .publish_batch(frames)
        .await
        .map_err(|error| NavStackError::Internal(format!("publish guard status batch: {error}")))?;
    #[cfg(all(test,feature="surface-recovery-qualification"))]
    drop(publish);
    #[cfg(all(test, feature = "surface-recovery-qualification"))]
    guarded_tests::hold_test_publication_ack(&status, telemetry, planner_observation).await;
    #[cfg(test)]
    guarded_tests::record_test_publication(&receipts);
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
    build_body_frame(channel, "nuwax_msgs.NavigationState", body)
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
    let message = if !has_public_session && !status.message.starts_with("REJECTED ") {
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
        producer_runtime_generation: telemetry.producer_runtime_generation,
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
        nuwax_nav_runtime::SchedulerPhase::Cancelling|nuwax_nav_runtime::SchedulerPhase::Fencing|nuwax_nav_runtime::SchedulerPhase::Failed)) {
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
    pending_invalidated: bool,
    pending_goal: Option<schema::GoalEnvelope>,
    newest_goal: Option<schema::GoalEnvelope>,
    active_goal_applied: bool,
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
        self.active_goal_applied = false;
        self.pending_invalidated = false;
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
        self.take_ready_goal()
    }

    fn mark_applied(&mut self, goal: &schema::GoalEnvelope) -> bool {
        let is_current = self.is_newest_goal(goal);
        if is_current {
            self.active_goal_applied = true;
            if self.pending_goal.as_ref().is_some_and(|pending| {
                planner_goal_identity_matches(pending, goal)
            }) {
                self.pending_goal = None;
            }
        }
        is_current
    }

    fn is_newest_goal(&self, goal: &schema::GoalEnvelope) -> bool {
        self.newest_goal.as_ref().is_some_and(|newest| {
            planner_goal_identity_matches(goal, newest)
        })
    }

    fn defer_ready_goal(&mut self, goal: schema::GoalEnvelope) -> bool {
        let is_current = self.newest_goal.as_ref().is_some_and(|newest| {
            planner_goal_identity_matches(&goal, newest)
        });
        if is_current {
            self.active_goal_applied = false;
            self.pending_goal = Some(goal);
        }
        is_current
    }

    fn take_ready_goal(&mut self) -> Option<Box<schema::GoalEnvelope>> {
        if self.costmap_observed && !self.pending_invalidated {
            self.pending_goal.take().map(Box::new)
        } else {
            None
        }
    }

    fn has_applied_goal(&self) -> bool {
        self.active_goal_applied
    }

    #[cfg(feature = "simple-2d")]
    fn clear_applied_goal(&mut self) {
        self.active_goal_applied = false;
    }

    fn cancelled_pending_goal(
        &self,
        cancel: &schema::CancelNavigation,
    ) -> Result<Option<Box<schema::GoalEnvelope>>, String> {
        let Some(pending) = self.pending_goal.as_ref() else {
            return Ok(None);
        };
        validate_cancel_goal_correlation(cancel, pending)?;
        Ok(Some(Box::new(pending.clone())))
    }

    fn clear_pending_goal_for(&mut self, goal: &schema::GoalEnvelope) -> bool {
        let matches = self.pending_goal.as_ref().is_some_and(|pending| {
            planner_goal_identity_matches(pending, goal)
        });
        if matches {
            self.pending_goal = None;
            self.active_goal_applied = false;
        }
        matches
    }
}

#[cfg(feature = "simple-2d")]
fn clear_simple_planner_active_projection(
    goal_buffer: &mut PlannerGoalBuffer,
    active_goal: &mut Option<schema::GoalEnvelope>,
    active_boundary: &mut Option<NavigationBoundary>,
    path_revision: &mut i64,
    published_path_is_boundary_safe: &mut bool,
) {
    active_goal.take();
    active_boundary.take();
    *path_revision = 0;
    *published_path_is_boundary_safe = false;
    goal_buffer.clear_applied_goal();
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
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
fn planner_goal_identity_matches(
    left: &schema::GoalEnvelope,
    right: &schema::GoalEnvelope,
) -> bool {
    left.session_id == right.session_id
        && left.owner_kind == right.owner_kind
        && left.parent_request_id == right.parent_request_id
        && left.goal_generation == right.goal_generation
        && left.producer == right.producer
        && left.producer_epoch == right.producer_epoch
        && left.producer_runtime_generation == right.producer_runtime_generation
        && left.committed_map_session_id == right.committed_map_session_id
        && left.committed_map_session_generation == right.committed_map_session_generation
        && left.committed_map_revision == right.committed_map_revision
        && left.committed_map_boot_nonce == right.committed_map_boot_nonce
        && left.committed_map_commit_sequence == right.committed_map_commit_sequence
        && left.committed_map_source_timestamp_ns == right.committed_map_source_timestamp_ns
        && left.committed_map_frame_id == right.committed_map_frame_id
        && left.boundary_id == right.boundary_id
        && left.boundary_frame_id == right.boundary_frame_id
        && left.boundary_points == right.boundary_points
        && left.goal.pose == right.goal.pose
        && left.goal.header.frame_id == right.goal.header.frame_id
        && left.options == right.options
        && left.body_reference_frame_id == right.body_reference_frame_id
        && left.body_to_support_z_m == right.body_to_support_z_m
        && left.map_frame_generation == right.map_frame_generation
        && left.motion_guard_boot_nonce == right.motion_guard_boot_nonce
        && left.motion_stop_generation == right.motion_stop_generation
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

            {
                Err(PlannerGoalRejectReason::DuplicateLease)
            } else {
                Err(PlannerGoalRejectReason::ConflictingLease)
            }
        }
        std::cmp::Ordering::Greater => Ok(()),
    }
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
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

fn decode_configured_goal_envelope(
    payload: &[u8],
    body_reference_frame_id: &str,
    surface_support_z_m: Option<f64>,
) -> nuwax_codec::CodecResult<schema::GoalEnvelope> {
    let message = schema::decode_goal_envelope_body(payload)?;
    validate_goal_reference(&message, body_reference_frame_id, surface_support_z_m)?;
    Ok(message)
}

fn validate_goal_reference(
    message: &schema::GoalEnvelope,
    body_reference_frame_id: &str,
    surface_support_z_m: Option<f64>,
) -> nuwax_codec::CodecResult<()> {
    if message.body_reference_frame_id != body_reference_frame_id {
        return Err(nuwax_codec::CodecError::LcmDecode("navigation body reference differs across owners".into()));
    }
    if let (Some(declared), Some(expected)) = (message.body_to_support_z_m, surface_support_z_m) {
        if declared != expected {
            return Err(nuwax_codec::CodecError::LcmDecode("goal support conversion differs from execution owner".into()));
        }
    }
    Ok(())
}

pub(crate) fn capture_body_reference() -> Result<Arc<str>, String> {
    let body = configured_frame_alias("NUWAX_BASE_FRAME", "NUWAX_GEOMETRY_BASE_FRAME", "base_link")
        .map_err(|error| error.to_string())?;
    if !crate::map_pose::valid_body_reference(&body) {
        return Err("navigation body reference must be a valid frame ID".into());
    }
    Ok(Arc::from(body))
}

pub(crate) fn captured_body_reference(body: &Result<Arc<str>, String>) -> NavStackResult<Arc<str>> {
    body.as_ref().map(Arc::clone).map_err(|reason| NavStackError::InvalidConfig(reason.clone()))
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
    module.set_goal_options(nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..envelope.options})
        .map_err(|error|module_error("simple_planner.goal_options",error))?;
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
    owner_kind: i32,
    parent_request_id: String,
    goal_generation: i64,
    producer_runtime_generation: i64,
    path_revision: i64,
    planner_runtime_generation: i64,
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn far_join_key(state: &schema::NavigationState) -> Option<FarJoinKey> {
    if state.state != nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED
        || state.runtime_generation <= 0
        || state.path_revision <= 0
        || state.producer_runtime_generation <= 0
        || state.goal_generation <= 0
        || state.planner_runtime_generation != state.runtime_generation
    {
        return None;
    }
    Some(FarJoinKey {
        backend: state.robot_frame.clone(),
        runtime_generation: state.runtime_generation,
        session_id: state.session_id.clone(),
        owner_kind: state.owner_kind,
        parent_request_id: state.parent_request_id.clone(),
        goal_generation: state.goal_generation,
        producer_runtime_generation: state.producer_runtime_generation,
        path_revision: state.path_revision,
        planner_runtime_generation: state.planner_runtime_generation,
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn far_key_matches_goal(
    key: &FarJoinKey,
    goal: &schema::GoalEnvelope,
    accepted_generation: i64,
) -> bool {
    accepted_generation > 0
        && key.backend == "far-visibility-graph"
        && key.runtime_generation == accepted_generation
        && key.session_id == goal.session_id
        && key.owner_kind == goal.owner_kind
        && key.parent_request_id == goal.parent_request_id
        && key.goal_generation == goal.goal_generation
        && key.producer_runtime_generation == goal.producer_runtime_generation
        && key.planner_runtime_generation == accepted_generation
}

#[cfg(all(any(feature = "cmu-native", feature = "explore"), test))]
fn far_terminal_matches_goal(
    state: &schema::NavigationState,
    goal: &schema::GoalEnvelope,
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
fn native_path_respects_goal(path: &Path2D, goal: &schema::GoalEnvelope) -> bool {
    goal.committed_map_frame_id == "map"
        && goal.goal.header.frame_id == goal.committed_map_frame_id
        && path.frame_id.as_str() == goal.committed_map_frame_id
        && navigation_boundary_from_goal(goal)
            .ok()
            .is_some_and(|boundary| {
                path_respects_navigation_boundary(path, boundary.as_ref(), 0.025)
            })
}


#[cfg(any(feature = "cmu-native", feature = "explore"))]
fn admit_native_adapter_goal(
    latest_goal: &mut Option<schema::GoalEnvelope>,
    candidate: schema::GoalEnvelope,
) -> bool {
    let Some(current) = latest_goal.as_ref() else {
        *latest_goal = Some(candidate);
        return true;
    };
    let current_order = (current.producer_runtime_generation, current.goal_generation);
    let candidate_order = (
        candidate.producer_runtime_generation,
        candidate.goal_generation,
    );
    if candidate_order <= current_order {
        return false;
    }
    *latest_goal = Some(candidate);
    true
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(crate) fn native_planner_path_adapter_owner(
    node: &ModuleNode, body_reference_frame_id: Arc<str>) -> NavStackResult<Box<dyn InProcessNode>> {
    if node.inputs.len() < 2 || node.inputs.len() > 3 || node.outputs.len() != 1 {
        return Err(NavStackError::InvalidConfig(format!(
            "native planner adapter requires 2 or 3 inputs and one output, got {}/{}",
            node.inputs.len(),
            node.outputs.len()
        )));
    }
    let node = node.clone();
    Ok(boxed_owner(
        "native-planner-path-adapter",
        Box::new(move |shutdown, ready| {
            Box::pin(run_native_planner_path_adapter(node, body_reference_frame_id, shutdown, ready))
        }),
    ))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn run_native_planner_path_adapter(
    node: ModuleNode,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_native_planner_path_adapter_with_transport(node, body_reference_frame_id, transport, shutdown, ready).await
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
async fn run_native_planner_path_adapter_with_transport(
    node: ModuleNode,
    body_reference_frame_id: Arc<str>,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let mut goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.GoalEnvelope").await?;
    let mut events =
        typed_stream(&transport, &node.inputs[1], "nuwax_msgs.NavigationState").await?;
    let mut far_results = if node.inputs.len() == 3 {
        Some(typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationState").await?)
    } else {
        None
    };
    let mut latest_goal: Option<schema::GoalEnvelope> = None;
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
                let goal = match decode_configured_goal_envelope(&frame.payload, &body_reference_frame_id, None) {
                    Ok(goal) => goal,
                    Err(_) => continue,
                };
                if !admit_native_adapter_goal(&mut latest_goal, goal) {
                    continue;
                }
                #[cfg(feature = "mls")]
                {
                    path_revision = 0;
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
                    let state = match schema::decode_navigation_state_body(&frame.payload) {
                        Ok(state) => state,
                        Err(_) => continue,
                    };
                    let Some(goal) = latest_goal.as_ref() else { continue; };
                    let Some(key) = far_join_key(&state) else { continue; };
                    if !far_key_matches_goal(&key, goal, accepted_generation)
                        || state.planner_map_frame != goal.committed_map_frame_id
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
                        let state = match schema::decode_navigation_state_body(&frame.payload) {
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
                            || state.planner_map_frame != goal.committed_map_frame_id
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
pub(crate) fn simple_planner_owner(node: &ModuleNode, body_reference_frame_id: Arc<str>) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 4, &[2])?;
    let node = node.clone();
    Ok(boxed_owner(
        "simple-planner",
        Box::new(move |shutdown, ready| Box::pin(run_simple_planner(node, body_reference_frame_id, shutdown, ready))),
    ))
}

#[cfg(feature = "simple-2d")]
async fn run_simple_planner(
    node: ModuleNode,
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
        typed_stream(&transport, &node.inputs[2], "nuwax_msgs.GoalEnvelope").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let robot_geometry = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").ok()
        .filter(|value| !value.trim().is_empty())
        .map(|value| nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value))
        .transpose().map_err(NavStackError::InvalidConfig)?;
    let motion_limits=nuwax_nav_types_messages::NavigationCapabilities::from_env()
        .map_err(NavStackError::InvalidConfig)?.map(|profile| {
            let mut limits=profile.motion;
            let navigation=nuwax_nav_runtime::surface_follower::forward_navigation_kinematics(limits.kinematics());
            limits.model=navigation.model;
            limits.max_reverse_mps=navigation.max_reverse_mps;
            limits.max_lateral_mps=navigation.max_lateral_mps;
            limits
        });
    let mut module = SimplePlannerModule::with_config(nuwax_nav_simple_planner::SimplePlannerConfig {
        robot_geometry, motion_limits, ..nuwax_nav_simple_planner::SimplePlannerConfig::default()
    }).map_err(|error| module_error("simple_planner.robot_geometry", error))?;
    module.enable_background_planning();
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
                            module.on_occupancy_grid(raw_occupancy)
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
                    module.on_occupancy_grid(occupancy)
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
                let envelope = match decode_configured_goal_envelope(&frame.payload, &body_reference_frame_id, None) {
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
                if stop {
                    // stop=true 只清理当前执行投影；保留 newest_goal 以便真实 replacement 继续按顺序准入。
                    clear_simple_planner_active_projection(
                        &mut goal_buffer,
                        &mut active_goal,
                        &mut active_boundary,
                        &mut path_revision,
                        &mut published_path_is_boundary_safe,
                    );
                }
            }
            _ = ticker.tick(), if goal_buffer.has_applied_goal() => {
                let output = Module::tick(&mut module)
                    .map_err(|error| module_tick_error("simple_planner.tick", error))?;
                let mut publication = SimpleOutputPublication {
                    path_yaws: module.planned_yaws(),
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
    path_yaws: Option<&'a [f64]>,
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
                    path_yaws: publication.path_yaws,                    goal: publication.active_goal.ok_or_else(|| {
                        NavStackError::Internal(
                            "simple planner path has no active GoalEnvelope".into(),
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
                            "simple planner hold has no active GoalEnvelope".into(),
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
pub(crate) fn astar_planner_owner(node: &ModuleNode, body_reference_frame_id: Arc<str>) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 4, &[4])?;
    let node = node.clone();
    Ok(boxed_owner(
        "astar-planner",
        Box::new(move |shutdown, ready| Box::pin(run_astar_planner(node, body_reference_frame_id, shutdown, ready))),
    ))
}

#[cfg(feature = "replanning-astar")]
async fn run_astar_planner(
    node: ModuleNode,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_astar_planner_with_transport(node, body_reference_frame_id, transport, shutdown, ready).await
}

#[cfg(feature = "replanning-astar")]
async fn run_astar_planner_with_transport(
    node: ModuleNode,
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
    let mut goal = typed_stream(&transport, &node.inputs[2], "nuwax_msgs.GoalEnvelope").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let robot_geometry = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").ok()
        .filter(|value| !value.trim().is_empty())
        .map(|value| nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value))
        .transpose().map_err(NavStackError::InvalidConfig)?;
    let config = nuwax_nav_astar::GlobalPlannerConfig {
        robot_geometry, ..nuwax_nav_astar::GlobalPlannerConfig::default()
    };
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
                let envelope = match decode_configured_goal_envelope(&frame.payload, &body_reference_frame_id, None) {
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
                    path_yaws: None,                            goal: active_goal.as_ref().ok_or_else(|| {
                                NavStackError::Internal(
                                    "A* path has no active GoalEnvelope".into(),
                                )
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
    goal: &'a schema::GoalEnvelope,
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
    let header = schema_header(stamp, publication.goal.committed_map_frame_id.as_str())?;
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
    goal: &schema::GoalEnvelope,
    planner_runtime_generation: i64,
    path_revision: i64,
    source_timestamp_ns: i64,
    path: schema::Path,
) -> NavStackResult<schema::PathEnvelope> {
    let envelope = schema::PathEnvelope {
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: goal.session_id.clone(),


        goal_generation: goal.goal_generation,


        producer_runtime_generation: goal.producer_runtime_generation,


        planner_runtime_generation: planner_runtime_generation,
        path_revision: path_revision,

        path,
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
fn planner_test_goal(session_id: &str, goal_generation: i64) -> schema::GoalEnvelope {
    let mut goal = schema::GoalEnvelope {
            body_reference_frame_id: "base_link".into(),
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: session_id.into(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: "explore-parent".into(),
        goal_generation,
        producer: "wavefront".into(),
        producer_epoch: 13,
        producer_runtime_generation: 17,
        committed_map_session_id: "planner-test-map".into(),
        committed_map_session_generation: 3,
        committed_map_revision: 5,
        committed_map_boot_nonce: [0x31; 16],
        committed_map_commit_sequence: 1,
        committed_map_source_timestamp_ns: 1,
        committed_map_frame_id: "map".into(),
        source_timestamp_ns: goal_generation * 1_000_000_000,
        boundary_id: "explore-parent:boundary".into(),
        boundary_frame_id: "map".into(),
        boundary_points: vec![
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [5.0, 5.0, 0.0],
            [0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0],
        ],

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

    schema::validate_goal_envelope(&goal).expect("seal planner test goal");
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
            schema::validate_goal_envelope(&goal).expect("seal producer fixture goal");
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
    fn published_path_preserves_goal_heading_and_uses_segment_headings() {
        let mut goal = planner_test_goal("heading", 1);
        goal.goal.pose.position[2] = 3.0;
        let half_yaw = std::f64::consts::FRAC_PI_4;
        goal.goal.pose.orientation_xyzw = [0.0, 0.0, half_yaw.sin(), half_yaw.cos()];
        schema::validate_goal_envelope(&goal).unwrap();
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
        let mut buffer = PlannerGoalBuffer::default();
        assert!(matches!(
            buffer.observe_goal(goal.clone()),
            PlannerGoalIngress::Buffered
        ));
        assert_eq!(buffer.pending_goal.as_ref(), Some(&goal));

        let ready = buffer
            .observe_costmap()
            .expect("first valid costmap must release the buffered simple goal");
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
    fn simple_stop_clears_late_path_gate_and_accepts_real_replacement() {
        let mut buffer = PlannerGoalBuffer::default();
        assert!(buffer.observe_costmap().is_none());
        let goal_a = planner_test_goal("simple-stop-a", 1);
        let ready_a = match buffer.observe_goal(goal_a.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("goal A must be ready: {outcome:?}"),
        };
        assert!(buffer.mark_applied(&ready_a));

        let mut active_goal = Some(ready_a);
        let mut active_boundary =
            Some(NavigationBoundary::new(&goal_a.boundary_points).expect("goal A boundary"));
        let mut path_revision = 3_i64;
        let mut published_path_is_boundary_safe = true;
        assert!(active_goal.is_some());
        assert!(active_boundary.is_some());
        assert_eq!(path_revision, 3);
        assert!(published_path_is_boundary_safe);

        clear_simple_planner_active_projection(
            &mut buffer,
            &mut active_goal,
            &mut active_boundary,
            &mut path_revision,
            &mut published_path_is_boundary_safe,
        );

        let mut module = SimplePlannerModule::new();
        Module::start(&mut module).expect("start simple planner");
        let mut robot_pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
        robot_pose.stamp = Timestamp::from_unix(30, 0);
        Module::on_odometry(&mut module, robot_pose).expect("simple planner pose");
        Module::on_stop_movement(&mut module, true).expect("first stop");
        assert!(matches!(
            Module::tick(&mut module).expect("first post-stop tick"),
            ModuleOutput::SafetyHold {
                reason: nuwax_nav_simple_planner::module::HoldReason::StopRequested,
                ..
            }
        ));
        assert!(
            !buffer.has_applied_goal(),
            "the first StopRequested hold must not be wrapped as a late PathEnvelope"
        );
        Module::on_stop_movement(&mut module, true).expect("repeated stop");
        assert!(matches!(
            Module::tick(&mut module).expect("repeated post-stop tick"),
            ModuleOutput::SafetyHold {
                reason: nuwax_nav_simple_planner::module::HoldReason::StopRequested,
                ..
            }
        ));

        assert!(
            !buffer.has_applied_goal(),
            "stop must gate every later tick"
        );
        assert!(active_goal.is_none());
        assert!(active_boundary.is_none());
        assert_eq!(path_revision, 0);
        assert!(!published_path_is_boundary_safe);
        assert_eq!(
            buffer
                .newest_goal
                .as_ref()
                .map(|goal| goal.session_id.as_str()),
            Some("simple-stop-a"),
            "stop retains the ordering watermark"
        );

        let goal_b = planner_test_goal("simple-stop-b", 2);
        let ready_b = match buffer.observe_goal(goal_b.clone()) {
            PlannerGoalIngress::Ready(goal) => *goal,
            outcome => panic!("real replacement must be accepted: {outcome:?}"),
        };
        assert!(buffer.mark_applied(&ready_b));
        assert!(buffer.has_applied_goal());
        assert_eq!(
            buffer
                .newest_goal
                .as_ref()
                .map(|goal| goal.session_id.as_str()),
            Some("simple-stop-b")
        );
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
        planner
            .on_occupancy_grid(
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
        planner
            .on_occupancy_grid(occupancy)
            .expect("simple planner accepts valid grid");
    }
}

#[cfg(all(test, feature = "guard-runtime"))]
mod guarded_tests {
    fn planar_test_costmap(
        observation: &nuwax_nav_runtime::CostmapObservation,
    ) -> &nuwax_nav_astar::navigation_map::CostMap2D {
        match &observation.costmap {
            nuwax_nav_runtime::ExecutionMap::Planar(map) => map,
            _ => panic!("expected planar map"),
        }
    }

    use super::*;
    use futures::channel::mpsc::{unbounded, UnboundedSender};
    use nuwax_nav_runtime::{NavMotionPort, NavSchedulerHandle, RuntimeError, SchedulerPhase, TerminalOutcome};
    use tokio::task::JoinHandle;

    #[cfg(feature = "surface-recovery-qualification")]
    mod ordinary_ack_cancel_tests {
        use super::*;
        include!("guarded_ordinary_ack_cancel_tests.rs");
    }

    #[cfg(feature = "surface-recovery-qualification")]
    mod ingress_ack_tests {
        use super::*;
        include!("guarded_ingress_ack_tests.rs");
    }

    #[cfg(feature = "surface-recovery-qualification")]
    pub(super) async fn hold_test_publication_ack(
        status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry,
        planner_observation: Option<bool>,
    ) {
        ingress_ack_tests::hold_publication_ack(status, telemetry, planner_observation).await;
    }

    #[cfg(feature = "surface-recovery-qualification")]
    pub(super) async fn hold_test_publication_before_delegate(
        status: &nuwax_nav_runtime::SchedulerStatus,
        telemetry: &GuardedNavTelemetry,
        planner_observation: Option<bool>,
        frames: &[RawLcmFrame],
    ) {
        ingress_ack_tests::hold_publication_before_delegate(
            status, telemetry, planner_observation, frames,
        ).await;
    }

    #[cfg(feature = "surface-recovery-qualification")]
    pub(super) fn record_ack_return_cancel(phase: &'static str, cancel: &schema::CancelNavigation) {
        ingress_ack_tests::record_cancel(phase, cancel);
    }

    #[test]
    fn arbiter_rotates_all_simultaneously_ready_sources() {
        let mut arbiter = GuardedNavArbiter::default();
        let selected = (0..GuardedNavSource::ORDER.len())
            .map(|_| arbiter.choose_ready([true; 6]).expect("ready source"))
            .collect::<Vec<_>>();

        assert_eq!(selected, GuardedNavSource::ORDER);
        assert_eq!(arbiter.cursor, GuardedNavSource::Pose);
    }

    #[test]
    fn arbiter_bounds_pose_and_costmap_under_continuous_status() {
        let mut arbiter = GuardedNavArbiter::default();
        let ready = [true, true, true, false, false, false];
        let selected = (0..9)
            .map(|_| arbiter.choose_ready(ready).expect("live planner input"))
            .collect::<Vec<_>>();

        for source in [
            GuardedNavSource::Pose,
            GuardedNavSource::Costmap,
            GuardedNavSource::Status,
        ] {
            let positions = selected
                .iter()
                .enumerate()
                .filter_map(|(index, selected)| (*selected == source).then_some(index))
                .collect::<Vec<_>>();
            assert_eq!(positions.len(), 3);
            assert!(positions.windows(2).all(|pair| pair[1] - pair[0] <= 3));
        }
    }

    #[tokio::test]
    async fn arbiter_keeps_status_twist_and_zero_live_under_continuous_inputs() {
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
        harness.publication_probe.lock().unwrap().ledgers=Some((Arc::clone(&feeder.pose_ledger),Arc::clone(&feeder.costmap_ledger)));

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
        let twist_ready_dispatch = feeder.dequeued_by_source();
        let (applied,receipt) = tokio::time::timeout(Duration::from_secs(1), async {
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
                    return (twist,frame.payload.to_vec());
                }
            }
        })
        .await
        .expect("bounded status Twist publication");
        feeder.assert_ready_evidence("Twist");
        feeder.assert_status_dispatch_bound(twist_ready_dispatch,harness.publication_dispatches(&receipt).await);
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
        let terminal_ready_dispatch = feeder.dequeued_by_source();

        let (zero,receipt) = tokio::time::timeout(Duration::from_secs(1), async {
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
                    return (twist,frame.payload.to_vec());
                }
            }
        })
        .await
        .expect("bounded status ZERO publication");
        feeder.assert_ready_evidence("ZERO");
        feeder.assert_status_dispatch_bound(terminal_ready_dispatch,harness.publication_dispatches(&receipt).await);
        assert_eq!(zero.linear, [0.0; 3]);
        assert_eq!(zero.angular, [0.0; 3]);

        let (reached,receipt) = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .reached
                    .next()
                    .await
                    .expect("reached stream open")
                    .expect("reached frame");
                let reached = schema::decode_bool_body(&frame.payload).expect("Succeeded bool");
                if reached.data {
                    return (reached,frame.payload.to_vec());
                }
            }
        })
        .await
        .expect("bounded Succeeded bool publication");
        feeder.assert_ready_evidence("Succeeded Bool");
        feeder.assert_status_dispatch_bound(terminal_ready_dispatch,harness.publication_dispatches(&receipt).await);
        assert!(reached.data);

        let (terminal,receipt) = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness
                    .states
                    .next()
                    .await
                    .expect("state stream open")
                    .expect("state frame");
                let state = schema::decode_navigation_state_body(&frame.payload)
                    .expect("real status NavigationState");
                if state.session_id == goal.session_id
                    && !state.zero_ack_id.is_empty()
                    && state.zero_applied_monotonic_ns > 0
                {
                    return (state,frame.payload.to_vec());
                }
            }
        })
        .await
        .expect("bounded terminal NavigationState publication");
        feeder.assert_ready_evidence("terminal NavigationState");
        feeder.assert_status_dispatch_bound(terminal_ready_dispatch,harness.publication_dispatches(&receipt).await);
        assert_eq!(
            harness.scheduler.status().last_terminal,
            Some(TerminalOutcome::Succeeded)
        );
        assert!(!terminal.zero_ack_id.is_empty());
        assert!(
            !harness.loop_task.is_finished(),
            "single owner loop remains live"
        );
        feeder.stop().await.expect("contention feeder cleanup");
        harness.shutdown().await;
    }

    #[test]
    fn arbiter_bounds_every_ready_source() {
        for target in GuardedNavSource::ORDER {
            let mut arbiter = GuardedNavArbiter {
                cursor: target.successor(),
            };
            let mut ready = [true; 6];
            let mut preceding_dispatches = 0;
            loop {
                let selected = arbiter.choose_ready(ready).expect("ready source");
                if selected == target {
                    break;
                }
                ready[selected.index()] = false;
                preceding_dispatches += 1;
            }
            assert!(
                preceding_dispatches <= 5,
                "{target:?} waited for {preceding_dispatches} handlers"
            );
        }
    }

    #[tokio::test]
    async fn arbiter_allows_one_wake_race_then_restores_rotation() {
        let (_goals_tx, mut goals) = guarded_test_stream();
        let (_poses_tx, mut poses) = guarded_test_stream();
        let (paths_tx, mut paths) = guarded_test_stream();
        let (_costmaps_tx, mut costmaps) = guarded_test_stream();
        let (_cancels_tx, mut cancels) = guarded_test_stream();
        let (_status_tx, mut status) =
            tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
        let waiting = wait_guarded_nav_event(
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        );
        tokio::pin!(waiting);
        assert!(
            tokio::time::timeout(Duration::from_millis(10), waiting.as_mut())
                .await
                .is_err(),
            "all-pending sources must await a wake"
        );

        send_guarded_test_body(&paths_tx, "nuwax_msgs.PathEnvelope", vec![0]);
        let wake = tokio::time::timeout(Duration::from_secs(1), waiting.as_mut())
            .await
            .expect("path wake");
        assert_eq!(wake.source(), GuardedNavSource::Path);

        let mut arbiter = GuardedNavArbiter::default();
        arbiter.record_dispatch(wake.source());
        assert_eq!(
            arbiter.choose_ready([true; 6]),
            Some(GuardedNavSource::Cancel),
            "the wake winner must move the next scan to its successor"
        );
        assert_eq!(
            arbiter.choose_ready([true; 6]),
            Some(GuardedNavSource::Pose)
        );
    }

    #[test]
    fn arbiter_shutdown_preempts_ready_work() {
        let (shutdown_tx, mut shutdown_rx) = oneshot::channel();
        shutdown_tx.send(()).expect("shutdown receiver");
        let arbiter = GuardedNavArbiter::default();

        assert!(guarded_nav_shutdown_requested(&mut shutdown_rx));
        assert_eq!(
            arbiter.cursor,
            GuardedNavSource::Pose,
            "ready inputs must not advance the cursor after shutdown"
        );

        let (closed_tx, mut closed_rx) = oneshot::channel::<()>();
        drop(closed_tx);
        assert!(guarded_nav_shutdown_requested(&mut closed_rx));
    }

    #[tokio::test]
    async fn arbiter_propagates_required_stream_close() {
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
                    send_guarded_test_body(&goals_tx, "nuwax_msgs.GoalEnvelope", payload.clone())
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
            let arbiter = GuardedNavArbiter { cursor: source };
            let event = scan_guarded_nav_event(
                &arbiter,
                &mut goals,
                &mut poses,
                &mut paths,
                &mut costmaps,
                &mut cancels,
                &mut status,
            )
            .expect("ready source mapping");
            assert_eq!(event.source(), source);
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
            let event = scan_guarded_nav_event(
                &GuardedNavArbiter { cursor: source },
                &mut goals,
                &mut poses,
                &mut paths,
                &mut costmaps,
                &mut cancels,
                &mut status,
            )
            .expect("closed stream is Ready");
            assert_eq!(event.source(), source);
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
    async fn arbiter_propagates_status_close() {
        let (_goals_tx, mut goals) = guarded_test_stream();
        let (_poses_tx, mut poses) = guarded_test_stream();
        let (_paths_tx, mut paths) = guarded_test_stream();
        let (_costmaps_tx, mut costmaps) = guarded_test_stream();
        let (_cancels_tx, mut cancels) = guarded_test_stream();
        let (status_tx, mut status) =
            tokio::sync::watch::channel(nuwax_nav_runtime::SchedulerStatus::default());
        drop(status_tx);
        let event = scan_guarded_nav_event(
            &GuardedNavArbiter {
                cursor: GuardedNavSource::Status,
            },
            &mut goals,
            &mut poses,
            &mut paths,
            &mut costmaps,
            &mut cancels,
            &mut status,
        )
        .expect("closed watch is Ready");
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
    fn arbiter_preserves_reject_and_fail_closed_paths() {
        assert!(matches!(
            runtime_input_error::<()>(
                "guarded arbitration",
                RuntimeError::StaleInput("old pose".into()),
            )
            .expect("stale external input remains rejectable"),
            OwnerInput::Rejected(_)
        ));
        assert!(runtime_input_error::<()>(
            "guarded arbitration",
            RuntimeError::InvalidFence("missing zero proof".into()),
        )
        .is_err());
        assert!(matches!(
            decode_guarded_pose_input(&[0xff]).expect("bad pose remains external input"),
            OwnerInput::Rejected(_)
        ));
        assert!(matches!(
            decode_guarded_costmap_input(&[0xff]).expect("bad costmap remains external input"),
            OwnerInput::Rejected(_)
        ));
    }

    #[test]
    fn canonical_base_frame_value_precedes_legacy_alias() {
        assert_eq!(
            resolve_frame_alias(Some(" body "), Some("base_footprint"), "base_link"),
            Some("body".to_string())
        );
        assert_eq!(
            resolve_frame_alias(Some("  "), Some(" base_footprint "), "base_link"),
            Some("base_footprint".to_string())
        );
        assert_eq!(
            resolve_frame_alias(None, None, " base_link "),
            Some("base_link".to_string())
        );
    }

    fn active_goal(session_id: &str) -> schema::GoalEnvelope {
        schema::GoalEnvelope {
            body_reference_frame_id: "base_link".into(),
            session_id: session_id.into(),
            ..schema::GoalEnvelope::default()
        }
    }

    fn sealed_goal(session_id: &str) -> schema::GoalEnvelope {
        let mut goal = schema::GoalEnvelope {
            body_reference_frame_id: "base_link".into(),
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: session_id.into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "public-api-request-1".into(),
            goal_generation: 3,
            producer: "public-navigation".into(),
            producer_epoch: 7,
            producer_runtime_generation: 7,
            committed_map_session_id: "guarded-test-map".into(),
            committed_map_session_generation: 2,
            committed_map_revision: 4,
            committed_map_boot_nonce: [0x42; 16],
            committed_map_commit_sequence: 1,
            committed_map_source_timestamp_ns: 1,
            committed_map_frame_id: "map".into(),
            source_timestamp_ns: 1_000_000_000,

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
        schema::validate_goal_envelope(&goal).expect("seal goal");
        goal
    }

    fn sealed_path(goal: &schema::GoalEnvelope) -> schema::PathEnvelope {
        let mut path = schema::PathEnvelope {
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: goal.session_id.clone(),


            goal_generation: goal.goal_generation,


            producer_runtime_generation: goal.producer_runtime_generation,


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
        schema::validate_goal_envelope(&goal).expect("seal rollover goal");
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
    pub(super) struct RecordingMotion(pub(super) Arc<Mutex<Vec<String>>>);

    struct FirstAcquireMotion {
        inner: RecordingMotion,
        reject_first: bool,
    }

    #[async_trait::async_trait]
    impl nuwax_nav_runtime::NavMotionPort for FirstAcquireMotion {
        async fn acquire(&mut self, operation_id: &str, admission: &nuwax_nav_runtime::NavMotionAdmission)
            -> Result<nuwax_nav_runtime::LeaseProof, nuwax_nav_runtime::MotionPortError>
        {
            if self.reject_first {
                self.reject_first = false;
                self.inner.0.lock().unwrap().push(format!("rejected_acquire:{operation_id}"));
                return Err(nuwax_nav_runtime::MotionPortError::definite_no_lease("STALE_STOP_GENERATION"));
            }
            self.inner.acquire(operation_id, admission).await
        }

        async fn renew(&mut self, operation_id: &str) -> Result<(), nuwax_nav_runtime::MotionPortError> {
            self.inner.renew(operation_id).await
        }

        async fn submit_twist(&mut self, operation_id: &str, twist: Twist2)
            -> Result<nuwax_nav_runtime::AppliedProof, nuwax_nav_runtime::MotionPortError>
        {
            self.inner.submit_twist(operation_id, twist).await
        }

        async fn revoke_and_zero(&mut self, operation_id: &str)
            -> Result<nuwax_nav_runtime::FenceProof, nuwax_nav_runtime::MotionPortError>
        {
            self.inner.revoke_and_zero(operation_id).await
        }
    }

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

    struct CommandingPlanner {
        succeeds: Arc<std::sync::atomic::AtomicBool>,
        twist: Arc<Mutex<Twist2>>,
    }

    impl nuwax_nav_runtime::NavPlanner for CommandingPlanner {
        fn replace_path(&mut self, _path: Path2D) {}

        fn update_odometry(&mut self, _odometry: PoseStamped) {}

        fn update_costmap(&mut self, _costmap: nuwax_nav_astar::navigation_map::CostMap2D) {}

        fn step(&mut self, _robot_width_m: f64) -> nuwax_nav_runtime::PlannerStep {
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

        fn dequeued_by_source(&self) -> (usize, usize) {
            (
                self.pose_ledger
                    .dequeued
                    .load(std::sync::atomic::Ordering::SeqCst),
                self.costmap_ledger
                    .dequeued
                    .load(std::sync::atomic::Ordering::SeqCst),
            )
        }

        fn assert_status_dispatch_bound(&self, before: (usize, usize),after:(usize,usize)) {
            // 状态就绪后最多分派一次位姿批次和一次地图；合并读取不等于多次分派。
            assert!(
                after.0.saturating_sub(before.0) <= MAX_REPLACEABLE_POSE_BATCH,
                "状态发送前消费超过一个位姿批次"
            );
            assert!(
                after.1.saturating_sub(before.1) <= 1,
                "状态发送前分派超过一次地图输入"
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

    #[derive(Default)]
    struct PublicationProbe {
        #[cfg(feature = "surface-recovery-qualification")]
        ack_return_probe: Option<ingress_ack_tests::AckReturnProbe>,
        ledgers:Option<(Arc<GuardedDequeueLedger>,Arc<GuardedDequeueLedger>)>,
        entries:std::collections::BTreeMap<Vec<u8>,(usize,usize)>,
    }
    tokio::task_local! {static PUBLICATION_PROBE:Arc<Mutex<PublicationProbe>>;}
    pub(super) fn record_test_publication(receipts:&[Vec<u8>]) {
        let _=PUBLICATION_PROBE.try_with(|probe| {
            let mut probe=probe.lock().unwrap();
            if let Some((pose,map))=&probe.ledgers {
                let counts=(pose.dequeued.load(std::sync::atomic::Ordering::SeqCst),map.dequeued.load(std::sync::atomic::Ordering::SeqCst));
                for payload in receipts {probe.entries.entry(payload.clone()).or_insert(counts);}
            }
        });
    }

    struct GuardedLoopHarness {
        goals: GuardedTestSender,
        poses: GuardedTestSender,
        paths: GuardedTestSender,
        costmaps: GuardedTestSender,
        cancels: GuardedTestSender,
        commands: LcmFrameStream,
        publication_probe:Arc<Mutex<PublicationProbe>>,
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
        async fn publication_dispatches(&self,payload:&[u8])->(usize,usize) {
            // 网络接收者可能先于发送者的完成回调被唤醒，等待同一载荷回执落地。
            tokio::time::timeout(Duration::from_secs(1),async {
                loop {
                    if let Some(counts)=self.publication_probe.lock().unwrap().entries.get(payload).copied(){return counts;}
                    tokio::task::yield_now().await;
                }
            }).await.expect("真实发送回执未落地")
        }
        async fn spawn() -> Self {Self::spawn_with_planar_mode(false).await}

        async fn spawn_with_planar_mode(planar_mode:bool) -> Self {
            Self::spawn_with_reference(planar_mode, "base_link", None).await
        }

        async fn spawn_with_reference(planar_mode:bool, body_reference: &'static str, support: Option<f64>) -> Self {
            Self::spawn_with_first_acquire(planar_mode, body_reference, support, false).await
        }

        async fn spawn_with_first_acquire(planar_mode:bool, body_reference: &'static str, support: Option<f64>, reject_first: bool) -> Self {
            static NEXT_LOOP_ID: std::sync::atomic::AtomicU64 =
                std::sync::atomic::AtomicU64::new(0);
            let loop_id = NEXT_LOOP_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            // One real liblcm context is shared by this in-process fixture.
            // Functional owner/codec checks do not require a host multicast route.
            let transport = LcmTransport::new_real("memq://")
                .expect("guarded loop test LCM memq");
            let motion_calls = Arc::new(Mutex::new(Vec::<String>::new()));
            let planner_succeeds = Arc::new(std::sync::atomic::AtomicBool::new(false));
            let planner_twist = Arc::new(Mutex::new(Twist2 {
                linear: Point2::new(0.2, 0.0),
                angular_z: 0.1,
            }));
            let scheduler = nuwax_nav_runtime::NavScheduler::new(
                FirstAcquireMotion { inner: RecordingMotion(Arc::clone(&motion_calls)), reject_first },
                CommandingPlanner {
                    succeeds: Arc::clone(&planner_succeeds),
                    twist: Arc::clone(&planner_twist),
                },
                nuwax_nav_runtime::SchedulerConfig {
                    tick_interval: Duration::from_millis(5),
                    renew_interval: Duration::from_millis(100),
                    path_wait_timeout: Duration::from_secs(5),
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
            let commands = typed_stream(&transport, &node.outputs[0], "geometry_msgs.Twist")
                .await
                .expect("guarded command test stream");
            let states = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.NavigationState")
                .await
                .expect("guarded state test stream");
            let reached = typed_stream(&transport, &node.outputs[2], "std_msgs.Bool")
                .await
                .expect("guarded reached test stream");
            let loop_transport = transport.clone();
            let publication_probe=Arc::new(Mutex::new(PublicationProbe::default()));
            let loop_task = tokio::spawn(PUBLICATION_PROBE.scope(Arc::clone(&publication_probe),async move {
                run_guarded_nav_loop(
                    &node,
                    &loop_transport,
                    &loop_scheduler,
                    shutdown_rx,
                    goal_stream,
                    pose_stream,
                    path_stream,
                    costmap_stream,
                    cancel_stream,
                    status,
                    None,
                    planar_mode,
                    body_reference,
                    support,
                    None,
                )
                .await
            }));

            Self {
                goals,
                poses,
                paths,
                costmaps,
                cancels,
                commands,
                publication_probe,
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

        async fn publish_stopped_goal_pose(&self, goal: &schema::GoalEnvelope) {
            let mut status = self.scheduler.subscribe_status();
            let source = tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    let snapshot = status.borrow().clone();
                    assert_ne!(snapshot.last_terminal, Some(TerminalOutcome::Succeeded),
                        "停车后观察尚未送达，不能提前成功");
                    if snapshot.phase == SchedulerPhase::Fencing && snapshot.message.starts_with("已确认停稳") {
                        assert!(snapshot.last_zero_ack_id.is_none());
                        break snapshot.pose_source_timestamp_ns + 1;
                    }
                    status.changed().await.unwrap();
                }
            }).await.expect("真实停车证明后的复核阶段");
            // 夹具继续通过同一位姿数据面发送停车后的目标位置，不直接改终态。
            let mut pose = goal.goal.clone();
            pose.header.stamp_sec = (source / 1_000_000_000) as i32;
            pose.header.stamp_nsec = (source % 1_000_000_000) as i32;
            send_guarded_test_body(&self.poses, "geometry_msgs.PoseStamped",
                schema::encode_pose_stamped_body(&pose).unwrap());
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

        fn send_cancel(&self, goal: &schema::GoalEnvelope, cancel_generation: i64) {
            send_guarded_test_body(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                Self::cancel_body(goal, cancel_generation),
            );
        }

        async fn send_cancel_and_wait_dequeued(
            &self,
            goal: &schema::GoalEnvelope,
            cancel_generation: i64,
        ) {
            let dequeued = send_guarded_test_body_observed(
                &self.cancels,
                "nuwax_msgs.CancelNavigation",
                Self::cancel_body(goal, cancel_generation),
            );
            tokio::time::timeout(Duration::from_secs(1), dequeued)
                .await
                .expect("guarded cancellation was not dequeued")
                .expect("guarded cancellation dequeue receipt was dropped");
            assert!(!self.loop_task.is_finished(), "guarded loop exited early");
        }

        fn cancel_body(goal: &schema::GoalEnvelope, cancel_generation: i64) -> Vec<u8> {
            schema::encode_cancel_navigation_body(&schema::CancelNavigation {
                session_id: goal.session_id.clone(),
                reason: 1,
                cancel_generation,
                owner_kind: goal.owner_kind,
                parent_request_id: goal.parent_request_id.clone(),
                goal_generation: goal.goal_generation,
                producer_runtime_generation: goal.producer_runtime_generation,
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

        async fn wait_for_correlated_zero(
            &mut self,
            goal: &schema::GoalEnvelope,
            cancel_generation: i64,
        ) -> schema::NavigationState {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    let frame = self
                        .states
                        .next()
                        .await
                        .expect("guarded state stream open")
                        .expect("guarded state transport frame");
                    let state = schema::decode_navigation_state_body(&frame.payload)
                        .expect("guarded NavigationState body");
                    if state.session_id == goal.session_id
                        && state.owner_kind == goal.owner_kind
                        && state.parent_request_id == goal.parent_request_id
                        && state.goal_generation == goal.goal_generation
                        && state.cancel_generation == cancel_generation
                        && !state.zero_ack_id.is_empty()
                        && state.zero_applied_monotonic_ns > 0
                    {
                        return state;
                    }
                }
            })
            .await
            .expect("correlated ZEROED NavigationState")
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
                #[cfg(feature = "surface-recovery-qualification")]
                ingress_ack_tests::record_observed_dequeue(&frame);
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
            map_frame_generation: goal.map_frame_generation,
            runtime_generation:goal.producer_runtime_generation,session_id:goal.committed_map_session_id.clone(),
            session_generation:goal.committed_map_session_generation,map_revision:goal.committed_map_revision,
            map_boot_nonce:goal.committed_map_boot_nonce,map_commit_sequence:goal.committed_map_commit_sequence,
            map_source_timestamp_ns:goal.committed_map_source_timestamp_ns,observation_revision:1,source_sequence:0,
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
        harness.wait_for_zero().await;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        send_guarded_test_body(&harness.costmaps,planar_navigation::TYPE_NAME,body);
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn rejected_lease_publishes_candidate_failure_and_owner_accepts_next_goal() {
        let mut harness = GuardedLoopHarness::spawn_with_first_acquire(false, "base_link", None, true).await;
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        let rejected = sealed_goal("rejected-lease");
        harness.send_goal_and_wait_dequeued(&rejected).await;
        let failure = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_state_body(&frame.payload).unwrap();
                if state.session_id == rejected.session_id && state.state == 4 { break state; }
            }
        }).await.expect("必须实际发布被拒绝候选目标的失败");
        assert_eq!(failure.message, "LEASE_NOT_GRANTED: STALE_STOP_GENERATION");
        assert_eq!(failure.owner_kind, rejected.owner_kind);
        assert_eq!(failure.parent_request_id, rejected.parent_request_id);
        assert_eq!(failure.goal_generation, rejected.goal_generation);
        assert_eq!(failure.producer_runtime_generation, rejected.producer_runtime_generation);
        assert_eq!(failure.path_revision, 0);
        assert_eq!(failure.planner_runtime_generation, 0);
        assert!(failure.zero_ack_id.is_empty());
        assert_eq!(failure.zero_applied_monotonic_ns, 0);
        assert_ne!(failure.sink_ready, 0);
        harness.send_pose(2);
        harness.wait_for_pose(2).await;
        harness.send_costmap(12);
        harness.wait_for_costmap(12).await;
        harness.assert_no_session().await;
        assert_eq!(harness.motion_call_count("rejected_acquire:"), 1);
        assert_eq!(harness.motion_call_count("acquire:"), 0, "失败目标不能被新输入自动复活");
        assert_eq!(harness.motion_call_count("zero:"), 0, "未取得租约不能伪造归零");
        let mut next = sealed_goal("lease-recovered");
        next.goal_generation = rejected.goal_generation + 1;
        next.motion_stop_generation = rejected.motion_stop_generation + 1;
        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session(&next.session_id).await;
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        harness.send_cancel(&next, 1);
        harness.wait_for_correlated_zero(&next, 1).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn configuration_rejected_goal_stays_cancellable_and_cannot_reactivate() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let mut goal = sealed_goal("configuration-rejected");
        goal.body_reference_frame_id = "other/body".into();
        schema::validate_goal_envelope(&goal).unwrap();
        harness.send_goal_and_wait_dequeued(&goal).await;
        harness.assert_no_session().await;
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.assert_no_session().await;
        assert_eq!(harness.motion_call_count("acquire:"), 0);
        harness.send_cancel(&goal, 1);
        harness.wait_for_correlated_zero(&goal, 1).await;
        assert_eq!(harness.motion_call_count("zero:"), 1);
        let mut next = sealed_goal("configuration-recovered");
        next.goal_generation = goal.goal_generation + 1;
        schema::validate_goal_envelope(&next).unwrap();
        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session(&next.session_id).await;
        harness.send_cancel(&next, 1);
        harness.wait_for_correlated_zero(&next, 1).await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn support_rejected_goal_stays_cancellable_and_cannot_reactivate() {
        let mut harness = GuardedLoopHarness::spawn_with_reference(false, "base_link", Some(0.4)).await;
        let mut goal = sealed_goal("support-rejected");
        goal.body_to_support_z_m = Some(0.6);
        harness.send_goal_and_wait_dequeued(&goal).await;
        harness.assert_no_session().await;
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.assert_no_session().await;
        assert_eq!(harness.motion_call_count("acquire:"), 0);
        harness.send_cancel(&goal, 1);
        let terminal = harness.wait_for_correlated_zero(&goal, 1).await;
        assert_eq!(terminal.producer_runtime_generation, goal.producer_runtime_generation);
        assert_eq!(terminal.planner_runtime_generation, 0);
        assert_eq!(harness.motion_call_count("zero:"), 1);
        let mut next = sealed_goal("support-recovered");
        next.goal_generation = goal.goal_generation + 1;
        next.body_to_support_z_m = Some(0.4);
        harness.send_goal_and_wait_dequeued(&next).await;
        harness.wait_for_session(&next.session_id).await;
        harness.send_cancel(&next, 1);
        harness.wait_for_correlated_zero(&next, 1).await;
        harness.shutdown().await;
    }

    #[test]
    fn goal_reference_contract_uses_only_consumed_body_and_support_facts() {
        let mut goal = sealed_goal("reference-contract");
        let captured_body = Arc::<str>::from("base_link");
        assert!(validate_goal_reference(&goal, &captured_body, None).is_ok());
        // Explicit public body poses do not consume Geometry's support conversion.
        assert!(validate_goal_reference(&goal, &captured_body, Some(0.2)).is_ok());
        assert!(validate_goal_reference(&goal, &captured_body, Some(0.9)).is_ok());
        goal.body_reference_frame_id = "body".into();
        assert!(validate_goal_reference(&goal, &captured_body, None).is_err());
        goal.body_reference_frame_id = captured_body.to_string();
        goal.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        goal.producer = "wavefront".into();
        // Mission-return home is an original body pose, not a support-derived endpoint.
        // None is also the valid planar exploration contract.
        assert!(validate_goal_reference(&goal, &captured_body, Some(0.4)).is_ok());
        assert!(validate_goal_reference(&goal, &captured_body, None).is_ok());
        goal.body_to_support_z_m = Some(0.4);
        assert!(validate_goal_reference(&goal, &captured_body, Some(0.4)).is_ok());
        assert!(validate_goal_reference(&goal, &captured_body, Some(0.6)).is_err());
        // Planar owners do not consume support offsets; their existing coordinate/profile rules still apply.
        assert!(validate_goal_reference(&goal, &captured_body, None).is_ok());
        // The assigned contract is preserved on sloped body poses, where offset affects XY too.
        goal.goal.pose.orientation_xyzw = [0.0, 0.3826834323650898, 0.0, 0.9238795325112867];
        let body = schema::encode_goal_envelope_body(&goal).unwrap();
        assert!(decode_configured_goal_envelope(&body, &captured_body, Some(0.4)).is_ok());
        assert!(decode_configured_goal_envelope(&body, &captured_body, Some(0.6)).is_err());
        assert!(decode_configured_goal_envelope(&body, &captured_body, None).is_ok());
    }

    #[test]
    fn all_goal_bound_profiles_accept_their_canonical_body_tf_and_goal_reference() {
        use crate::{NavFeatureFlags, NavStackProfile, NavTopicConfig};
        for profile in [NavStackProfile::Simple2D, NavStackProfile::ReplanningAStar, NavStackProfile::Mls3D] {
            let graph = crate::profiles::build_graph(profile, &NavTopicConfig::default(), NavFeatureFlags::default());
            assert!(graph.find("map_pose_adapter").is_some());
            for body in ["base_link", "body", "robot/body_a"] {
                let mut adapter = bound_map_pose_adapter("map".into(), "odom".into(), body).unwrap();
                let tf = schema::TfMessage { transforms: vec![
                    schema::TransformStamped {
                        header: schema::Header { stamp_sec: 7, frame_id: "map".into(), ..Default::default() },
                        child_frame_id: "odom".into(),
                        transform: schema::Transform { translation: [1.0, 2.0, 0.0], rotation_xyzw: [0.0, 0.0, 0.0, 1.0] },
                    },
                    schema::TransformStamped {
                        header: schema::Header { stamp_sec: 7, frame_id: "odom".into(), ..Default::default() },
                        child_frame_id: body.into(),
                        transform: schema::Transform { translation: [3.0, 4.0, 0.5], rotation_xyzw: [0.0, 0.0, 0.0, 1.0] },
                    },
                ] };
                let wire = schema::encode_tf_message_body(&tf).unwrap();
                let OwnerInput::Accepted(Some(observed)) = ingest_map_pose_tf(&mut adapter, &wire).unwrap() else {
                    panic!("canonical bound TF must produce the actual body pose");
                };
                assert_eq!(observed.pose.pose.position, [4.0, 6.0, 0.5]);
                let mut goal = sealed_goal("canonical-bound-goal");
                goal.body_reference_frame_id = body.into();
                assert!(decode_configured_goal_envelope(&schema::encode_goal_envelope_body(&goal).unwrap(), body, None).is_ok());
                let odom = schema::Odometry {
                    header: schema::Header { stamp_sec: 7, frame_id: "odom".into(), ..Default::default() },
                    child_frame_id: format!(" {body} "),
                    pose: schema::Pose { position: [3.0, 4.0, 0.5], orientation_xyzw: [0.0, 0.0, 0.0, 1.0] },
                    ..Default::default()
                };
                assert!(adapter.transform_odometry(&odom).unwrap().is_some(), "existing whitespace normalization is retained");
            }
        }
    }

    #[test]
    fn retained_goal_replay_ignores_send_time_and_signed_zero_but_keeps_actions() {
        let mut accepted = sealed_goal("assigned-replay");
        accepted.goal.pose.position[1] = 0.0;
        let mut replay = accepted.clone();
        replay.source_timestamp_ns += 1;
        replay.goal.header.stamp_nsec += 1;
        replay.goal.header.seq += 1;
        replay.goal.pose.position[1] = -0.0;
        assert_eq!(validate_goal_envelope_order(&replay, &accepted), Err(PlannerGoalRejectReason::DuplicateLease));
        replay.goal.pose.position[0] += 0.1;
        assert_eq!(validate_goal_envelope_order(&replay, &accepted), Err(PlannerGoalRejectReason::ConflictingLease));
        replay = accepted.clone();
        replay.options.allow_reverse = !replay.options.allow_reverse;
        assert_eq!(validate_goal_envelope_order(&replay, &accepted), Err(PlannerGoalRejectReason::ConflictingLease));
        replay = accepted.clone();
        replay.producer_epoch += 1;
        assert_eq!(validate_goal_envelope_order(&replay, &accepted), Err(PlannerGoalRejectReason::ConflictingLease));
    }

    #[tokio::test]
    async fn guarded_cancel_acquires_and_fences_a_buffered_goal() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = sealed_goal("buffered-cancel-goal");
        harness.send_goal_and_wait_dequeued(&goal).await;
        harness.assert_no_session().await;

        harness.send_cancel(&goal, 1);
        harness.wait_for_zero().await;
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().last_terminal
                != Some(nuwax_nav_runtime::TerminalOutcome::Cancelled)
            {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("buffered goal cancellation did not become terminal");
        let terminal = harness.wait_for_correlated_zero(&goal, 1).await;
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(terminal.planner_runtime_generation, 0);
        assert!(harness.scheduler.status().session_id.is_none());
        assert!(harness
            .motion_calls
            .lock()
            .unwrap()
            .iter()
            .any(|call| call.starts_with("acquire:")));

        let motion_call_count = harness.motion_calls.lock().unwrap().len();
        harness.send_cancel(&goal, 1);
        let replay = harness.wait_for_correlated_zero(&goal, 1).await;
        assert_eq!(replay.zero_ack_id, terminal.zero_ack_id);
        assert_eq!(
            replay.zero_applied_monotonic_ns,
            terminal.zero_applied_monotonic_ns
        );
        tokio::time::sleep(Duration::from_millis(30)).await;
        assert_eq!(
            harness.motion_calls.lock().unwrap().len(),
            motion_call_count,
            "exact cancel retry must replay proof without reacquiring motion"
        );

        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.assert_no_session().await;
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_new_cancel_generation_reacquires_before_exact_replay() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = sealed_goal("buffered-cancel-new-generation");
        harness.send_goal_and_wait_dequeued(&goal).await;

        harness.send_cancel(&goal, 1);
        let first = harness.wait_for_correlated_zero(&goal, 1).await;
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        assert_eq!(harness.motion_call_count("zero:"), 1);

        harness.send_cancel(&goal, 2);
        let second = harness.wait_for_correlated_zero(&goal, 2).await;
        assert_eq!(harness.motion_call_count("acquire:"), 2);
        assert_eq!(harness.motion_call_count("zero:"), 2);

        harness.send_cancel(&goal, 2);
        let replay = harness.wait_for_correlated_zero(&goal, 2).await;
        assert_eq!(replay.zero_ack_id, second.zero_ack_id);
        assert_eq!(
            replay.zero_applied_monotonic_ns,
            second.zero_applied_monotonic_ns
        );
        assert_eq!(harness.motion_call_count("acquire:"), 2);
        assert_eq!(harness.motion_call_count("zero:"), 2);

        harness.send_cancel_and_wait_dequeued(&goal, 1).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(harness.motion_call_count("acquire:"), 2);
        assert_eq!(harness.motion_call_count("zero:"), 2);
        assert!(!first.zero_ack_id.is_empty());
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_busy_buffered_cancel_retains_the_goal_for_retry() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let buffered_goal = sealed_goal("buffered-cancel-busy-retry");
        let blocker = sealed_goal("direct-scheduler-blocker");
        harness
            .scheduler
            .start_session(nuwax_nav_runtime::NavSession {
            boundary: Vec::new(),
            options: Default::default(),
                goal: nuwax_nav_runtime::ExecutionGoal::Body(pose_from_schema(&blocker.goal).expect("valid goal pose")),
                session_id: blocker.session_id.clone(),
                execution_identity: guarded_goal_execution_identity(&blocker),

                motion_admission: nuwax_nav_runtime::NavMotionAdmission {

                    guard_boot_nonce: blocker.motion_guard_boot_nonce,
                    motion_stop_generation: u64::try_from(blocker.motion_stop_generation)
                        .expect("non-negative blocker stop generation"),
                },
            })
            .await
            .expect("install direct scheduler blocker");
        harness.send_goal_and_wait_dequeued(&buffered_goal).await;

        harness
            .send_cancel_and_wait_dequeued(&buffered_goal, 2)
            .await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(
            harness.scheduler.status().session_id.as_deref(),
            Some(blocker.session_id.as_str())
        );
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        assert_eq!(harness.motion_call_count("zero:"), 0);

        harness
            .scheduler
            .cancel(&blocker.session_id)
            .await
            .expect("remove direct scheduler blocker");
        harness
            .send_cancel_and_wait_dequeued(&buffered_goal, 1)
            .await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(harness.motion_call_count("acquire:"), 1);
        assert_eq!(harness.motion_call_count("zero:"), 1);

        harness.send_cancel(&buffered_goal, 2);
        let terminal = harness.wait_for_correlated_zero(&buffered_goal, 2).await;
        assert!(!terminal.zero_ack_id.is_empty());
        assert_eq!(harness.motion_call_count("acquire:"), 2);
        assert_eq!(harness.motion_call_count("zero:"), 2);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_replacement_clears_cancel_reacquire_eligibility() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal_a = sealed_goal("cancelled-before-replacement");
        harness.send_goal_and_wait_dequeued(&goal_a).await;
        harness.send_cancel(&goal_a, 1);
        harness.wait_for_correlated_zero(&goal_a, 1).await;

        let goal_b = rollover_goal(
            &goal_a,
            "buffered-replacement",
            goal_a.producer_runtime_generation,
            goal_a.goal_generation + 1,
        );
        harness.send_goal_and_wait_dequeued(&goal_b).await;
        let acquire_count = harness.motion_call_count("acquire:");
        let zero_count = harness.motion_call_count("zero:");

        harness.send_cancel_and_wait_dequeued(&goal_a, 2).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(harness.motion_call_count("acquire:"), acquire_count);
        assert_eq!(harness.motion_call_count("zero:"), zero_count);

        harness.send_cancel(&goal_b, 1);
        harness.wait_for_correlated_zero(&goal_b, 1).await;
        assert_eq!(harness.motion_call_count("acquire:"), acquire_count + 1);
        assert_eq!(harness.motion_call_count("zero:"), zero_count + 1);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_loop_rejects_old_planner_generation_as_the_first_path() {
        let harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        let mut late = sealed_path(&goal);
        late.planner_runtime_generation -= 1;
        schema::validate_path_envelope(&late).expect("seal old-generation late path");

        harness.send_path(&late);
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(
            harness.scheduler.status().path_revision,
            0,
            "old generation must be rejected before any current-generation path is accepted"
        );

        let current = sealed_path(&goal);
        harness.send_path(&current);
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
        .expect("current-generation path was not accepted");

        let mut higher_revision = current;
        higher_revision.path_revision = 2;
        higher_revision.path.header.stamp_nsec += 1;
        schema::validate_path_envelope(&higher_revision)
            .expect("seal same-generation higher-revision path");
        harness.send_path(&higher_revision);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 2 {
                assert!(
                    !harness.loop_task.is_finished(),
                    "guarded loop exited early"
                );
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("same-generation higher path revision was not accepted");
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_active_cancel_projects_pre_path_sentinel_with_zero_proof() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        harness.send_path(&sealed_path(&goal));
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
        .expect("active path was not accepted");

        harness.send_cancel(&goal, 2);
        harness.wait_for_zero().await;
        let terminal = harness.wait_for_correlated_zero(&goal, 2).await;
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(terminal.planner_runtime_generation, 0);
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_active_cancel_same_generation_replays_terminal_without_refencing() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;

        harness.send_cancel(&goal, 3);
        harness.wait_for_zero().await;
        let terminal = harness.wait_for_correlated_zero(&goal, 3).await;
        let acquire_count = harness.motion_call_count("acquire:");
        let zero_count = harness.motion_call_count("zero:");

        harness.send_cancel(&goal, 3);
        let replay = harness.wait_for_correlated_zero(&goal, 3).await;
        assert_eq!(replay.zero_ack_id, terminal.zero_ack_id);
        assert_eq!(
            replay.zero_applied_monotonic_ns,
            terminal.zero_applied_monotonic_ns
        );
        assert_eq!(harness.motion_call_count("acquire:"), acquire_count);
        assert_eq!(harness.motion_call_count("zero:"), zero_count);
        harness.shutdown().await;
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
        harness.publish_stopped_goal_pose(goal_a).await;
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

    #[tokio::test]
    async fn guarded_succeeded_terminal_rejects_late_path_without_losing_zero_ack() {
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
        harness.publish_stopped_goal_pose(&goal).await;
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().last_terminal != Some(TerminalOutcome::Succeeded) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("scheduler success terminal");
        let terminal = harness.scheduler.status();
        assert_eq!(terminal.phase, SchedulerPhase::Idle);
        let zero_ack = terminal
            .last_zero_ack_id
            .clone()
            .expect("Succeeded terminal ZeroAck");
        assert!(!zero_ack.is_empty());

        harness.send_path(&path);
        let mut wrong_identity = path.clone();
        wrong_identity.session_id.push_str("-wrong");
        schema::validate_path_envelope(&wrong_identity).expect("seal wrong identity path");
        harness.send_path(&wrong_identity);
        tokio::time::sleep(Duration::from_millis(50)).await;
        let after_late = harness.scheduler.status();
        assert_eq!(after_late.phase, SchedulerPhase::Idle);
        assert_eq!(after_late.last_terminal, Some(TerminalOutcome::Succeeded));
        assert_eq!(
            after_late.last_zero_ack_id.as_deref(),
            Some(zero_ack.as_str())
        );
        harness.shutdown().await;
    }

    #[tokio::test]
    async fn guarded_active_same_revision_mutation_preserves_original_execution() {
        let mut harness = GuardedLoopHarness::spawn().await;
        let goal = activate_map_a(&harness).await;
        let path = sealed_path(&goal);
        harness.send_path(&path);
        tokio::time::timeout(Duration::from_secs(1), async {
            while harness.scheduler.status().path_revision != 1 {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("original path accepted");

        let mut mutation = path;
        mutation.path.poses[0].pose.position[0] += 0.25;
        schema::validate_path_envelope(&mutation).expect("seal same-revision mutation");
        harness.send_path(&mutation);
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let frame = harness.states.next().await.unwrap().unwrap();
                let state = schema::decode_navigation_state_body(&frame.payload).unwrap();
                if state.message.contains("versioned path mutation detected") { break; }
            }
        }).await.expect("同修订变化须通过真实状态流发布拒绝诊断");
        let retained = harness.scheduler.status();
        assert_eq!(retained.phase, SchedulerPhase::Controlling);
        assert_eq!(retained.last_terminal, None);
        assert_eq!(retained.session_id.as_deref(), Some(goal.session_id.as_str()));
        assert_eq!(retained.path_revision, 1);
        assert!(retained.last_zero_ack_id.is_none());
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
    fn guarded_local_display_survives_heartbeats_and_clears_at_terminal() {
        let telemetry = GuardedNavTelemetry { public_session_id: "display-goal".into(), ..Default::default() };
        let mut status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Controlling,
            session_id: Some("display-goal".into()),
            local_path: vec![[0.0,0.0,0.4],[1.0,0.5,0.6]],
            ..Default::default()
        };
        for observation in [None, Some(false), Some(true)] {
            let wire = build_guarded_navigation_state(&status, &telemetry, observation, Duration::from_secs(1));
            let decoded = schema::decode_navigation_state(&schema::encode_navigation_state(&wire).unwrap()).unwrap();
            assert_eq!(decoded.local_path, status.local_path);
        }
        status.last_terminal = Some(nuwax_nav_runtime::TerminalOutcome::Succeeded);
        assert!(build_guarded_navigation_state(&status, &telemetry, None, Duration::from_secs(2)).local_path.is_empty());
        status.last_terminal = None;
        status.session_id = Some("old-goal".into());
        assert!(build_guarded_navigation_state(&status, &telemetry, None, Duration::from_secs(3)).local_path.is_empty());
        status.session_id = Some("display-goal".into());
        status.path_revision = 1;
        assert!(build_guarded_navigation_state(&status, &telemetry, None, Duration::from_secs(4)).local_path.is_empty());
    }

    #[test]
    fn guarded_navigation_state_receipt_projection_is_read_only() {
        let telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            ..GuardedNavTelemetry::default()
        };
        let baseline_status = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::AwaitingPath,
            session_id: Some("goal-a".into()),
            ..nuwax_nav_runtime::SchedulerStatus::default()
        };
        let baseline = build_guarded_navigation_state(
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
            build_guarded_navigation_state(&status, &telemetry, None, Duration::from_secs(1));
        assert_eq!(projected.pose_source_timestamp_ns, 11);
        assert_eq!(projected.costmap_source_timestamp_ns, 17);
        assert_eq!(
            build_guarded_navigation_state(&status, &telemetry, None, Duration::from_secs(1)),
            projected,
            "replaying the same scheduler snapshot must not advance receipt identity",
        );

        let mut without_receipts = projected.clone();
        without_receipts.pose_source_timestamp_ns = 0;
        without_receipts.costmap_source_timestamp_ns = 0;
        assert_eq!(
            without_receipts, baseline,
            "projection must change no other field"
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
            build_guarded_navigation_state(&rejected, &telemetry, None, Duration::from_secs(1));
        assert_eq!(published.state, 1, "rejection must not fabricate FAILED");
    }

    #[test]
    fn late_path_rejection_keeps_parking_and_terminal_cause_and_receipts() {
        for (phase,terminal) in [
            (SchedulerPhase::Fencing,None),(SchedulerPhase::Cancelling,None),
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
    fn terminal_cleanup_preserves_every_qualified_outcome() {
        for (outcome, phase) in [
            (TerminalOutcome::Cancelled, SchedulerPhase::Idle),
            (TerminalOutcome::PlannerFailed, SchedulerPhase::Failed),
            (TerminalOutcome::SafetyFailed, SchedulerPhase::Failed),
            (TerminalOutcome::Shutdown, SchedulerPhase::Idle),
        ] {
            let mut active = Some(active_goal("terminal-matrix"));
            let status = nuwax_nav_runtime::SchedulerStatus {
                phase,
                last_terminal: Some(outcome),
                last_zero_ack_id: Some("matrix-zero-ack".into()),
                last_fence_monotonic_ns: 42,
                ..nuwax_nav_runtime::SchedulerStatus::default()
            };
            assert!(clear_terminal_active_session(&mut active, &status));
            assert!(active.is_none());
            assert_eq!(status.last_terminal, Some(outcome));
            assert_eq!(status.last_zero_ack_id.as_deref(), Some("matrix-zero-ack"));
            assert_eq!(status.last_fence_monotonic_ns, 42);
        }
    }

    #[test]
    fn terminal_publication_transitions_to_unowned_guard_heartbeat() {
        let mut active = Some(active_goal("goal-a"));
        let terminal = nuwax_nav_runtime::SchedulerStatus {
            phase: SchedulerPhase::Idle,
            motion_sink_ready: true,

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
            current_z: 0.0,
            goal_z: 0.0,
            surface_navigation: false,
            initial_distance: 3.0,
            progress: 0.5,
            path_revision: 3,
            producer_runtime_generation: 11,
            planner_runtime_generation: 22,
        };

        let terminal_message =
            build_guarded_navigation_state(&terminal, &telemetry, None, Duration::from_secs(1));
        assert_eq!(terminal_message.session_id, "goal-a");
        assert_eq!(terminal_message.producer_runtime_generation, 11);
        assert_eq!(terminal_message.planner_runtime_generation, 22);
        assert_eq!(terminal_message.runtime_generation, 0);
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
        assert_eq!(heartbeat.sink_ready, 1);
        assert_eq!(heartbeat.cancel_generation, 0);
        assert!(heartbeat.zero_ack_id.is_empty());
        assert_eq!(heartbeat.zero_applied_monotonic_ns, 0);
        assert_eq!(heartbeat.owner_kind, 0);
        assert!(heartbeat.parent_request_id.is_empty());
        assert_eq!(heartbeat.goal_generation, 0);
        assert_eq!(heartbeat.path_revision, 0);
        assert_eq!(heartbeat.producer_runtime_generation, 0);
        assert_eq!(heartbeat.planner_runtime_generation, 0);
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
        foreign_path.goal_generation += 1;
        assert!(validate_path_goal_correlation(&foreign_path, &goal).is_err());

        let mut wrong_producer_generation = path.clone();
        wrong_producer_generation.producer_runtime_generation += 1;
        assert!(validate_path_goal_correlation(&wrong_producer_generation, &goal).is_err());

        let authoritative = saturating_i64(runtime_generation());
        assert_eq!(runtime_generation(), runtime_generation());
        assert!(planner_generation_matches_authority(
            authoritative,
            authoritative
        ));
        assert!(!planner_generation_matches_authority(
            authoritative - 1,
            authoritative
        ));
        assert!(!planner_generation_matches_authority(
            authoritative + 1,
            authoritative
        ));
        assert!(
            !planner_generation_matches_authority(authoritative - 1, authoritative)
                && !planner_generation_is_stale(None, goal.goal_key(), authoritative - 1),
            "an old-process late path is rejected even when it arrives before the new generation"
        );
        assert!(
            planner_generation_matches_authority(authoritative, authoritative)
                && !planner_generation_is_stale(
                    Some((goal.goal_key(), authoritative)),
                    goal.goal_key(),
                    authoritative,
                ),
            "same-generation higher path revisions remain eligible for scheduler ordering"
        );

        let accepted = Some((goal.goal_key(), 12));
        assert!(planner_generation_is_stale(
            accepted,
            goal.goal_key(),
            11
        ));
        assert!(!planner_generation_is_stale(
            accepted,
            goal.goal_key(),
            12
        ));
        assert!(!planner_generation_is_stale(
            accepted,
            goal.goal_key(),
            13
        ));
        let mut new_goal_key = goal.goal_key();
        new_goal_key.producer_runtime_generation += 1;
        assert!(!planner_generation_is_stale(accepted, new_goal_key, 1));

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
        schema::validate_goal_envelope(&goal_a).expect("seal producer 41 goal");

        let mut goal_b = goal_a.clone();
        goal_b.producer_runtime_generation = 42;
        goal_b.source_timestamp_ns += 1_000_000_000;
        schema::validate_goal_envelope(&goal_b).expect("seal producer 42 goal");

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
        schema::validate_goal_envelope(&goal_a).expect("seal producer 41 goal");
        harness.send_pose(1);
        harness.wait_for_pose(1).await;
        harness.send_costmap(11);
        harness.wait_for_costmap(11).await;
        harness.send_goal(&goal_a);
        harness.wait_for_session(&goal_a.session_id).await;

        let mut goal_b = goal_a.clone();
        goal_b.producer_runtime_generation = 42;
        goal_b.source_timestamp_ns += 1_000_000_000;
        schema::validate_goal_envelope(&goal_b).expect("seal producer 42 goal");
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
            CommandingPlanner {
                succeeds: Arc::new(std::sync::atomic::AtomicBool::new(false)),
                twist: Arc::new(Mutex::new(Twist2 {
                    linear: Point2::new(0.2, 0.0),
                    angular_z: 0.1,
                })),
            },
            nuwax_nav_runtime::SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                renew_interval: Duration::from_millis(100),
                path_wait_timeout: Duration::from_secs(5),
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
                costmap: nuwax_nav_runtime::ExecutionMap::Planar(
                    nuwax_nav_astar::navigation_map::CostMap2D::new_all_free("map", 4, 4, 0.1),
                ),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("map-a costmap");
        let goal_a = sealed_goal("session-a");
        let mut path_identity = guarded_goal_execution_identity(&goal_a);
        path_identity.planner_runtime_generation = 1;
        path_identity.path_revision = 1;
        handle
            .start_session(nuwax_nav_runtime::NavSession {
            boundary: Vec::new(),
            options: Default::default(),
                goal: nuwax_nav_runtime::ExecutionGoal::Body(pose_from_schema(&goal_a.goal).expect("valid goal pose")),
                session_id: goal_a.session_id.clone(),
                execution_identity: guarded_goal_execution_identity(&goal_a),

                motion_admission: nuwax_nav_runtime::NavMotionAdmission {

                    guard_boot_nonce: goal_a.motion_guard_boot_nonce,
                    motion_stop_generation: u64::try_from(goal_a.motion_stop_generation)
                        .expect("non-negative stop generation"),
                },
            })
            .await
            .expect("map-a session");
        handle
            .submit_path(nuwax_nav_runtime::VersionedPath {
                path_yaws: None,
                session_id: goal_a.session_id.clone(),
                execution_identity: path_identity,
                path_revision: 1,
                path: nuwax_nav_runtime::ExecutionPath::Planar(Path2D::stamped(
                    NavFrameId::Map,
                    Timestamp::from_unix(1, 0),
                    vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                )),
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
            producer_runtime_generation: 22,
            planner_runtime_generation: 33,
            ..GuardedNavTelemetry::default()
        };
        fence_guarded_active_session(&handle, &mut telemetry, &mut active_session)
            .await
            .expect("pending map-b goal fences map-a");
        assert!(active_session.is_none());
        assert_eq!(telemetry.path_revision, 0);
        assert_eq!(telemetry.producer_runtime_generation, 0);
        assert_eq!(telemetry.planner_runtime_generation, 0);
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
            OwnerInput::Accepted(None)
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
    fn queued_pose_keeps_latest_valid_sample_and_preserves_stream_failure() {
        let (sender, mut poses) = guarded_test_stream();
        let make = |stamp| {
            build_body_frame(
                "/test/pose",
                "geometry_msgs.PoseStamped",
                guarded_pose_body(stamp),
            )
            .unwrap()
        };
        sender.unbounded_send((Ok(make(3)), None, None)).unwrap();
        sender.unbounded_send((Ok(make(2)), None, None)).unwrap();
        sender
            .unbounded_send((
                Ok(build_body_frame("/test/pose", "geometry_msgs.PoseStamped", vec![255]).unwrap()),
                None,
                None,
            ))
            .unwrap();
        let mut diagnostics = InputDiagnosticLimiter::default();
        let input = drain_guarded_pose_input(Some(Ok(make(1))), &mut poses, &mut diagnostics, None)
            .unwrap()
            .unwrap();
        assert_eq!(input.pose.stamp.nanos, 3_000_000_000);
        drop(sender);
        assert!(
            drain_guarded_pose_input(Some(Ok(make(4))), &mut poses, &mut diagnostics, None)
                .is_err()
        );
    }

    #[test]
    fn queued_future_pose_cannot_discard_a_valid_sample() {
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
            Some(Duration::from_millis(500)),
        )
        .unwrap()
        .unwrap();
        assert_eq!(input.pose.stamp.nanos, current.as_nanos() as i64);
    }

    #[test]
    fn status_publications_reset_per_goal_and_publish_success_once() {
        let mut node = ModuleNode::new(
            "guard-test",
            "守卫状态",
            crate::graph::ModuleKind::Coordinator,
        );
        node.outputs = vec![
            "/test/cmd".into(),
            "/test/state".into(),
            "/test/reached".into(),
        ];
        let mut publications = GuardedStatusPublications::default();
        let mut telemetry = GuardedNavTelemetry {
            public_session_id: "goal-a".into(),
            goal_generation: 1,
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
        status.last_zero_ack_id = Some("zero-a".into());
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
        telemetry.goal_generation = 2;
        status.last_terminal = None;
        status.last_zero_ack_id = None;
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
    fn pose_source_deadline_uses_original_sampling_time() {
        let wall = std::time::UNIX_EPOCH + Duration::from_secs(10);
        let mono = tokio::time::Instant::now();
        let maximum = Duration::from_millis(500);
        let deadline = guarded_pose_source_deadline(9_600_000_000, maximum, wall, mono).unwrap();
        assert_eq!(deadline.duration_since(mono), Duration::from_millis(100));
        assert!(guarded_pose_source_deadline(9_500_000_000, maximum, wall, mono).is_err());
        assert!(guarded_pose_source_deadline(10_051_000_000, maximum, wall, mono).is_err());
    }

    #[test]
    fn map_pose_output_fence_accepts_only_strictly_newer_source_timestamps() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let atomic_tf = schema::encode_tf_message_body(&schema::TfMessage {
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
        .expect("encode atomic TF");
        let tf_observation =
            match ingest_map_pose_tf(&mut adapter, &atomic_tf).expect("coherent TF observation") {
                OwnerInput::Accepted(Some(observed)) => observed,
                OwnerInput::Accepted(None) => panic!("coherent TF did not produce a map pose"),
                OwnerInput::Rejected(reason) => panic!("coherent TF rejected: {reason}"),
            };
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
        let odometry_body = schema::encode_odometry_body(&odometry).expect("encode odometry");
        let odometry_observation = match ingest_map_pose_odometry(&adapter, &odometry_body)
            .expect("aligned odometry observation")
        {
            MapPoseOdometryIngress::Accepted(observed) => observed,
            MapPoseOdometryIngress::AwaitingTransform(_, reason) => {
                panic!("aligned odometry still waiting: {reason}")
            }
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("aligned odometry rejected: {reason}")
            }
        };

        let last_published_timestamp_ns =
            match fence_map_pose_source_timestamp(None, &tf_observation)
                .expect("first observation timestamp")
            {
                OwnerInput::Accepted(timestamp_ns) => Some(timestamp_ns),
                OwnerInput::Rejected(reason) => panic!("first observation rejected: {reason}"),
            };
        assert!(matches!(
            fence_map_pose_source_timestamp(last_published_timestamp_ns, &odometry_observation)
                .expect("duplicate timestamp is recoverable"),
            OwnerInput::Rejected(_)
        ));

        let mut older_observation = odometry_observation.clone();
        older_observation.pose.header.stamp_sec = 6;
        assert!(matches!(
            fence_map_pose_source_timestamp(last_published_timestamp_ns, &older_observation)
                .expect("older timestamp is recoverable"),
            OwnerInput::Rejected(_)
        ));

        let mut newer_observation = odometry_observation;
        newer_observation.pose.header.stamp_sec = 8;
        assert!(matches!(
            fence_map_pose_source_timestamp(last_published_timestamp_ns, &newer_observation)
                .expect("newer timestamp"),
            OwnerInput::Accepted(8_000_000_000)
        ));
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
    fn map_pose_owner_keeps_newer_pending_odometry_after_older_atomic_tf() {
        let mut adapter =
            crate::map_pose::MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let pending_odometry = schema::Odometry {
            header: schema::Header {
                stamp_sec: 8,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            child_frame_id: "base_link".into(),
            pose: schema::Pose {
                position: [5.0, 6.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        };
        let mut pending = match transform_map_pose_odometry(&adapter, pending_odometry)
            .expect("odometry before TF is retryable")
        {
            MapPoseOdometryIngress::AwaitingTransform(odometry, _) => Some(odometry),
            MapPoseOdometryIngress::Accepted(_) => panic!("odometry unexpectedly accepted"),
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("retryable odometry rejected: {reason}")
            }
        };

        let atomic_tf = schema::encode_tf_message_body(&schema::TfMessage {
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
        .expect("encode atomic TF");
        let atomic = match ingest_map_pose_tf(&mut adapter, &atomic_tf).expect("atomic TF input") {
            OwnerInput::Accepted(Some(observed)) => observed,
            OwnerInput::Accepted(None) => panic!("coherent TF did not produce a map pose"),
            OwnerInput::Rejected(reason) => panic!("coherent TF rejected: {reason}"),
        };
        assert_eq!(atomic.pose.header.stamp_sec, 7);
        assert_eq!(atomic.pose.pose.position, [4.0, 6.0, 0.0]);
        discard_pending_odometry_through(&mut pending, &atomic)
            .expect("compare atomic pose with pending odometry");
        assert_eq!(
            pending.as_ref().map(|odometry| odometry.header.stamp_sec),
            Some(8),
            "newer odometry must remain available for a later aligned transform"
        );

        let aligned_tf = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 8,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "odom".into(),
                transform: schema::Transform {
                    translation: [2.0, 3.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode aligned TF");
        assert!(matches!(
            ingest_map_pose_tf(&mut adapter, &aligned_tf).expect("aligned TF input"),
            OwnerInput::Accepted(None)
        ));
        let observed = match transform_map_pose_odometry(
            &adapter,
            *pending.take().expect("newer odometry retained"),
        )
        .expect("retry retained odometry")
        {
            MapPoseOdometryIngress::Accepted(observed) => observed,
            MapPoseOdometryIngress::AwaitingTransform(_, reason) => {
                panic!("aligned odometry still waiting: {reason}")
            }
            MapPoseOdometryIngress::Rejected(reason) => {
                panic!("aligned odometry rejected: {reason}")
            }
        };
        assert_eq!(observed.pose.header.stamp_sec, 8);
        assert_eq!(observed.pose.pose.position, [7.0, 9.0, 0.0]);
        assert!(pending.is_none());
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

        let mut planner = LocalPlanner::with_config(LocalPlannerConfig::default_config());
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

        let (_, stop) = planner.step(0.3);
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
        let (twist,stop)=planner.step(0.3);
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
        let mut buffer = PlannerGoalBuffer::default();
        assert!(matches!(
            buffer.observe_goal(goal.clone()),
            PlannerGoalIngress::Buffered
        ));
        assert_eq!(buffer.pending_goal.as_ref(), Some(&goal));

        let ready = buffer
            .observe_costmap()
            .expect("first valid costmap must release the buffered A* goal");
        assert_eq!(ready.goal_key(), expected_key);
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
        .expect("A* map-b path stays inside boundary");
        assert_eq!(
            envelope.goal_key(),
            active_goal.as_ref().unwrap().goal_key()
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
            body_reference_frame_id: "base_link".into(),
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

    fn goal() -> schema::GoalEnvelope {
        let mut goal = schema::GoalEnvelope {
            body_reference_frame_id: "base_link".into(),
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: "native-session".into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            parent_request_id: "native-parent".into(),
            goal_generation: 9,
            producer: "wavefront".into(),
            producer_epoch: 3,
            producer_runtime_generation: 4,
            committed_map_session_id: "map-session".into(),
            committed_map_session_generation: 5,
            committed_map_revision: 6,
            committed_map_boot_nonce: [0x21; 16],
            committed_map_commit_sequence: 1,
            committed_map_source_timestamp_ns: 1,
            committed_map_frame_id: "map".into(),
            source_timestamp_ns: 7_000_000_000,
            boundary_id: "native-boundary".into(),
            boundary_frame_id: "map".into(),
            boundary_points: vec![
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 5.0, 0.0],
                [0.0, 5.0, 0.0],
                [0.0, 0.0, 0.0],
            ],

            motion_guard_boot_nonce: [0x31; 16],
            motion_stop_generation: 8,
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
            ..schema::GoalEnvelope::default()
        };

        schema::validate_goal_envelope(&goal).expect("seal native goal");
        goal
    }

    fn path(points: Vec<Point2>) -> Path2D {
        Path2D::stamped(NavFrameId::Map, Timestamp::from_unix(7, 0), points)
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    fn goal_generation(generation: i64) -> schema::GoalEnvelope {
        let mut value = goal();
        value.goal_generation = generation;
        value.source_timestamp_ns = generation * 1_000_000_000;
        value.goal.header.seq = i32::try_from(generation).expect("test generation fits i32");
        value.goal.header.stamp_sec = i32::try_from(generation).expect("test generation fits i32");
        schema::validate_goal_envelope(&value).expect("reseal native goal generation");
        value
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    fn far_key(goal: &schema::GoalEnvelope) -> FarJoinKey {
        FarJoinKey {
            backend: "far-visibility-graph".into(),
            runtime_generation: 12,
            session_id: goal.session_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
            path_revision: 4,
            planner_runtime_generation: 12,
        }
    }

    #[cfg(all(
        feature = "native-runner",
        any(feature = "cmu-native", feature = "explore")
    ))]
    #[test]
    fn far_readiness_rejects_one_wrong_channel() {
        let expected = nuwax_nav_far_planner_bridge::worker::FarReadyContract {
            backend: "far-visibility-graph".into(),
            generation: 0,
            goal: "/goal".into(),
            cancel: "/cancel".into(),
            odometry: "/odometry".into(),
            terrain_map_ext: "/terrain-ext".into(),
            terrain_map: "/terrain".into(),
            registered_scan: "/scan".into(),
            planner_result: "/result".into(),
            worker_event: "/event".into(),
            result_contract: nuwax_nav_far_planner_bridge::worker::FAR_RESULT_V3_CONTRACT.into(),
        };
        let mut published = expected.clone();
        published.generation = 12;
        let mut state = schema::NavigationState {
            robot_frame: expected.backend.clone(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_READY,
            runtime_generation: 12,
            sink_ready: 1,
            message: published.encode_v2(),
            ..schema::NavigationState::default()
        };
        assert!(far_ready_contract_matches(&expected, 12, &state));
        published.result_contract = "far-private-result-v2".into();
        state.message = published.encode_v2();
        assert!(!far_ready_contract_matches(&expected, 12, &state));
        published.result_contract = nuwax_nav_far_planner_bridge::worker::FAR_RESULT_V3_CONTRACT.into();
        published.odometry = "/wrong-odometry".into();
        state.message = published.encode_v2();
        assert!(!far_ready_contract_matches(&expected, 12, &state));
    }


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
    fn stale_goal_preserves_newer_far_result_until_terminal() {
        let stale = goal_generation(9);
        let current = goal_generation(10);
        let key = far_key(&current);
        let mut latest = None;
        assert!(admit_native_adapter_goal(&mut latest, stale.clone()));
        assert!(admit_native_adapter_goal(&mut latest, current.clone()));
        let mut pending_results = HashMap::from([(
            key.clone(),
            path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]),
        )]);
        let mut pending_terminals = HashSet::new();
        let mut consumed = HashSet::new();

        assert!(!admit_native_adapter_goal(&mut latest, stale));
        assert_eq!(latest.as_ref(), Some(&current));
        pending_terminals.insert(key.clone());
        assert!(take_far_join(
            &key,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_some());
        assert!(consumed.contains(&key));
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
        wrong_map.committed_map_frame_id = "odom".into();
        assert!(!native_path_respects_goal(&inside, &wrong_map));

        let mut wrong_boundary = goal;
        wrong_boundary.boundary_frame_id = "odom".into();
        assert!(!native_path_respects_goal(&inside, &wrong_boundary));
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn far_terminal_gate_rejects_old_future_and_wrong_goal_identity() {
        let goal = goal();
        let state = schema::NavigationState {
            session_id: goal.session_id.clone(),
            robot_frame: "far-visibility-graph".into(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED,
            runtime_generation: 12,
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            path_revision: 4,
            producer_runtime_generation: goal.producer_runtime_generation,
            planner_runtime_generation: 12,
            ..schema::NavigationState::default()
        };
        assert!(far_terminal_matches_goal(&state, &goal, 12));
        assert!(!far_terminal_matches_goal(&state, &goal, 11));
        assert!(!far_terminal_matches_goal(&state, &goal, 13));

        let mut wrong_goal = goal;
        wrong_goal.producer_runtime_generation += 1;
        schema::validate_goal_envelope(&wrong_goal).expect("reseal wrong goal");
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
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
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
        let node = ModuleNode::new(
            format!("far-adapter-{id}"),
            "FAR adapter real LCM regression",
            crate::graph::ModuleKind::Coordinator,
        )
        .add_input(format!("{prefix}/g"))
        .add_input(format!("{prefix}/e"))
        .add_input(format!("{prefix}/p"))
        .add_output(format!("{prefix}/o"));
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

        let goal = goal();
        publish_body(
            &transport,
            &node.inputs[0],
            "nuwax_msgs.GoalEnvelope",
            schema::encode_goal_envelope_body(&goal).expect("encode FAR goal"),
        )
        .await
        .expect("publish FAR goal");
        FAR_READY_GENERATION.store(12, Ordering::Release);

        let state = |revision| schema::NavigationState {
            session_id: goal.session_id.clone(),
            robot_frame: "far-visibility-graph".into(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED,
            runtime_generation: 12,
            planner_map_frame: goal.committed_map_frame_id.clone(),
            planner_pose_frame: goal.goal.header.frame_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            path_revision: revision,
            planner_runtime_generation: 12,
            producer_runtime_generation: goal.producer_runtime_generation,
            ..schema::NavigationState::default()
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
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&wrong_result).expect("encode wrong FAR result"),
        )
        .await
        .expect("publish wrong FAR result");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&wrong_terminal)
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
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&accepted_terminal)
                .expect("encode accepted FAR terminal"),
        )
        .await
        .expect("publish accepted FAR terminal first");
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&accepted_result)
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
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&result_first)
                .expect("encode result-first FAR result"),
        )
        .await
        .expect("publish FAR result first");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&terminal_second)
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
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&result_first).expect("encode replay FAR result"),
        )
        .await
        .expect("publish replay FAR result");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationState",
            schema::encode_navigation_state_body(&terminal_second)
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
