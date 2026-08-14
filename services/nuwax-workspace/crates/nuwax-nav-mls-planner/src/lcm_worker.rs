//! Real LCM worker for the MLS planner.
//!
//! The input task remains responsive while the independent planning task owns
//! the graph. Map updates cross that boundary through a Tokio watch channel,
//! so only the newest pending map is retained while an expensive update or
//! plan is running.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nuwax_codec::lcm::schema::{
    self, CancelNavigation, GoalEnvelope, Header, NavigationState, Path, PointCloud2, PointField,
    Pose, PoseStamped,
};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::{LcmTransport, OwnedLcmSubscription};
use thiserror::Error;
use tokio::sync::{mpsc, oneshot, watch, Mutex};
use validator::Validate;

use crate::edges::{edges_to_segments, PlannerGraph};
use crate::mls_planner::{Config, Planner, RegionBounds};
use crate::voxel::surface_point_xyz;

type Xyz = (f32, f32, f32);

/// Typed LCM channels used by the worker.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WorkerTopics {
    /// Complete map input.
    pub global_map: String,
    /// Incremental local-map input.
    pub local_map: String,
    /// Bounds paired with `local_map` by exact timestamp.
    pub region_bounds: String,
    /// Latest robot pose. It does not itself trigger replanning.
    pub start_pose: String,
    /// Strict V4 navigation goal envelope.
    pub goal: String,
    /// Correlated navigation cancellation input.
    pub cancel: String,
    /// Backend-private typed planner events and results.
    pub events: String,
    /// Extracted MLS surface visualization.
    pub surface_map: String,
    /// Planner nodes visualization.
    pub nodes: String,
    /// Planner edges encoded as paired Path poses.
    pub node_edges: String,
}

impl Default for WorkerTopics {
    fn default() -> Self {
        Self {
            global_map: "/mls/global_map".into(),
            local_map: "/mls/local_map".into(),
            region_bounds: "/mls/region_bounds".into(),
            start_pose: "/mls/start_pose".into(),
            goal: "/mls/goal".into(),
            cancel: "/mls/cancel".into(),
            events: "/mls/events".into(),
            surface_map: "/mls/surface_map".into(),
            nodes: "/mls/nodes".into(),
            node_edges: "/mls/node_edges".into(),
        }
    }
}

/// Complete worker configuration.
#[derive(Clone)]
pub struct WorkerOptions {
    /// liblcm provider URL.
    pub provider_url: String,
    /// Typed port channels.
    pub topics: WorkerTopics,
    /// Stable backend identity asserted by every worker event.
    pub backend_id: String,
    /// Restart generation asserted by READY and every terminal/result event.
    pub backend_generation: i64,
    /// Bounded deadline for one planning attempt.
    pub planning_timeout: Duration,
    /// MLS algorithm parameters.
    pub planner: Config,
}

impl Default for WorkerOptions {
    fn default() -> Self {
        Self {
            provider_url: std::env::var("NUWAX_LCM_URL")
                .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=1".into()),
            topics: WorkerTopics::default(),
            backend_id: std::env::var("NUWAX_MLS_BACKEND_ID").unwrap_or_else(|_| "mls3d".into()),
            backend_generation: 1,
            planning_timeout: Duration::from_millis(
                std::env::var("NUWAX_MLS_PLANNING_TIMEOUT_MS")
                    .ok()
                    .and_then(|value| value.parse().ok())
                    .unwrap_or(5_000),
            ),
            planner: Config {
                world_frame: "map".into(),
                voxel_size: 0.1,
                robot_height: 1.5,
                surface_closing_radius: 0.3,
                node_spacing_m: 1.0,
                wall_clearance_m: 0.3,
                wall_buffer_m: 0.75,
                wall_buffer_weight: 100.0,
                step_threshold_m: 0.25,
                step_penalty_weight: 4.0,
                goal_tolerance: 0.3,
                viz_publish_hz: 2.0,
            },
        }
    }
}

impl WorkerOptions {
    /// Build production worker options while rejecting an invalid generation override.
    pub fn from_env() -> Result<Self, WorkerError> {
        let mut options = Self::default();
        if let Some(value) = std::env::var_os("NUWAX_MLS_BACKEND_GENERATION") {
            let value = value
                .into_string()
                .map_err(|_| WorkerError::Config("backend generation is not UTF-8".into()))?;
            options.backend_generation = value.parse::<i64>().map_err(|_| {
                WorkerError::Config(
                    "NUWAX_MLS_BACKEND_GENERATION must be a positive integer".into(),
                )
            })?;
            if options.backend_generation <= 0 {
                return Err(WorkerError::Config(
                    "NUWAX_MLS_BACKEND_GENERATION must be a positive integer".into(),
                ));
            }
        }
        Ok(options)
    }
}

/// Worker startup, transport, or message failure.
#[derive(Debug, Error)]
pub enum WorkerError {
    /// Invalid command or planner configuration.
    #[error("invalid MLS worker configuration: {0}")]
    Config(String),
    /// Typed topic or message type could not be built.
    #[error("invalid MLS LCM contract: {0}")]
    Contract(String),
    /// Generated schema encoding or decoding failed.
    #[error("MLS codec failure: {0}")]
    Codec(String),
    /// The real transport failed or closed unexpectedly.
    #[error("MLS transport failure: {0}")]
    Transport(String),
}

const WORKER_PROTOCOL_MAGIC: &[u8; 4] = b"MLS1";
const WORKER_PROTOCOL_VERSION: u8 = 2;
const WORKER_KIND_READY: u8 = 1;
const WORKER_KIND_PLANNED: u8 = 2;
const WORKER_KIND_CANCELLED: u8 = 3;
const WORKER_KIND_TIMEOUT: u8 = 4;
const WORKER_KIND_CRASHED: u8 = 5;
const WORKER_KIND_REJECTED: u8 = 6;
const WORKER_FRAME: &str = "mls-planner-worker-v1";
const MAX_PRIVATE_EVENT_BYTES: usize = 4 * 1024 * 1024;
const MAX_POINT_CLOUD_POINTS: usize = 1_000_000;
const MAX_POINT_STEP_BYTES: usize = 1_024;
const MAX_POINT_CLOUD_DATA_BYTES: usize = 4 * 1024 * 1024;

/// Backend-private, fully correlated MLS planning result.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannerResult {
    /// Exact V4 goal accepted by this planning attempt.
    pub goal: GoalEnvelope,
    /// Restart generation of the backend that produced the result.
    pub backend_generation: i64,
    /// Raw algorithm path for the canonical adapter to wrap later.
    pub path: Path,
}

/// Channels owned by the worker generation when READY is published.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadyContract {
    /// Complete point-cloud map channel.
    pub global_map: String,
    /// Incremental point-cloud map channel.
    pub local_map: String,
    /// Region bounds channel paired with local maps.
    pub region_bounds: String,
    /// Robot start-pose channel.
    pub start_pose: String,
    /// Strict GoalEnvelope input channel.
    pub goal: String,
    /// Correlated cancellation channel.
    pub cancel: String,
    /// Backend-private event channel.
    pub events: String,
}

impl From<&WorkerTopics> for ReadyContract {
    fn from(topics: &WorkerTopics) -> Self {
        Self {
            global_map: topics.global_map.clone(),
            local_map: topics.local_map.clone(),
            region_bounds: topics.region_bounds.clone(),
            start_pose: topics.start_pose.clone(),
            goal: topics.goal.clone(),
            cancel: topics.cancel.clone(),
            events: topics.events.clone(),
        }
    }
}

/// Backend-private worker event transported as a strict typed state body.
#[derive(Debug, Clone, PartialEq)]
pub enum WorkerEvent {
    /// All required subscriptions are acknowledged and owned by input pumps.
    Ready(ReadyContract),
    /// The MLS algorithm produced a non-canonical planner result.
    Planned(PlannerResult),
    /// A matching cancellation invalidated the active goal.
    Cancelled {
        /// Exact goal invalidated by the cancellation.
        goal: GoalEnvelope,
        /// Monotonic cancellation generation accepted by the worker.
        cancel_generation: i64,
    },
    /// The bounded planning deadline expired.
    TimedOut(GoalEnvelope),
    /// A planning task panicked or failed to join.
    Crashed {
        /// Exact goal being planned when the task failed.
        goal: GoalEnvelope,
        /// Backend-local terminal detail.
        detail: String,
    },
    /// The worker rejected a semantically unusable but well-formed goal.
    Rejected {
        /// Exact goal rejected before planning.
        goal: GoalEnvelope,
        /// Backend-local rejection reason.
        detail: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct GoalIdentity {
    session_id: String,
    owner_kind: i32,
    parent_request_id: String,
    goal_generation: i64,
    producer_runtime_generation: i64,
}

impl From<&GoalEnvelope> for GoalIdentity {
    fn from(goal: &GoalEnvelope) -> Self {
        Self {
            session_id: goal.session_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
        }
    }
}

impl From<&CancelNavigation> for GoalIdentity {
    fn from(cancel: &CancelNavigation) -> Self {
        Self {
            session_id: cancel.session_id.clone(),
            owner_kind: cancel.owner_kind,
            parent_request_id: cancel.parent_request_id.clone(),
            goal_generation: cancel.goal_generation,
            producer_runtime_generation: cancel.producer_runtime_generation,
        }
    }
}

#[derive(Clone)]
struct GoalAttempt {
    token: u64,
    goal: GoalEnvelope,
}

#[derive(Clone)]
enum GoalCommand {
    Set(GoalAttempt),
    Cancel {
        attempt: GoalAttempt,
        cancel_generation: i64,
    },
}

struct CancelWatermark {
    generation: i64,
    reported: bool,
}

#[derive(Default)]
struct GoalState {
    next_token: u64,
    active: Option<GoalAttempt>,
    cancel_watermarks: HashMap<GoalIdentity, CancelWatermark>,
}

#[derive(Clone)]
struct EventPublisher {
    transport: LcmTransport,
    channel: String,
    backend_id: String,
    backend_generation: i64,
}

impl EventPublisher {
    async fn publish(&self, event: &WorkerEvent) -> Result<(), WorkerError> {
        let body = encode_worker_event_body(&self.backend_id, self.backend_generation, event)?;
        publish(
            &self.transport,
            &self.channel,
            "nuwax_msgs.NavigationState",
            body,
        )
        .await
    }
}

#[derive(Clone)]
enum MapUpdate {
    Region {
        cloud: PointCloud2,
        bounds: PoseStamped,
    },
    Global(PointCloud2),
}

enum InputEvent {
    Global(PointCloud2),
    Local(PointCloud2),
    Bounds(PoseStamped),
    Start(PoseStamped),
    Goal(Box<GoalEnvelope>),
    Cancel(CancelNavigation),
    Failed(String),
}

#[derive(Clone, Copy)]
enum CloudInput {
    Global,
    Local,
}

#[derive(Clone, Copy)]
enum PoseInput {
    Bounds,
    Start,
}

struct PumpSet(Vec<tokio::task::JoinHandle<()>>);

impl Drop for PumpSet {
    fn drop(&mut self) {
        for task in &self.0 {
            task.abort();
        }
    }
}

/// Start a real-liblcm worker and run until a required input stream fails.
pub async fn run_worker(options: WorkerOptions) -> Result<(), WorkerError> {
    options
        .planner
        .validate()
        .map_err(|error| WorkerError::Config(error.to_string()))?;
    let transport = LcmTransport::new_real(&options.provider_url)
        .map_err(|error| WorkerError::Transport(error.to_string()))?;
    run_with_transport(transport, options).await
}

/// Run against an already-created transport.
///
/// This entry point is useful for embedding the worker under another runtime
/// owner while preserving exactly the same typed-port behavior as the binary.
pub async fn run_with_transport(
    transport: LcmTransport,
    options: WorkerOptions,
) -> Result<(), WorkerError> {
    options
        .planner
        .validate()
        .map_err(|error| WorkerError::Config(error.to_string()))?;

    if options.backend_id.trim().is_empty()
        || options.backend_generation <= 0
        || options.planning_timeout.is_zero()
    {
        return Err(WorkerError::Config(
            "backend identity, positive generation, and planning timeout are required".into(),
        ));
    }

    let global = subscribe(
        &transport,
        &options.topics.global_map,
        "sensor_msgs.PointCloud2",
    )
    .await?;
    let local = subscribe(
        &transport,
        &options.topics.local_map,
        "sensor_msgs.PointCloud2",
    )
    .await?;
    let bounds = subscribe(
        &transport,
        &options.topics.region_bounds,
        "geometry_msgs.PoseStamped",
    )
    .await?;
    let start = subscribe(
        &transport,
        &options.topics.start_pose,
        "geometry_msgs.PoseStamped",
    )
    .await?;
    let goal = subscribe(&transport, &options.topics.goal, "nuwax_msgs.GoalEnvelope").await?;
    let cancel = subscribe(
        &transport,
        &options.topics.cancel,
        "nuwax_msgs.CancelNavigation",
    )
    .await?;

    let publisher = EventPublisher {
        transport: transport.clone(),
        channel: options.topics.events.clone(),
        backend_id: options.backend_id.clone(),
        backend_generation: options.backend_generation,
    };

    let (event_tx, event_rx) = mpsc::unbounded_channel();
    let (global_ready_tx, global_ready_rx) = oneshot::channel();
    let (local_ready_tx, local_ready_rx) = oneshot::channel();
    let (bounds_ready_tx, bounds_ready_rx) = oneshot::channel();
    let (start_ready_tx, start_ready_rx) = oneshot::channel();
    let (goal_ready_tx, goal_ready_rx) = oneshot::channel();
    let (cancel_ready_tx, cancel_ready_rx) = oneshot::channel();
    let pumps = PumpSet(vec![
        tokio::spawn(pump_cloud(
            global,
            CloudInput::Global,
            event_tx.clone(),
            global_ready_tx,
        )),
        tokio::spawn(pump_cloud(
            local,
            CloudInput::Local,
            event_tx.clone(),
            local_ready_tx,
        )),
        tokio::spawn(pump_pose(
            bounds,
            PoseInput::Bounds,
            event_tx.clone(),
            bounds_ready_tx,
        )),
        tokio::spawn(pump_pose(
            start,
            PoseInput::Start,
            event_tx.clone(),
            start_ready_tx,
        )),
        tokio::spawn(pump_goal(goal, event_tx.clone(), goal_ready_tx)),
        tokio::spawn(pump_cancel(cancel, event_tx, cancel_ready_tx)),
    ]);
    tokio::try_join!(
        pump_ready("global_map", global_ready_rx),
        pump_ready("local_map", local_ready_rx),
        pump_ready("region_bounds", bounds_ready_rx),
        pump_ready("start_pose", start_ready_rx),
        pump_ready("goal", goal_ready_rx),
        pump_ready("cancel", cancel_ready_rx),
    )?;
    publisher
        .publish(&WorkerEvent::Ready(ReadyContract::from(&options.topics)))
        .await?;

    let (map_tx, map_rx) = watch::channel::<Option<Arc<MapUpdate>>>(None);
    let (start_tx, start_rx) = watch::channel::<Option<Xyz>>(None);
    let (goal_tx, goal_rx) = watch::channel::<Option<GoalCommand>>(None);
    let goal_state = Arc::new(Mutex::new(GoalState::default()));

    let input_loop = handle_inputs(
        event_rx,
        map_tx,
        start_tx,
        goal_tx,
        publisher.clone(),
        goal_state.clone(),
        options.planner.world_frame.clone(),
    );
    let planning_loop = plan_and_publish(
        publisher,
        options.topics,
        options.planner,
        options.planning_timeout,
        PlanningInputs {
            map_rx,
            start_rx,
            goal_rx,
            goal_state,
        },
    );
    let result = tokio::try_join!(input_loop, planning_loop).map(|_| ());
    drop(pumps);
    result
}

async fn subscribe(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
) -> Result<OwnedLcmSubscription, WorkerError> {
    let topic = topic(channel)?;
    let message_type = message_type(type_name)?;
    transport
        .subscribe_owned(topic, Some(message_type))
        .await
        .map_err(|error| WorkerError::Transport(error.to_string()))
}

async fn pump_ready(name: &'static str, ready: oneshot::Receiver<()>) -> Result<(), WorkerError> {
    ready.await.map_err(|_| {
        WorkerError::Transport(format!(
            "required input pump {name} exited before taking ownership"
        ))
    })
}

async fn pump_cloud(
    mut subscription: OwnedLcmSubscription,
    port: CloudInput,
    tx: mpsc::UnboundedSender<InputEvent>,
    ready: oneshot::Sender<()>,
) {
    let name = match port {
        CloudInput::Global => "global_map",
        CloudInput::Local => "local_map",
    };
    if ready.send(()).is_err() {
        return;
    }
    loop {
        let frame = match subscription.recv().await {
            Ok(frame) => frame,
            Err(error) => {
                let _ = tx.send(InputEvent::Failed(format!("{name}: {error}")));
                return;
            }
        };
        match decode_point_cloud_body(&frame.payload) {
            Ok(message) => {
                let event = match port {
                    CloudInput::Global => InputEvent::Global(message),
                    CloudInput::Local => InputEvent::Local(message),
                };
                if tx.send(event).is_err() {
                    return;
                }
            }
            Err(error) => {
                tracing::warn!(port = name, %error, "dropping malformed MLS point cloud");
            }
        }
    }
}

async fn pump_pose(
    mut subscription: OwnedLcmSubscription,
    port: PoseInput,
    tx: mpsc::UnboundedSender<InputEvent>,
    ready: oneshot::Sender<()>,
) {
    let name = match port {
        PoseInput::Bounds => "region_bounds",
        PoseInput::Start => "start_pose",
    };
    if ready.send(()).is_err() {
        return;
    }
    loop {
        let frame = match subscription.recv().await {
            Ok(frame) => frame,
            Err(error) => {
                let _ = tx.send(InputEvent::Failed(format!("{name}: {error}")));
                return;
            }
        };
        match decode_pose_stamped_body(&frame.payload) {
            Ok(message) => {
                let event = match port {
                    PoseInput::Bounds => InputEvent::Bounds(message),
                    PoseInput::Start => InputEvent::Start(message),
                };
                if tx.send(event).is_err() {
                    return;
                }
            }
            Err(error) => {
                tracing::warn!(port = name, %error, "dropping malformed MLS pose");
            }
        }
    }
}

async fn pump_goal(
    mut subscription: OwnedLcmSubscription,
    tx: mpsc::UnboundedSender<InputEvent>,
    ready: oneshot::Sender<()>,
) {
    if ready.send(()).is_err() {
        return;
    }
    loop {
        let frame = match subscription.recv().await {
            Ok(frame) => frame,
            Err(error) => {
                let _ = tx.send(InputEvent::Failed(format!("goal: {error}")));
                return;
            }
        };
        match schema::decode_goal_envelope_body(&frame.payload) {
            Ok(goal) => {
                if tx.send(InputEvent::Goal(Box::new(goal))).is_err() {
                    return;
                }
            }
            Err(error) => {
                tracing::warn!(%error, "dropping malformed MLS GoalEnvelope");
            }
        }
    }
}

async fn pump_cancel(
    mut subscription: OwnedLcmSubscription,
    tx: mpsc::UnboundedSender<InputEvent>,
    ready: oneshot::Sender<()>,
) {
    if ready.send(()).is_err() {
        return;
    }
    loop {
        let frame = match subscription.recv().await {
            Ok(frame) => frame,
            Err(error) => {
                let _ = tx.send(InputEvent::Failed(format!("cancel: {error}")));
                return;
            }
        };
        match schema::decode_cancel_navigation_body(&frame.payload) {
            Ok(cancel) => {
                if tx.send(InputEvent::Cancel(cancel)).is_err() {
                    return;
                }
            }
            Err(error) => {
                tracing::warn!(%error, "dropping malformed MLS cancellation");
            }
        }
    }
}

async fn handle_inputs(
    mut events: mpsc::UnboundedReceiver<InputEvent>,
    map_tx: watch::Sender<Option<Arc<MapUpdate>>>,
    start_tx: watch::Sender<Option<Xyz>>,
    goal_tx: watch::Sender<Option<GoalCommand>>,
    publisher: EventPublisher,
    goal_state: Arc<Mutex<GoalState>>,
    world_frame: String,
) -> Result<(), WorkerError> {
    let mut pending_local = None;
    let mut pending_bounds = None;

    while let Some(event) = events.recv().await {
        match event {
            InputEvent::Global(cloud) => {
                map_tx.send_replace(Some(Arc::new(MapUpdate::Global(cloud))));
            }
            InputEvent::Local(cloud) => {
                pending_local = Some(cloud);
                try_pair(&mut pending_local, &mut pending_bounds, &map_tx);
            }
            InputEvent::Bounds(bounds) => {
                pending_bounds = Some(bounds);
                try_pair(&mut pending_local, &mut pending_bounds, &map_tx);
            }
            InputEvent::Start(start) => {
                if start.header.frame_id != world_frame {
                    tracing::warn!(
                        start_frame = %start.header.frame_id,
                        %world_frame,
                        "dropping MLS start pose in wrong frame"
                    );
                    continue;
                }
                let Some(position) = checked_position(&start.pose) else {
                    tracing::warn!("dropping non-finite MLS start pose");
                    continue;
                };
                start_tx.send_replace(Some(position));
            }
            InputEvent::Goal(goal) => {
                let goal = *goal;
                let identity = GoalIdentity::from(&goal);
                let (attempt, terminal) = {
                    let mut state = goal_state.lock().await;
                    if let Some(watermark) = state.cancel_watermarks.get_mut(&identity) {
                        let terminal = if watermark.reported {
                            None
                        } else {
                            watermark.reported = true;
                            Some(WorkerEvent::Cancelled {
                                goal: goal.clone(),
                                cancel_generation: watermark.generation,
                            })
                        };
                        (None, terminal)
                    } else if state.active.is_some() {
                        (
                            None,
                            Some(WorkerEvent::Rejected {
                                goal: goal.clone(),
                                detail: "another MLS goal attempt is active".into(),
                            }),
                        )
                    } else {
                        state.next_token = state.next_token.wrapping_add(1).max(1);
                        let attempt = GoalAttempt {
                            token: state.next_token,
                            goal,
                        };
                        state.active = Some(attempt.clone());
                        (Some(attempt), None)
                    }
                };
                if let Some(terminal) = terminal {
                    publisher.publish(&terminal).await?;
                }
                if let Some(attempt) = attempt {
                    goal_tx.send_replace(Some(GoalCommand::Set(attempt)));
                }
            }
            InputEvent::Cancel(cancel) => {
                if let Some(attempt) = claim_cancel(&goal_state, &cancel).await {
                    publisher
                        .publish(&WorkerEvent::Cancelled {
                            goal: attempt.goal.clone(),
                            cancel_generation: cancel.cancel_generation,
                        })
                        .await?;
                    goal_tx.send_replace(Some(GoalCommand::Cancel {
                        attempt,
                        cancel_generation: cancel.cancel_generation,
                    }));
                }
            }
            InputEvent::Failed(error) => return Err(WorkerError::Transport(error)),
        }
    }
    Err(WorkerError::Transport("all input streams closed".into()))
}

fn try_pair(
    local: &mut Option<PointCloud2>,
    bounds: &mut Option<PoseStamped>,
    output: &watch::Sender<Option<Arc<MapUpdate>>>,
) {
    let paired = matches!(
        (bounds.as_ref(), local.as_ref()),
        (Some(bounds), Some(cloud)) if same_stamp(&bounds.header, &cloud.header)
    );
    if !paired {
        return;
    }
    let cloud = local.take().expect("pair checked");
    let bounds = bounds.take().expect("pair checked");
    output.send_replace(Some(Arc::new(MapUpdate::Region { cloud, bounds })));
}

struct PlanningInputs {
    map_rx: watch::Receiver<Option<Arc<MapUpdate>>>,
    start_rx: watch::Receiver<Option<Xyz>>,
    goal_rx: watch::Receiver<Option<GoalCommand>>,
    goal_state: Arc<Mutex<GoalState>>,
}

async fn plan_and_publish(
    publisher: EventPublisher,
    topics: WorkerTopics,
    config: Config,
    planning_timeout: Duration,
    inputs: PlanningInputs,
) -> Result<(), WorkerError> {
    let PlanningInputs {
        mut map_rx,
        start_rx,
        mut goal_rx,
        goal_state,
    } = inputs;
    let mut planner = Planner::default();
    let mut last_viz_at = None;
    let mut active_goal: Option<GoalAttempt> = None;
    let mut map_ready = false;

    loop {
        enum Wake {
            Map,
            Goal,
        }
        let wake = tokio::select! {
            changed = map_rx.changed() => {
                changed.map_err(|_| WorkerError::Transport("map handoff closed".into()))?;
                Wake::Map
            }
            changed = goal_rx.changed() => {
                changed.map_err(|_| WorkerError::Transport("goal handoff closed".into()))?;
                Wake::Goal
            }
        };

        if matches!(wake, Wake::Map) {
            if let Some(update) = map_rx.borrow_and_update().clone() {
                let updated =
                    match tokio::task::block_in_place(|| ingest(&mut planner, &update, &config)) {
                        Ok(updated) => updated,
                        Err(error) => {
                            tracing::warn!(%error, "dropping unusable MLS map update");
                            false
                        }
                    };
                map_ready |= updated;
                if updated && visualization_due(&config, last_viz_at) {
                    let stamp = now_header(&config.world_frame);
                    let (surface, nodes, edges) = build_graph_messages(&planner, &config, stamp);
                    publish_point_cloud(&publisher.transport, &topics.surface_map, surface).await?;
                    publish_point_cloud(&publisher.transport, &topics.nodes, nodes).await?;
                    publish_path(&publisher.transport, &topics.node_edges, edges).await?;
                    last_viz_at = Some(Instant::now());
                }
            }
        } else if let Some(command) = goal_rx.borrow_and_update().clone() {
            match command {
                GoalCommand::Set(attempt) => active_goal = Some(attempt),
                GoalCommand::Cancel {
                    attempt,
                    cancel_generation,
                } => {
                    tracing::info!(
                        session_id = %attempt.goal.session_id,
                        goal_generation = attempt.goal.goal_generation,
                        cancel_generation,
                        "MLS goal cancelled"
                    );
                    if active_goal
                        .as_ref()
                        .is_some_and(|active| active.token == attempt.token)
                    {
                        active_goal = None;
                    }
                }
            }
        }

        let Some(start) = *start_rx.borrow() else {
            continue;
        };
        let Some(attempt) = active_goal.clone() else {
            continue;
        };
        let goal = attempt.goal.clone();
        if !map_ready {
            if claim_terminal(&goal_state, &attempt).await {
                publisher
                    .publish(&WorkerEvent::Rejected {
                        goal: goal.clone(),
                        detail: "MLS map is not ready".into(),
                    })
                    .await?;
            }
            active_goal = None;
            continue;
        }
        if goal.goal.header.frame_id != config.world_frame {
            let detail = format!(
                "goal frame {} does not match MLS world frame {}",
                goal.goal.header.frame_id, config.world_frame
            );
            if claim_terminal(&goal_state, &attempt).await {
                publisher
                    .publish(&WorkerEvent::Rejected {
                        goal: goal.clone(),
                        detail,
                    })
                    .await?;
            }
            active_goal = None;
            continue;
        }

        let goal_xyz = pose_position(&goal.goal.pose);
        if is_at_goal(start, goal_xyz, config.goal_tolerance) {
            let path =
                path_from_waypoints(&[], &config.world_frame, now_header(&config.world_frame));
            if claim_terminal(&goal_state, &attempt).await {
                publisher
                    .publish(&WorkerEvent::Planned(PlannerResult {
                        goal: goal.clone(),
                        backend_generation: publisher.backend_generation,
                        path,
                    }))
                    .await?;
            }
            active_goal = None;
            continue;
        }

        let mut attempt_planner = planner.clone();
        let attempt_config = config.clone();
        let attempt_task = tokio::task::spawn_blocking(move || {
            let waypoints = attempt_planner.plan_or_truncate(start, goal_xyz, &attempt_config);
            (attempt_planner, waypoints)
        });
        let outcome = tokio::select! {
            changed = goal_rx.changed() => {
                changed.map_err(|_| WorkerError::Transport("goal handoff closed".into()))?;
                if let Some(command) = goal_rx.borrow_and_update().clone() {
                    match command {
                        GoalCommand::Set(new_attempt) => active_goal = Some(new_attempt),
                        GoalCommand::Cancel { attempt, .. }
                            if active_goal
                                .as_ref()
                                .is_some_and(|active| active.token == attempt.token) =>
                        {
                            active_goal = None;
                        }
                        GoalCommand::Cancel { .. } => {}
                    }
                }
                continue;
            }
            outcome = tokio::time::timeout(planning_timeout, attempt_task) => outcome,
        };

        match outcome {
            Err(_) => {
                if claim_terminal(&goal_state, &attempt).await {
                    publisher
                        .publish(&WorkerEvent::TimedOut(goal.clone()))
                        .await?;
                }
                active_goal = None;
            }
            Ok(Err(error)) => {
                if claim_terminal(&goal_state, &attempt).await {
                    publisher
                        .publish(&WorkerEvent::Crashed {
                            goal: goal.clone(),
                            detail: error.to_string(),
                        })
                        .await?;
                }
                active_goal = None;
            }
            Ok(Ok((updated_planner, waypoints))) => {
                planner = updated_planner;
                let path = path_from_waypoints(
                    &waypoints,
                    &config.world_frame,
                    now_header(&config.world_frame),
                );
                if claim_terminal(&goal_state, &attempt).await {
                    publisher
                        .publish(&WorkerEvent::Planned(PlannerResult {
                            goal: goal.clone(),
                            backend_generation: publisher.backend_generation,
                            path,
                        }))
                        .await?;
                }
                active_goal = None;
            }
        }
    }
}

async fn claim_cancel(
    goal_state: &Mutex<GoalState>,
    cancel: &CancelNavigation,
) -> Option<GoalAttempt> {
    let identity = GoalIdentity::from(cancel);
    let mut state = goal_state.lock().await;
    if state
        .cancel_watermarks
        .get(&identity)
        .is_some_and(|watermark| cancel.cancel_generation <= watermark.generation)
    {
        return None;
    }
    state.cancel_watermarks.insert(
        identity.clone(),
        CancelWatermark {
            generation: cancel.cancel_generation,
            reported: false,
        },
    );
    let matches = state
        .active
        .as_ref()
        .is_some_and(|attempt| GoalIdentity::from(&attempt.goal) == identity);
    if !matches {
        return None;
    }
    let attempt = state.active.take().expect("active attempt checked");
    state
        .cancel_watermarks
        .get_mut(&identity)
        .expect("cancel watermark inserted")
        .reported = true;
    Some(attempt)
}

async fn claim_terminal(goal_state: &Mutex<GoalState>, attempt: &GoalAttempt) -> bool {
    let mut state = goal_state.lock().await;
    let matches = state
        .active
        .as_ref()
        .is_some_and(|active| active.token == attempt.token);
    if matches {
        state.active = None;
    }
    matches
}

fn ingest(planner: &mut Planner, update: &MapUpdate, config: &Config) -> Result<bool, WorkerError> {
    match update {
        MapUpdate::Global(cloud) => {
            let points = extract_xyz(cloud)?;
            if points.is_empty() {
                return Ok(false);
            }
            planner.update_global_map(&points, config);
        }
        MapUpdate::Region { cloud, bounds } => {
            let points = extract_xyz(cloud)?;
            let values = [
                bounds.pose.position[0],
                bounds.pose.position[1],
                bounds.pose.orientation_xyzw[0],
                bounds.pose.orientation_xyzw[1],
                bounds.pose.orientation_xyzw[2],
            ];
            if values.iter().any(|value| !value.is_finite())
                || values[2] <= 0.0
                || values[3] > values[4]
            {
                return Err(WorkerError::Codec("invalid MLS region bounds".into()));
            }
            let bounds = RegionBounds {
                origin_x: bounds.pose.position[0] as f32,
                origin_y: bounds.pose.position[1] as f32,
                radius: bounds.pose.orientation_xyzw[0] as f32,
                z_min: bounds.pose.orientation_xyzw[1] as f32,
                z_max: bounds.pose.orientation_xyzw[2] as f32,
            };
            planner.update_region(&points, &bounds, config);
        }
    }
    Ok(true)
}

fn visualization_due(config: &Config, last: Option<Instant>) -> bool {
    if config.viz_publish_hz <= 0.0 {
        return false;
    }
    let interval = Duration::from_secs_f32(1.0 / config.viz_publish_hz);
    last.is_none_or(|last| last.elapsed() >= interval)
}

fn build_graph_messages(
    planner: &Planner,
    config: &Config,
    header: Header,
) -> (PointCloud2, PointCloud2, Path) {
    let surface_points: Vec<Xyz> = planner
        .surface()
        .map(|(x, y, z)| surface_point_xyz(x, y, z, config.voxel_size))
        .collect();
    let node_points: Vec<Xyz> = planner.graph().nodes.iter().map(|node| node.pos).collect();
    let surface = point_cloud_from_xyz(&surface_points, header.clone());
    let nodes = point_cloud_from_xyz(&node_points, header.clone());
    let edges = graph_edges_path(planner.graph(), config.voxel_size, header);
    (surface, nodes, edges)
}

fn graph_edges_path(graph: &PlannerGraph, voxel_size: f32, header: Header) -> Path {
    let segments = edges_to_segments(&graph.cells, &graph.cell_state, &graph.node_edges);
    let mut poses = Vec::with_capacity(segments.len() * 2);
    for (a, b, cost) in segments {
        let a = surface_point_xyz(a.0, a.1, a.2, voxel_size);
        let b = surface_point_xyz(b.0, b.1, b.2, voxel_size);
        poses.push(stamped_pose(a, cost as f64, header.clone()));
        poses.push(stamped_pose(b, cost as f64, header.clone()));
    }
    Path { header, poses }
}

fn point_cloud_from_xyz(points: &[Xyz], header: Header) -> PointCloud2 {
    let mut data = Vec::with_capacity(points.len() * 12);
    for &(x, y, z) in points {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
    }
    let width = i32::try_from(points.len()).unwrap_or(i32::MAX);
    PointCloud2 {
        header,
        height: 1,
        width,
        fields: vec![
            PointField {
                name: "x".into(),
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "y".into(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "z".into(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step: 12,
        row_step: width.saturating_mul(12),
        data,
        is_dense: true,
    }
}

fn path_from_waypoints(waypoints: &[Xyz], frame_id: &str, header: Header) -> Path {
    let poses = waypoints
        .iter()
        .copied()
        .map(|point| stamped_pose(point, 1.0, header.clone()))
        .collect();
    let mut path = Path { header, poses };
    path.header.frame_id = frame_id.to_string();
    path
}

fn stamped_pose(point: Xyz, orientation_w: f64, header: Header) -> PoseStamped {
    PoseStamped {
        header,
        pose: Pose {
            position: [point.0 as f64, point.1 as f64, point.2 as f64],
            orientation_xyzw: [0.0, 0.0, 0.0, orientation_w],
        },
    }
}

fn now_header(frame_id: &str) -> Header {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    Header {
        seq: 0,
        stamp_sec: duration.as_secs().min(i32::MAX as u64) as i32,
        stamp_nsec: duration.subsec_nanos() as i32,
        frame_id: frame_id.to_string(),
    }
}

fn checked_position(pose: &Pose) -> Option<Xyz> {
    let position = pose_position(pose);
    (position.0.is_finite() && position.1.is_finite() && position.2.is_finite()).then_some(position)
}

fn pose_position(pose: &Pose) -> Xyz {
    (
        pose.position[0] as f32,
        pose.position[1] as f32,
        pose.position[2] as f32,
    )
}

fn is_at_goal(start: Xyz, goal: Xyz, tolerance: f32) -> bool {
    (start.0 - goal.0).hypot(start.1 - goal.1) < tolerance
}

fn same_stamp(a: &Header, b: &Header) -> bool {
    a.stamp_sec == b.stamp_sec && a.stamp_nsec == b.stamp_nsec
}

fn extract_xyz(message: &PointCloud2) -> Result<Vec<Xyz>, WorkerError> {
    if message.width < 0 || message.height < 0 || message.point_step <= 0 {
        return Err(WorkerError::Codec("invalid PointCloud2 dimensions".into()));
    }
    if message.data.len() > MAX_POINT_CLOUD_DATA_BYTES {
        return Err(WorkerError::Codec("PointCloud2 payload exceeds cap".into()));
    }
    if message.is_bigendian {
        return Err(WorkerError::Codec(
            "big-endian PointCloud2 point data is not supported".into(),
        ));
    }
    let field_offset = |name: &str| {
        message
            .fields
            .iter()
            .find(|field| field.name == name && field.datatype == 7 && field.offset >= 0)
            .map(|field| field.offset as usize)
            .ok_or_else(|| WorkerError::Codec(format!("missing float32 {name} field")))
    };
    let x = field_offset("x")?;
    let y = field_offset("y")?;
    let z = field_offset("z")?;
    let step = message.point_step as usize;
    if step > MAX_POINT_STEP_BYTES {
        return Err(WorkerError::Codec(
            "PointCloud2 point_step exceeds cap".into(),
        ));
    }
    if x + 4 > step || y + 4 > step || z + 4 > step {
        return Err(WorkerError::Codec(
            "PointCloud2 xyz offsets exceed point_step".into(),
        ));
    }
    let count = (message.width as usize)
        .checked_mul(message.height as usize)
        .ok_or_else(|| WorkerError::Codec("PointCloud2 point count overflow".into()))?;
    if count > MAX_POINT_CLOUD_POINTS {
        return Err(WorkerError::Codec(
            "PointCloud2 point count exceeds cap".into(),
        ));
    }
    let required = count
        .checked_mul(step)
        .ok_or_else(|| WorkerError::Codec("PointCloud2 byte length overflow".into()))?;
    if message.data.len() < required {
        return Err(WorkerError::Codec(format!(
            "PointCloud2 data has {} bytes, expected at least {required}",
            message.data.len()
        )));
    }

    let mut points = Vec::with_capacity(count);
    for index in 0..count {
        let base = index * step;
        let point = (
            read_f32(&message.data, base + x),
            read_f32(&message.data, base + y),
            read_f32(&message.data, base + z),
        );
        if point.0.is_finite() && point.1.is_finite() && point.2.is_finite() {
            points.push(point);
        }
    }
    Ok(points)
}

fn read_f32(data: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(data[offset..offset + 4].try_into().expect("bounds checked"))
}

#[cfg(test)]
fn cancel_matches_goal(cancel: &CancelNavigation, goal: &GoalEnvelope) -> bool {
    cancel.session_id == goal.session_id
        && cancel.owner_kind == goal.owner_kind
        && cancel.parent_request_id == goal.parent_request_id
        && cancel.goal_generation == goal.goal_generation
        && cancel.producer_runtime_generation == goal.producer_runtime_generation
}

fn decode_pose_stamped_body(body: &[u8]) -> Result<PoseStamped, WorkerError> {
    let mut encoded = schema::POSE_STAMPED_FINGERPRINT.to_vec();
    encoded.extend_from_slice(body);
    schema::decode_pose_stamped(&encoded).map_err(|error| WorkerError::Codec(error.to_string()))
}

fn decode_point_cloud_body(body: &[u8]) -> Result<PointCloud2, WorkerError> {
    let mut encoded = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
    encoded.extend_from_slice(body);
    schema::decode_point_cloud2(&encoded).map_err(|error| WorkerError::Codec(error.to_string()))
}

fn encode_ready_contract(contract: &ReadyContract) -> Result<Vec<u8>, WorkerError> {
    let channels = [
        &contract.global_map,
        &contract.local_map,
        &contract.region_bounds,
        &contract.start_pose,
        &contract.goal,
        &contract.cancel,
        &contract.events,
    ];
    let mut body = Vec::new();
    for channel in channels {
        topic(channel)?;
        let len = u16::try_from(channel.len())
            .map_err(|_| WorkerError::Codec("READY channel is too long".into()))?;
        body.extend_from_slice(&len.to_be_bytes());
        body.extend_from_slice(channel.as_bytes());
    }
    Ok(body)
}

fn decode_ready_contract(body: &[u8]) -> Result<ReadyContract, WorkerError> {
    let mut cursor = EventCursor::new(body);
    let contract = ReadyContract {
        global_map: cursor.string_u16()?,
        local_map: cursor.string_u16()?,
        region_bounds: cursor.string_u16()?,
        start_pose: cursor.string_u16()?,
        goal: cursor.string_u16()?,
        cancel: cursor.string_u16()?,
        events: cursor.string_u16()?,
    };
    if !cursor.is_empty() {
        return Err(WorkerError::Codec(
            "READY contract contains trailing bytes".into(),
        ));
    }
    for channel in [
        &contract.global_map,
        &contract.local_map,
        &contract.region_bounds,
        &contract.start_pose,
        &contract.goal,
        &contract.cancel,
        &contract.events,
    ] {
        topic(channel)?;
    }
    Ok(contract)
}

/// Encode a backend-private worker event inside a registered typed state body.
pub fn encode_worker_event_body(
    backend_id: &str,
    backend_generation: i64,
    event: &WorkerEvent,
) -> Result<Vec<u8>, WorkerError> {
    if backend_id.trim().is_empty() || backend_generation <= 0 {
        return Err(WorkerError::Codec(
            "worker event requires backend identity and positive generation".into(),
        ));
    }

    let (kind, ready, goal, path, cancel_generation, detail) = match event {
        WorkerEvent::Ready(contract) => (WORKER_KIND_READY, Some(contract), None, None, 0, ""),
        WorkerEvent::Planned(result) => {
            if result.backend_generation != backend_generation {
                return Err(WorkerError::Codec(
                    "planner result backend generation mismatch".into(),
                ));
            }
            (
                WORKER_KIND_PLANNED,
                None,
                Some(&result.goal),
                Some(&result.path),
                0,
                "",
            )
        }
        WorkerEvent::Cancelled {
            goal,
            cancel_generation,
        } => (
            WORKER_KIND_CANCELLED,
            None,
            Some(goal),
            None,
            *cancel_generation,
            "",
        ),
        WorkerEvent::TimedOut(goal) => (WORKER_KIND_TIMEOUT, None, Some(goal), None, 0, "timeout"),
        WorkerEvent::Crashed { goal, detail } => (
            WORKER_KIND_CRASHED,
            None,
            Some(goal),
            None,
            0,
            detail.as_str(),
        ),
        WorkerEvent::Rejected { goal, detail } => (
            WORKER_KIND_REJECTED,
            None,
            Some(goal),
            None,
            0,
            detail.as_str(),
        ),
    };
    if cancel_generation < 0 || detail.len() > u16::MAX as usize {
        return Err(WorkerError::Codec(
            "invalid worker terminal metadata".into(),
        ));
    }

    let ready_body = ready
        .map(encode_ready_contract)
        .transpose()?
        .unwrap_or_default();
    let goal_body = goal
        .map(schema::encode_goal_envelope_body)
        .transpose()
        .map_err(|error| WorkerError::Codec(error.to_string()))?
        .unwrap_or_default();
    let path_body = path
        .map(schema::encode_path_body)
        .transpose()
        .map_err(|error| WorkerError::Codec(error.to_string()))?
        .unwrap_or_default();
    let backend_len = u16::try_from(backend_id.len())
        .map_err(|_| WorkerError::Codec("backend identity too long".into()))?;
    let ready_len = u32::try_from(ready_body.len())
        .map_err(|_| WorkerError::Codec("READY contract too large".into()))?;
    let goal_len = u32::try_from(goal_body.len())
        .map_err(|_| WorkerError::Codec("goal body too large".into()))?;
    let path_len = u32::try_from(path_body.len())
        .map_err(|_| WorkerError::Codec("path body too large".into()))?;
    let detail_len = detail.len() as u16;

    let mut private = Vec::new();
    private.extend_from_slice(WORKER_PROTOCOL_MAGIC);
    private.push(WORKER_PROTOCOL_VERSION);
    private.push(kind);
    private.extend_from_slice(&backend_len.to_be_bytes());
    private.extend_from_slice(backend_id.as_bytes());
    private.extend_from_slice(&backend_generation.to_be_bytes());
    private.extend_from_slice(&cancel_generation.to_be_bytes());
    private.extend_from_slice(&ready_len.to_be_bytes());
    private.extend_from_slice(&ready_body);
    private.extend_from_slice(&goal_len.to_be_bytes());
    private.extend_from_slice(&goal_body);
    private.extend_from_slice(&path_len.to_be_bytes());
    private.extend_from_slice(&path_body);
    private.extend_from_slice(&detail_len.to_be_bytes());
    private.extend_from_slice(detail.as_bytes());
    if private.len() > MAX_PRIVATE_EVENT_BYTES {
        return Err(WorkerError::Codec("worker event exceeds size limit".into()));
    }

    let mut state = NavigationState {
        session_id: goal
            .map(|goal| goal.session_id.clone())
            .unwrap_or_else(|| backend_id.to_string()),
        robot_frame: WORKER_FRAME.into(),
        state: kind as i32,
        message: hex_encode(&private),
        runtime_generation: backend_generation,
        sink_ready: i8::from(matches!(event, WorkerEvent::Ready(_))),
        zero_ack_id: backend_id.to_string(),
        cancel_generation,
        ..NavigationState::default()
    };
    if let Some(goal) = goal {
        state.goal = [
            goal.goal.pose.position[0],
            goal.goal.pose.position[1],
            goal.goal.pose.orientation_xyzw[2],
        ];
        state.guard_generation = goal.motion_guard_generation;
        state.planner_map_frame = goal.committed_map_frame_id.clone();
        state.planner_pose_frame = goal.goal.header.frame_id.clone();
        state.owner_kind = goal.owner_kind;
        state.parent_request_id = goal.parent_request_id.clone();
        state.goal_generation = goal.goal_generation;
        state.goal_payload_sha256 = goal.payload_sha256;
    }
    schema::encode_navigation_state_body(&state)
        .map_err(|error| WorkerError::Codec(error.to_string()))
}

/// Decode and strictly validate a backend-private worker event body.
pub fn decode_worker_event_body(body: &[u8]) -> Result<(String, i64, WorkerEvent), WorkerError> {
    let state = schema::decode_navigation_state_body(body)
        .map_err(|error| WorkerError::Codec(error.to_string()))?;
    if state.robot_frame != WORKER_FRAME || state.runtime_generation <= 0 {
        return Err(WorkerError::Codec(
            "invalid MLS worker event identity".into(),
        ));
    }
    let private = hex_decode(&state.message)?;
    if private.len() > MAX_PRIVATE_EVENT_BYTES {
        return Err(WorkerError::Codec("worker event exceeds size limit".into()));
    }
    let mut cursor = EventCursor::new(&private);
    if cursor.take(4)? != WORKER_PROTOCOL_MAGIC {
        return Err(WorkerError::Codec("invalid MLS worker event magic".into()));
    }
    if cursor.u8()? != WORKER_PROTOCOL_VERSION {
        return Err(WorkerError::Codec(
            "unsupported MLS worker event version".into(),
        ));
    }
    let kind = cursor.u8()?;
    let backend_id = cursor.string_u16()?;
    let backend_generation = cursor.i64()?;
    let cancel_generation = cursor.i64()?;
    let ready_body = cursor.bytes_u32()?;
    let goal_body = cursor.bytes_u32()?;
    let path_body = cursor.bytes_u32()?;
    let detail = cursor.string_u16()?;
    if !cursor.is_empty()
        || backend_id.trim().is_empty()
        || backend_generation != state.runtime_generation
        || backend_id != state.zero_ack_id
        || i32::from(kind) != state.state
        || cancel_generation != state.cancel_generation
        || cancel_generation < 0
        || (kind == WORKER_KIND_READY && state.sink_ready != 1)
        || (kind != WORKER_KIND_READY && state.sink_ready != 0)
    {
        return Err(WorkerError::Codec("inconsistent MLS worker event".into()));
    }

    let ready = if ready_body.is_empty() {
        None
    } else {
        Some(decode_ready_contract(ready_body)?)
    };
    let goal = if goal_body.is_empty() {
        None
    } else {
        Some(
            schema::decode_goal_envelope_body(goal_body)
                .map_err(|error| WorkerError::Codec(error.to_string()))?,
        )
    };
    let path = if path_body.is_empty() {
        None
    } else {
        let mut encoded = schema::PATH_FINGERPRINT.to_vec();
        encoded.extend_from_slice(path_body);
        Some(schema::decode_path(&encoded).map_err(|error| WorkerError::Codec(error.to_string()))?)
    };
    if let Some(goal) = goal.as_ref() {
        if state.session_id != goal.session_id
            || state.owner_kind != goal.owner_kind
            || state.parent_request_id != goal.parent_request_id
            || state.goal_generation != goal.goal_generation
            || state.goal_payload_sha256 != goal.payload_sha256
            || state.guard_generation != goal.motion_guard_generation
            || state.planner_map_frame != goal.committed_map_frame_id
            || state.planner_pose_frame != goal.goal.header.frame_id
            || state.goal[0] != goal.goal.pose.position[0]
            || state.goal[1] != goal.goal.pose.position[1]
            || state.goal[2] != goal.goal.pose.orientation_xyzw[2]
        {
            return Err(WorkerError::Codec(
                "worker event outer identity does not match GoalEnvelope".into(),
            ));
        }
    }

    if kind == WORKER_KIND_READY
        && (state.session_id != backend_id
            || state.owner_kind != 0
            || !state.parent_request_id.is_empty()
            || state.goal_generation != 0
            || state.goal_payload_sha256 != [0; 32]
            || state.guard_generation != 0
            || !state.planner_map_frame.is_empty()
            || !state.planner_pose_frame.is_empty()
            || state.goal != [0.0; 3]
            || cancel_generation != 0
            || ready.is_none()
            || goal.is_some()
            || path.is_some()
            || !detail.is_empty())
    {
        return Err(WorkerError::Codec("invalid MLS READY event shape".into()));
    }

    let event = match (kind, ready, goal, path) {
        (WORKER_KIND_READY, Some(contract), None, None) => WorkerEvent::Ready(contract),
        (WORKER_KIND_PLANNED, None, Some(goal), Some(path)) if detail.is_empty() => {
            WorkerEvent::Planned(PlannerResult {
                goal,
                backend_generation,
                path,
            })
        }
        (WORKER_KIND_CANCELLED, None, Some(goal), None)
            if cancel_generation > 0 && detail.is_empty() =>
        {
            WorkerEvent::Cancelled {
                goal,
                cancel_generation,
            }
        }
        (WORKER_KIND_TIMEOUT, None, Some(goal), None) if detail == "timeout" => {
            WorkerEvent::TimedOut(goal)
        }
        (WORKER_KIND_CRASHED, None, Some(goal), None) if !detail.is_empty() => {
            WorkerEvent::Crashed { goal, detail }
        }
        (WORKER_KIND_REJECTED, None, Some(goal), None) if !detail.is_empty() => {
            WorkerEvent::Rejected { goal, detail }
        }
        _ => return Err(WorkerError::Codec("invalid MLS worker event shape".into())),
    };
    Ok((backend_id, backend_generation, event))
}

struct EventCursor<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> EventCursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take(&mut self, len: usize) -> Result<&'a [u8], WorkerError> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or_else(|| WorkerError::Codec("worker event length overflow".into()))?;
        let value = self
            .bytes
            .get(self.offset..end)
            .ok_or_else(|| WorkerError::Codec("truncated MLS worker event".into()))?;
        self.offset = end;
        Ok(value)
    }

    fn u8(&mut self) -> Result<u8, WorkerError> {
        Ok(self.take(1)?[0])
    }

    fn i64(&mut self) -> Result<i64, WorkerError> {
        Ok(i64::from_be_bytes(
            self.take(8)?.try_into().expect("eight bytes"),
        ))
    }

    fn string_u16(&mut self) -> Result<String, WorkerError> {
        let len = u16::from_be_bytes(self.take(2)?.try_into().expect("two bytes")) as usize;
        String::from_utf8(self.take(len)?.to_vec())
            .map_err(|_| WorkerError::Codec("worker event contains invalid UTF-8".into()))
    }

    fn bytes_u32(&mut self) -> Result<&'a [u8], WorkerError> {
        let len = u32::from_be_bytes(self.take(4)?.try_into().expect("four bytes")) as usize;
        if len > MAX_PRIVATE_EVENT_BYTES {
            return Err(WorkerError::Codec(
                "worker event field exceeds size limit".into(),
            ));
        }
        self.take(len)
    }

    fn is_empty(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        encoded.push(HEX[(byte >> 4) as usize] as char);
        encoded.push(HEX[(byte & 0x0f) as usize] as char);
    }
    encoded
}

fn hex_decode(value: &str) -> Result<Vec<u8>, WorkerError> {
    if value.len() % 2 != 0 || value.len() / 2 > MAX_PRIVATE_EVENT_BYTES {
        return Err(WorkerError::Codec("invalid worker event hex length".into()));
    }
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let high = hex_nibble(pair[0])?;
            let low = hex_nibble(pair[1])?;
            Ok((high << 4) | low)
        })
        .collect()
}

fn hex_nibble(value: u8) -> Result<u8, WorkerError> {
    match value {
        b'0'..=b'9' => Ok(value - b'0'),
        b'a'..=b'f' => Ok(value - b'a' + 10),
        _ => Err(WorkerError::Codec("invalid worker event hex digit".into())),
    }
}

async fn publish_path(
    transport: &LcmTransport,
    channel: &str,
    message: Path,
) -> Result<(), WorkerError> {
    let body = schema::encode_path_body(&message)
        .map_err(|error| WorkerError::Codec(error.to_string()))?;
    publish(transport, channel, "nav_msgs.Path", body).await
}

async fn publish_point_cloud(
    transport: &LcmTransport,
    channel: &str,
    message: PointCloud2,
) -> Result<(), WorkerError> {
    let body = schema::encode_point_cloud2_body(&message)
        .map_err(|error| WorkerError::Codec(error.to_string()))?;
    publish(transport, channel, "sensor_msgs.PointCloud2", body).await
}

async fn publish(
    transport: &LcmTransport,
    channel: &str,
    message_type_name: &str,
    body: Vec<u8>,
) -> Result<(), WorkerError> {
    let frame = RawLcmFrameBuilder::new(topic(channel)?, message_type(message_type_name)?)
        .payload(body)
        .build();
    transport
        .publish(frame)
        .await
        .map_err(|error| WorkerError::Transport(error.to_string()))
}

fn topic(value: &str) -> Result<LcmTopic, WorkerError> {
    let normalized = if value.starts_with('/') {
        value.to_string()
    } else {
        format!("/{value}")
    };
    LcmTopic::new(&normalized).map_err(|error| WorkerError::Contract(error.to_string()))
}

fn message_type(value: &str) -> Result<LcmMessageType, WorkerError> {
    let message =
        LcmMessageType::new(value).map_err(|error| WorkerError::Contract(error.to_string()))?;
    if message.fingerprint() == [0; 8] {
        return Err(WorkerError::Contract(format!(
            "unregistered message type {}",
            message.fqn()
        )));
    }
    Ok(message)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(sec: i32, nsec: i32) -> Header {
        Header {
            stamp_sec: sec,
            stamp_nsec: nsec,
            ..Header::default()
        }
    }

    fn cloud(sec: i32, nsec: i32) -> PointCloud2 {
        PointCloud2 {
            header: header(sec, nsec),
            height: 1,
            width: 1,
            fields: vec![
                PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                PointField {
                    name: "y".into(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                PointField {
                    name: "z".into(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
            ],
            point_step: 12,
            row_step: 12,
            data: [1.0_f32, 2.0, 3.0]
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect(),
            is_dense: true,
            ..PointCloud2::default()
        }
    }

    fn goal(session_id: &str, generation: i64) -> GoalEnvelope {
        let mut goal = GoalEnvelope {
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: session_id.into(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            parent_request_id: "explore-parent".into(),
            goal_generation: generation,
            producer: "wavefront".into(),
            producer_epoch: 13,
            producer_runtime_generation: 17,
            committed_map_session_id: "map-session".into(),
            committed_map_session_generation: 3,
            committed_map_revision: 5,
            committed_map_payload_sha256: [0x31; 32],
            committed_map_frame_id: "map".into(),
            source_timestamp_ns: generation * 1_000_000_000,
            boundary_id: "explore-parent:boundary".into(),
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
            goal: PoseStamped {
                header: Header {
                    seq: generation as i32,
                    stamp_sec: generation as i32,
                    frame_id: "map".into(),
                    ..Header::default()
                },
                pose: Pose {
                    position: [4.5, 4.5, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
            ..GoalEnvelope::default()
        };
        goal.boundary_sha256 = schema::goal_intent_boundary_sha256(
            &goal.boundary_frame_id,
            &goal.boundary_id,
            &goal.boundary_points,
        )
        .unwrap();
        schema::seal_goal_envelope(&mut goal).unwrap();
        goal
    }

    #[test]
    fn local_map_pairs_only_with_exact_bounds_stamp() {
        let (tx, rx) = watch::channel(None);
        let mut local = Some(cloud(1, 2));
        let mut bounds = Some(PoseStamped {
            header: header(1, 3),
            ..PoseStamped::default()
        });
        try_pair(&mut local, &mut bounds, &tx);
        assert!(rx.borrow().is_none());
        assert!(local.is_some());

        bounds.as_mut().unwrap().header.stamp_nsec = 2;
        try_pair(&mut local, &mut bounds, &tx);
        assert!(matches!(
            rx.borrow().as_deref(),
            Some(MapUpdate::Region { .. })
        ));
        assert!(local.is_none());
        assert!(bounds.is_none());
    }

    #[test]
    fn cancel_requires_exact_goal_identity() {
        let goal = goal("session", 7);
        let cancel = CancelNavigation {
            session_id: goal.session_id.clone(),
            cancel_generation: 1,
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
            ..CancelNavigation::default()
        };
        assert!(cancel_matches_goal(&cancel, &goal));
        assert!(!cancel_matches_goal(
            &CancelNavigation {
                goal_generation: 8,
                ..cancel
            },
            &goal
        ));
    }

    #[test]
    fn worker_result_round_trip_preserves_full_goal_identity() {
        let goal = goal("session", 7);
        let path = path_from_waypoints(&[(1.0, 2.0, 0.0)], "map", header(1, 2));
        let event = WorkerEvent::Planned(PlannerResult {
            goal: goal.clone(),
            backend_generation: 11,
            path: path.clone(),
        });
        let body = encode_worker_event_body("mls3d", 11, &event).unwrap();
        let (backend, generation, decoded) = decode_worker_event_body(&body).unwrap();
        assert_eq!(backend, "mls3d");
        assert_eq!(generation, 11);
        assert_eq!(decoded, event);
        let WorkerEvent::Planned(result) = decoded else {
            panic!("expected planned event");
        };
        assert_eq!(result.goal, goal);
        assert_eq!(result.path, path);
    }

    #[test]
    fn terminal_events_round_trip_with_goal_identity() {
        let goal = goal("terminal-session", 9);
        let events = [
            WorkerEvent::Cancelled {
                goal: goal.clone(),
                cancel_generation: 4,
            },
            WorkerEvent::TimedOut(goal.clone()),
            WorkerEvent::Crashed {
                goal: goal.clone(),
                detail: "planner task panicked".into(),
            },
            WorkerEvent::Rejected {
                goal,
                detail: "map unavailable".into(),
            },
        ];
        for event in events {
            let body = encode_worker_event_body("mls3d", 11, &event).unwrap();
            let (_, _, decoded) = decode_worker_event_body(&body).unwrap();
            assert_eq!(decoded, event);
        }
    }

    #[test]
    fn hostile_worker_event_bytes_fail_closed() {
        let body = encode_worker_event_body(
            "mls3d",
            11,
            &WorkerEvent::Ready(ReadyContract::from(&WorkerTopics::default())),
        )
        .unwrap();
        let mut state = schema::decode_navigation_state_body(&body).unwrap();
        state.message = "0".into();
        let hostile = schema::encode_navigation_state_body(&state).unwrap();
        assert!(decode_worker_event_body(&hostile).is_err());

        let body = encode_worker_event_body(
            "mls3d",
            11,
            &WorkerEvent::Ready(ReadyContract::from(&WorkerTopics::default())),
        )
        .unwrap();
        let mut state = schema::decode_navigation_state_body(&body).unwrap();
        state.runtime_generation = 12;
        let wrong_generation = schema::encode_navigation_state_body(&state).unwrap();
        assert!(decode_worker_event_body(&wrong_generation).is_err());

        let mut truncated = body;
        truncated.truncate(truncated.len() - 1);
        assert!(decode_worker_event_body(&truncated).is_err());
    }

    #[test]
    fn point_cloud_extraction_uses_named_offsets_and_filters_nan() {
        let mut message = cloud(1, 2);
        message.width = 2;
        message.row_step = 24;
        message.data.extend_from_slice(&f32::NAN.to_le_bytes());
        message.data.extend_from_slice(&5.0_f32.to_le_bytes());
        message.data.extend_from_slice(&6.0_f32.to_le_bytes());
        assert_eq!(extract_xyz(&message).unwrap(), vec![(1.0, 2.0, 3.0)]);
    }

    #[test]
    fn empty_waypoints_encode_as_an_explicit_stop_path() {
        let message = path_from_waypoints(&[], "map", header(1, 2));
        assert_eq!(message.header.frame_id, "map");
        assert!(message.poses.is_empty());
        let encoded = schema::encode_path(&message).unwrap();
        assert_eq!(schema::decode_path(&encoded).unwrap(), message);
    }

    #[test]
    fn ready_round_trip_binds_all_required_channels() {
        let event = WorkerEvent::Ready(ReadyContract::from(&WorkerTopics::default()));
        let body = encode_worker_event_body("mls3d", 11, &event).unwrap();
        let (backend, generation, decoded) = decode_worker_event_body(&body).unwrap();
        assert_eq!(backend, "mls3d");
        assert_eq!(generation, 11);
        assert_eq!(decoded, event);
    }

    #[tokio::test]
    async fn cancel_before_goal_retains_exact_monotonic_watermark() {
        let goal = goal("cancel-first", 7);
        let cancel = CancelNavigation {
            session_id: goal.session_id.clone(),
            cancel_generation: 4,
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer_runtime_generation: goal.producer_runtime_generation,
            ..CancelNavigation::default()
        };
        let state = Mutex::new(GoalState::default());
        assert!(claim_cancel(&state, &cancel).await.is_none());
        assert!(claim_cancel(
            &state,
            &CancelNavigation {
                cancel_generation: 3,
                ..cancel.clone()
            }
        )
        .await
        .is_none());
        let state = state.lock().await;
        let watermark = state
            .cancel_watermarks
            .get(&GoalIdentity::from(&goal))
            .unwrap();
        assert_eq!(watermark.generation, 4);
        assert!(!watermark.reported);
    }

    #[tokio::test]
    async fn cancel_and_completion_atomically_claim_one_terminal() {
        let goal = goal("terminal-race", 8);
        let attempt = GoalAttempt { token: 1, goal };
        let state = Arc::new(Mutex::new(GoalState {
            next_token: 1,
            active: Some(attempt.clone()),
            cancel_watermarks: HashMap::new(),
        }));
        let cancel = CancelNavigation {
            session_id: attempt.goal.session_id.clone(),
            cancel_generation: 5,
            owner_kind: attempt.goal.owner_kind,
            parent_request_id: attempt.goal.parent_request_id.clone(),
            goal_generation: attempt.goal.goal_generation,
            producer_runtime_generation: attempt.goal.producer_runtime_generation,
            ..CancelNavigation::default()
        };
        let barrier = Arc::new(tokio::sync::Barrier::new(3));
        let cancel_task = {
            let state = state.clone();
            let barrier = barrier.clone();
            tokio::spawn(async move {
                barrier.wait().await;
                claim_cancel(&state, &cancel).await.is_some()
            })
        };
        let completion_task = {
            let state = state.clone();
            let barrier = barrier.clone();
            tokio::spawn(async move {
                barrier.wait().await;
                claim_terminal(&state, &attempt).await
            })
        };
        barrier.wait().await;
        let cancel_won = cancel_task.await.unwrap();
        let completion_won = completion_task.await.unwrap();
        assert_ne!(cancel_won, completion_won);
        assert!(state.lock().await.active.is_none());
    }

    #[tokio::test]
    async fn watch_map_handoff_retains_only_the_latest_update() {
        let (tx, mut rx) = watch::channel::<Option<Arc<MapUpdate>>>(None);
        tx.send_replace(Some(Arc::new(MapUpdate::Global(cloud(1, 0)))));
        tx.send_replace(Some(Arc::new(MapUpdate::Global(cloud(2, 0)))));
        rx.changed().await.unwrap();
        let current = rx.borrow_and_update().clone().unwrap();
        let MapUpdate::Global(current) = current.as_ref() else {
            panic!("expected global map");
        };
        assert_eq!(current.header.stamp_sec, 2);
    }
}
