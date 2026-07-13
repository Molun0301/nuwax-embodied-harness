//! Real-LCM adapters for in-process navigation graph nodes.

use std::future::Future;
use std::pin::Pin;
use std::sync::{mpsc as std_mpsc, Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use futures::{future::pending, StreamExt};
use nuwax_codec::lcm::schema;
#[cfg(feature = "patrolling")]
use nuwax_nav_types_messages::GridSnapshot;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "mls",
    feature = "explore"
))]
use nuwax_nav_types_messages::Point2;
#[cfg(any(feature = "simple-2d", feature = "cmu-native", feature = "explore"))]
use nuwax_nav_types_messages::PointCloud;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "mls",
    feature = "patrolling",
    feature = "click-goal",
    feature = "explore"
))]
use nuwax_nav_types_messages::PoseStamped;
use nuwax_nav_types_messages::{
    NavFrameId, Point3, PointStamped, Pose3, Quaternion, Timestamp, Twist3,
};
#[cfg(feature = "explore")]
use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
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

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
pub(crate) fn navigation_state_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 5, &[3])?;
    let node = node.clone();
    Ok(boxed_owner(
        "navigation-state-reporter",
        Box::new(move |shutdown, ready| {
            Box::pin(run_navigation_state_reporter(node, shutdown, ready))
        }),
    ))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
async fn run_navigation_state_reporter(
    node: ModuleNode,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    let mut goal = typed_stream(&transport, &node.inputs[0], "geometry_msgs.PoseStamped").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut path = typed_stream(&transport, &node.inputs[2], "nav_msgs.Path").await?;
    let mut reached = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut cancel =
        typed_stream(&transport, &node.inputs[4], "nuwax_msgs.CancelNavigation").await?;
    let mut reporter = NavigationStateReporter::default();
    publish_navigation_state(&transport, &node.outputs[0], &reporter).await?;
    signal_ready(ready)?;

    let mut heartbeat = tokio::time::interval(Duration::from_millis(250));
    heartbeat.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            _ = heartbeat.tick() => {
                publish_navigation_state(&transport, &node.outputs[0], &reporter).await?;
            }
            message = goal.next() => {
                let frame = required_frame(message, "navigation_state.goal")?;
                reporter.on_goal(&decode_pose_body(&frame.payload)?);
                publish_navigation_state(&transport, &node.outputs[0], &reporter).await?;
            }
            message = odometry.next() => {
                let frame = required_frame(message, "navigation_state.odometry")?;
                reporter.on_odometry(&decode_odometry_pose_body(&frame.payload)?);
            }
            message = path.next() => {
                let frame = required_frame(message, "navigation_state.path")?;
                reporter.on_path(&decode_path_body(&frame.payload)?);
                publish_navigation_state(&transport, &node.outputs[0], &reporter).await?;
            }
            message = reached.next() => {
                let frame = required_frame(message, "navigation_state.goal_reached")?;
                let reached = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("NavigationState reached decode: {error}")))?;
                if reached.data {
                    reporter.on_reached();
                    publish_navigation_state(&transport, &node.outputs[0], &reporter).await?;
                }
            }
            message = cancel.next() => {
                let frame = required_frame(message, "navigation_state.cancel")?;
                let cancel = schema::decode_cancel_navigation_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("NavigationState cancel decode: {error}")))?;
                publish_bool(&transport, &node.outputs[1], true).await?;
                publish_twist3(&transport, &node.outputs[2], Twist3::ZERO).await?;
                reporter.on_cancel(cancel.session_id);
                publish_navigation_state(&transport, &node.outputs[0], &reporter).await?;
            }
        }
    }
    publish_twist3(&transport, &node.outputs[2], Twist3::ZERO).await
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
struct NavigationStateReporter {
    session_id: String,
    state: i32,
    goal: [f64; 3],
    current: [f64; 3],
    initial_distance: f64,
    progress: f64,
    message: String,
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
impl Default for NavigationStateReporter {
    fn default() -> Self {
        Self {
            session_id: String::new(),
            state: 0,
            goal: [0.0; 3],
            current: [0.0; 3],
            initial_distance: 0.0,
            progress: 0.0,
            message: "nav-stack healthy".into(),
        }
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
impl NavigationStateReporter {
    fn on_goal(&mut self, goal: &PoseStamped) {
        self.session_id.clear();
        self.goal = [
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.orientation.yaw(),
        ];
        self.initial_distance = distance_2d(self.goal, self.current);
        self.progress = 0.0;
        self.state = 1;
        self.message = "planning".into();
    }

    fn on_odometry(&mut self, odometry: &PoseStamped) {
        self.current = [
            odometry.pose.position.x,
            odometry.pose.position.y,
            odometry.pose.orientation.yaw(),
        ];
        if self.state == 1 || self.state == 2 {
            let remaining = distance_2d(self.goal, self.current);
            self.progress = if self.initial_distance > f64::EPSILON {
                (1.0 - remaining / self.initial_distance).clamp(0.0, 1.0)
            } else {
                0.0
            };
        }
    }

    fn on_path(&mut self, path: &Path2D) {
        if !path.points.is_empty() {
            self.state = 2;
            self.message = "moving".into();
        }
    }

    fn on_reached(&mut self) {
        self.state = 3;
        self.progress = 1.0;
        self.message = "arrived".into();
    }

    fn on_cancel(&mut self, session_id: String) {
        self.session_id = session_id;
        self.state = 0;
        self.progress = 0.0;
        self.message = "CANCELLED: stack published stop and zero command".into();
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
async fn publish_navigation_state(
    transport: &LcmTransport,
    channel: &str,
    reporter: &NavigationStateReporter,
) -> NavStackResult<()> {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    let body = schema::encode_navigation_state_body(&schema::NavigationState {
        session_id: reporter.session_id.clone(),
        robot_frame: "base_link".into(),
        stamp_sec: now.as_secs().min(i64::MAX as u64) as i64,
        stamp_nsec: now.subsec_nanos() as i32,
        state: reporter.state,
        goal: reporter.goal,
        current: reporter.current,
        progress: reporter.progress,
        message: reporter.message.clone(),
    })
    .map_err(|error| NavStackError::Internal(format!("NavigationState encode: {error}")))?;
    publish_body(transport, channel, "nuwax_msgs.NavigationState", body).await
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn distance_2d(left: [f64; 3], right: [f64; 3]) -> f64 {
    (left[0] - right[0]).hypot(left[1] - right[1])
}

#[cfg(feature = "click-goal")]
pub(crate) fn click_goal_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 1, &[1, 2])?;
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
    use nuwax_nav_click_goal_bridge::{
        ClickPoint, ClickStartGoalRouter, ClickStartGoalRouterConfig, RoutedClick,
    };

    let transport = lcm_transport()?;
    let mut clicks =
        typed_stream(&transport, &node.inputs[0], "geometry_msgs.PointStamped").await?;
    let mut router = ClickStartGoalRouter::new(ClickStartGoalRouterConfig::default());
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = clicks.next() => {
                let frame = required_frame(message, "click_goal_router.clicked_point")?;
                let click = schema::decode_point_stamped_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("clicked_point decode: {error}")))?;
                let routed = router.handle_click(&ClickPoint::new(
                    click.header.frame_id,
                    click.point[0],
                    click.point[1],
                    click.point[2],
                ));
                match routed {
                    RoutedClick::StartPose(pose) => {
                        if node.outputs.len() == 2 {
                            publish_pose(&transport, &node.outputs[0], &pose).await?;
                        }
                    }
                    RoutedClick::GoalPose(pose) => {
                        let topic = node.outputs.last().expect("validated click output");
                        publish_pose(&transport, topic, &pose).await?;
                    }
                }
            }
        }
    }
    Ok(())
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

#[cfg(feature = "simple-2d")]
pub(crate) fn simple_planner_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 6, &[2])?;
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
    let mut terrain = typed_stream(&transport, &node.inputs[0], "sensor_msgs.PointCloud2").await?;
    let mut terrain_ext =
        typed_stream(&transport, &node.inputs[1], "sensor_msgs.PointCloud2").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[2], "nav_msgs.Odometry").await?;
    let mut start_pose =
        typed_stream(&transport, &node.inputs[3], "geometry_msgs.PoseStamped").await?;
    let mut goal_pose =
        typed_stream(&transport, &node.inputs[4], "geometry_msgs.PoseStamped").await?;
    let mut stop = typed_stream(&transport, &node.inputs[5], "std_msgs.Bool").await?;
    let mut module = SimplePlannerModule::new();
    Module::start(&mut module).map_err(|error| module_error("simple_planner", error))?;
    let mut ticker = tokio::time::interval(Duration::from_millis(
        nuwax_nav_simple_planner::module::TICK_PERIOD_MS,
    ));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = terrain.next() => {
                let frame = required_frame(message, "simple_planner.terrain_map")?;
                Module::on_terrain_map(&mut module, decode_cloud_body(&frame.payload)?)
                    .map_err(|error| module_error("simple_planner.terrain_map", error))?;
            }
            message = terrain_ext.next() => {
                let frame = required_frame(message, "simple_planner.terrain_map_ext")?;
                Module::on_terrain_map_ext(&mut module, decode_cloud_body(&frame.payload)?)
                    .map_err(|error| module_error("simple_planner.terrain_map_ext", error))?;
            }
            message = odometry.next() => {
                let frame = required_frame(message, "simple_planner.odometry")?;
                Module::on_odometry(&mut module, decode_odometry_pose_body(&frame.payload)?)
                    .map_err(|error| module_error("simple_planner.odometry", error))?;
            }
            message = start_pose.next() => {
                let frame = required_frame(message, "simple_planner.start_pose")?;
                Module::on_odometry(&mut module, decode_pose_body(&frame.payload)?)
                    .map_err(|error| module_error("simple_planner.start_pose", error))?;
            }
            message = goal_pose.next() => {
                let frame = required_frame(message, "simple_planner.goal_pose")?;
                let pose = decode_pose_body(&frame.payload)?;
                Module::on_goal(&mut module, PointStamped::new(pose.frame_id, pose.stamp, pose.pose.position))
                    .map_err(|error| module_error("simple_planner.goal_pose", error))?;
            }
            message = stop.next() => {
                let frame = required_frame(message, "simple_planner.stop_movement")?;
                let stop = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("stop_movement decode: {error}")))?;
                Module::on_stop_movement(&mut module, stop.data)
                    .map_err(|error| module_error("simple_planner.stop_movement", error))?;
            }
            _ = ticker.tick() => {
                let output = Module::tick(&mut module)
                    .map_err(|error| module_error("simple_planner.tick", error))?;
                publish_simple_output(&transport, &node, output).await?;
            }
        }
    }
    Module::stop(&mut module).map_err(|error| module_error("simple_planner.stop", error))
}

#[cfg(feature = "simple-2d")]
async fn publish_simple_output(
    transport: &LcmTransport,
    node: &ModuleNode,
    output: nuwax_nav_simple_planner::module::ModuleOutput,
) -> NavStackResult<()> {
    use nuwax_nav_simple_planner::module::ModuleOutput;

    match output {
        ModuleOutput::Plan { path, waypoint } => {
            publish_point(transport, &node.outputs[0], &waypoint).await?;
            publish_path(
                transport,
                &node.outputs[1],
                &path,
                waypoint.frame_id,
                waypoint.stamp,
            )
            .await
        }
        ModuleOutput::Waypoint(waypoint) => {
            publish_point(transport, &node.outputs[0], &waypoint).await
        }
        ModuleOutput::SafetyHold { waypoint, path, .. } => {
            publish_point(transport, &node.outputs[0], &waypoint).await?;
            publish_path(
                transport,
                &node.outputs[1],
                &path,
                waypoint.frame_id,
                waypoint.stamp,
            )
            .await
        }
        ModuleOutput::Idle => Ok(()),
    }
}

#[cfg(feature = "replanning-astar")]
pub(crate) fn astar_planner_owner(node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
    validate_ports(node, 5, &[5])?;
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
    let mut odometry = typed_stream(&transport, &node.inputs[0], "nav_msgs.Odometry").await?;
    let mut costmap = typed_stream(&transport, &node.inputs[1], "nav_msgs.OccupancyGrid").await?;
    let mut goal = typed_stream(&transport, &node.inputs[2], "geometry_msgs.PoseStamped").await?;
    let mut stop = typed_stream(&transport, &node.inputs[3], "std_msgs.Bool").await?;
    let mut planner_control = typed_stream(&transport, &node.inputs[4], "std_msgs.Bool").await?;
    let config = nuwax_nav_astar::GlobalPlannerConfig::default();
    let patrol_clearance =
        config.robot_rotation_diameter / 2.0 + nuwax_nav_patrolling::EXTRA_CLEARANCE;
    let mut module = ReplanningAStarPlannerModule::new(config);
    NavigationInterface::start(&mut module)
        .map_err(|error| module_error("astar_planner", error))?;
    let mut has_odometry = false;
    let mut has_costmap = false;
    let mut ticker = tokio::time::interval(Duration::from_millis(100));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            _ = &mut shutdown => break,
            message = odometry.next() => {
                let frame = required_frame(message, "astar_planner.odometry")?;
                let odometry = decode_odometry_pose_body(&frame.payload)?;
                require_map_pose(&odometry, "astar_planner.odometry")?;
                Module::on_odom(&mut module, odometry)
                    .map_err(|error| module_error("astar_planner.odometry", error))?;
                has_odometry = true;
            }
            message = costmap.next() => {
                let frame = required_frame(message, "astar_planner.costmap")?;
                Module::on_costmap(&mut module, decode_astar_costmap(&frame.payload)?)
                    .map_err(|error| module_error("astar_planner.costmap", error))?;
                has_costmap = true;
            }
            message = goal.next() => {
                let frame = required_frame(message, "astar_planner.goal")?;
                let goal = decode_pose_body(&frame.payload)?;
                require_map_pose(&goal, "astar_planner.goal")?;
                Module::on_goal(&mut module, goal)
                    .map_err(|error| module_error("astar_planner.goal", error))?;
            }
            message = stop.next() => {
                let frame = required_frame(message, "astar_planner.stop_movement")?;
                let stop = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("stop_movement decode: {error}")))?;
                Module::on_stop_movement(&mut module, stop.data)
                    .map_err(|error| module_error("astar_planner.stop_movement", error))?;
            }
            message = planner_control.next() => {
                let frame = required_frame(message, "astar_planner.patrol_control")?;
                let active = schema::decode_bool_body(&frame.payload)
                    .map_err(|error| NavStackError::Internal(format!("patrol planner control decode: {error}")))?
                    .data;
                apply_astar_patrol_control(&mut module, active, patrol_clearance)?;
                publish_bool(&transport, &node.outputs[4], active).await?;
            }
            _ = ticker.tick(), if has_odometry && has_costmap => {
                let output = Module::tick(&mut module)
                    .map_err(|error| module_error("astar_planner.tick", error))?;
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
                if let Some(path) = update.path {
                    publish_path(
                        &transport,
                        &node.outputs[1],
                        &path,
                        NavFrameId::Map,
                        Timestamp::default(),
                    ).await?;
                }
                publish_bool(&transport, &node.outputs[2], update.goal_reached).await?;
                if terminal_failure {
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

#[cfg(feature = "replanning-astar")]
fn apply_astar_patrol_control(
    module: &mut nuwax_nav_astar::module::ReplanningAStarPlannerModule,
    active: bool,
    patrol_clearance: f64,
) -> NavStackResult<()> {
    use nuwax_nav_astar::module::NavigationInterface;

    if active {
        NavigationInterface::set_replanning_enabled(module, false)
            .map_err(|error| module_error("astar_planner.disable_replanning", error))?;
        if let Err(error) = NavigationInterface::set_safe_goal_clearance(module, patrol_clearance) {
            let _ = NavigationInterface::set_replanning_enabled(module, true);
            let _ = NavigationInterface::reset_safe_goal_clearance(module);
            return Err(module_error("astar_planner.set_patrol_clearance", error));
        }
    } else {
        NavigationInterface::set_replanning_enabled(module, true)
            .map_err(|error| module_error("astar_planner.enable_replanning", error))?;
        NavigationInterface::reset_safe_goal_clearance(module)
            .map_err(|error| module_error("astar_planner.reset_safe_goal_clearance", error))?;
    }
    Ok(())
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
    let mut odometry = typed_stream(&transport, &node.inputs[0], "nav_msgs.Odometry").await?;
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
            message = odometry.next() => {
                let frame = required_frame(message, "patrolling.odometry")?;
                let odometry = decode_odometry_pose_body(&frame.payload)?;
                if odometry.frame_id != NavFrameId::Map {
                    return Err(NavStackError::InvalidConfig(format!(
                        "patrolling odometry frame must be map, got {}",
                        odometry.frame_id.as_str()
                    )));
                }
                module.update_odometry(odometry);
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

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "mls"
))]
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

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "mls"
))]
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

fn decode_point_body(body: &[u8]) -> NavStackResult<PointStamped> {
    let point = schema::decode_point_stamped_body(body)
        .map_err(|error| NavStackError::Internal(format!("PointStamped decode: {error}")))?;
    Ok(PointStamped::new(
        NavFrameId::from_static(&point.header.frame_id),
        timestamp(&point.header)?,
        Point3::new(point.point[0], point.point[1], point.point[2]),
    ))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn decode_pose_body(body: &[u8]) -> NavStackResult<PoseStamped> {
    let pose = schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PoseStamped decode: {error}")))?;
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

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "mls",
    feature = "patrolling",
    feature = "explore"
))]
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
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "mls"
))]
fn decode_path_body(body: &[u8]) -> NavStackResult<Path2D> {
    let path = schema::decode_path(&prefixed(schema::PATH_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Path decode: {error}")))?;
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
            .into_iter()
            .map(|pose| Point2::new(pose.pose.position[0], pose.pose.position[1]))
            .collect(),
    ))
}

fn decode_twist3_body(body: &[u8]) -> NavStackResult<Twist3> {
    let twist = schema::decode_twist(&prefixed(schema::TWIST_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Twist decode: {error}")))?;
    Ok(Twist3 {
        linear: Point3::new(twist.linear[0], twist.linear[1], twist.linear[2]),
        angular: Point3::new(twist.angular[0], twist.angular[1], twist.angular[2]),
    })
}

#[cfg(any(feature = "simple-2d", feature = "cmu-native", feature = "explore"))]
fn decode_cloud_body(body: &[u8]) -> NavStackResult<PointCloud> {
    let cloud = schema::decode_point_cloud2(&prefixed(schema::POINT_CLOUD2_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PointCloud2 decode: {error}")))?;
    point_cloud_from_schema(cloud)
}

#[cfg(any(feature = "simple-2d", feature = "cmu-native", feature = "explore"))]
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

#[cfg(any(feature = "simple-2d", feature = "cmu-native", feature = "explore"))]
fn cloud_field<'a>(
    fields: &'a [schema::PointField],
    name: &str,
) -> NavStackResult<&'a schema::PointField> {
    fields
        .iter()
        .find(|field| field.name == name && field.count >= 1)
        .ok_or_else(|| NavStackError::InvalidConfig(format!("PointCloud2 missing {name} field")))
}

#[cfg(any(feature = "simple-2d", feature = "cmu-native", feature = "explore"))]
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

#[cfg(feature = "replanning-astar")]
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
async fn publish_path(
    transport: &LcmTransport,
    channel: &str,
    path: &Path2D,
    frame: NavFrameId,
    stamp: Timestamp,
) -> NavStackResult<()> {
    let header = schema_header(stamp, frame.as_str())?;
    let poses = path
        .points
        .iter()
        .map(|point| schema::PoseStamped {
            header: header.clone(),
            pose: schema::Pose {
                position: [point.x, point.y, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .collect();
    let body = schema::encode_path_body(&schema::Path { header, poses })
        .map_err(|error| NavStackError::Internal(format!("Path encode: {error}")))?;
    publish_body(transport, channel, "nav_msgs.Path", body).await
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

#[cfg(all(test, feature = "replanning-astar"))]
mod tests {
    use super::*;

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
}
