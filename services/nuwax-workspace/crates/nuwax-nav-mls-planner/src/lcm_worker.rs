//! Real LCM worker for the MLS planner.
//!
//! The input task remains responsive while the independent planning task owns
//! the graph. Map updates cross that boundary through a Tokio watch channel,
//! so only the newest pending map is retained while an expensive update or
//! plan is running.

use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use futures::StreamExt;
use nuwax_codec::lcm::schema::{self, Header, Path, PointCloud2, PointField, Pose, PoseStamped};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::{LcmFrameStream, LcmTransport};
use thiserror::Error;
use tokio::sync::{mpsc, watch};
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
    /// Active goal pose. A non-finite coordinate cancels the goal.
    pub goal_pose: String,
    /// Planned path output.
    pub path: String,
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
            goal_pose: "/mls/goal_pose".into(),
            path: "/mls/path".into(),
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
    /// MLS algorithm parameters.
    pub planner: Config,
}

impl Default for WorkerOptions {
    fn default() -> Self {
        Self {
            provider_url: std::env::var("NUWAX_LCM_URL")
                .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=1".into()),
            topics: WorkerTopics::default(),
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
    Goal(PoseStamped),
    Failed(String),
    Closed(&'static str),
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
    Goal,
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
    let goal = subscribe(
        &transport,
        &options.topics.goal_pose,
        "geometry_msgs.PoseStamped",
    )
    .await?;

    let (event_tx, event_rx) = mpsc::unbounded_channel();
    let pumps = PumpSet(vec![
        tokio::spawn(pump_cloud(global, CloudInput::Global, event_tx.clone())),
        tokio::spawn(pump_cloud(local, CloudInput::Local, event_tx.clone())),
        tokio::spawn(pump_pose(bounds, PoseInput::Bounds, event_tx.clone())),
        tokio::spawn(pump_pose(start, PoseInput::Start, event_tx.clone())),
        tokio::spawn(pump_pose(goal, PoseInput::Goal, event_tx)),
    ]);

    let (map_tx, map_rx) = watch::channel::<Option<Arc<MapUpdate>>>(None);
    let (start_tx, start_rx) = watch::channel::<Option<Xyz>>(None);
    let (goal_tx, goal_rx) = watch::channel::<Option<Xyz>>(None);

    let input_loop = handle_inputs(event_rx, map_tx, start_tx, goal_tx);
    let planning_loop = plan_and_publish(
        transport,
        options.topics,
        options.planner,
        map_rx,
        start_rx,
        goal_rx,
    );
    let result = tokio::try_join!(input_loop, planning_loop).map(|_| ());
    drop(pumps);
    result
}

async fn subscribe(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
) -> Result<LcmFrameStream, WorkerError> {
    let topic = topic(channel)?;
    let message_type = message_type(type_name)?;
    Ok(transport.subscribe(topic, Some(message_type)).await)
}

async fn pump_cloud(
    mut stream: LcmFrameStream,
    port: CloudInput,
    tx: mpsc::UnboundedSender<InputEvent>,
) {
    let name = match port {
        CloudInput::Global => "global_map",
        CloudInput::Local => "local_map",
    };
    while let Some(frame) = stream.next().await {
        let frame = match frame {
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
    let _ = tx.send(InputEvent::Closed(name));
}

async fn pump_pose(
    mut stream: LcmFrameStream,
    port: PoseInput,
    tx: mpsc::UnboundedSender<InputEvent>,
) {
    let name = match port {
        PoseInput::Bounds => "region_bounds",
        PoseInput::Start => "start_pose",
        PoseInput::Goal => "goal_pose",
    };
    while let Some(frame) = stream.next().await {
        let frame = match frame {
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
                    PoseInput::Goal => InputEvent::Goal(message),
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
    let _ = tx.send(InputEvent::Closed(name));
}

async fn handle_inputs(
    mut events: mpsc::UnboundedReceiver<InputEvent>,
    map_tx: watch::Sender<Option<Arc<MapUpdate>>>,
    start_tx: watch::Sender<Option<Xyz>>,
    goal_tx: watch::Sender<Option<Xyz>>,
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
                // Match the original worker: odometry only refreshes the latest
                // start pose and never wakes replanning by itself.
                start_tx.send_replace(Some(pose_position(&start.pose)));
            }
            InputEvent::Goal(goal) => {
                goal_tx.send_replace(finite_position(&goal.pose));
            }
            InputEvent::Failed(error) => return Err(WorkerError::Transport(error)),
            InputEvent::Closed(port) => {
                return Err(WorkerError::Transport(format!(
                    "required input stream {port} closed"
                )));
            }
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

async fn plan_and_publish(
    transport: LcmTransport,
    topics: WorkerTopics,
    config: Config,
    mut map_rx: watch::Receiver<Option<Arc<MapUpdate>>>,
    start_rx: watch::Receiver<Option<Xyz>>,
    mut goal_rx: watch::Receiver<Option<Xyz>>,
) -> Result<(), WorkerError> {
    let mut planner = Planner::default();
    let mut last_viz_at = None;
    let mut active_goal = None;

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
                if updated && visualization_due(&config, last_viz_at) {
                    let stamp = now_header(&config.world_frame);
                    let (surface, nodes, edges) = build_graph_messages(&planner, &config, stamp);
                    publish_point_cloud(&transport, &topics.surface_map, surface).await?;
                    publish_point_cloud(&transport, &topics.nodes, nodes).await?;
                    publish_path(&transport, &topics.node_edges, edges).await?;
                    last_viz_at = Some(Instant::now());
                }
            }
        } else {
            active_goal = *goal_rx.borrow_and_update();
        }

        let Some(start) = *start_rx.borrow() else {
            continue;
        };
        let Some(goal) = active_goal else {
            continue;
        };
        if is_at_goal(start, goal, config.goal_tolerance) {
            active_goal = None;
            continue;
        }

        let waypoints =
            tokio::task::block_in_place(|| planner.plan_or_truncate(start, goal, &config));
        let path = path_from_waypoints(
            &waypoints,
            &config.world_frame,
            now_header(&config.world_frame),
        );
        publish_path(&transport, &topics.path, path).await?;
    }
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

fn finite_position(pose: &Pose) -> Option<Xyz> {
    let point = pose_position(pose);
    (point.0.is_finite() && point.1.is_finite() && point.2.is_finite()).then_some(point)
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
    if x + 4 > step || y + 4 > step || z + 4 > step {
        return Err(WorkerError::Codec(
            "PointCloud2 xyz offsets exceed point_step".into(),
        ));
    }
    let count = (message.width as usize)
        .checked_mul(message.height as usize)
        .ok_or_else(|| WorkerError::Codec("PointCloud2 point count overflow".into()))?;
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
    fn non_finite_goal_is_the_cancel_signal() {
        let mut pose = Pose {
            position: [1.0, 2.0, 3.0],
            ..Pose::default()
        };
        assert_eq!(finite_position(&pose), Some((1.0, 2.0, 3.0)));
        pose.position[1] = f64::NAN;
        assert_eq!(finite_position(&pose), None);
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
