//! FAR 规划器的真实 liblcm typed worker。
//!
//! worker 只消费版本化 `GoalEnvelope` 和精确关联的 `CancelNavigation`。
//! 算法结果通过 FAR 私有 `NavigationState` v2 合同发布，由后续唯一 adapter
//! 封装 canonical `PathEnvelope`；本模块绝不发布 canonical path topic。

use std::collections::HashMap;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use nuwax_codec::lcm::schema::{
    self, CancelNavigation, GoalEnvelope, NavigationState, Odometry, Path, PointCloud2, Pose,
    PoseStamped,
};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::{LcmTransport, OwnedLcmSubscription};
use tokio::sync::mpsc;

use crate::core::{plan, CircleObstacle, FarCoreConfig, Point2};

const GOAL_TYPE: &str = "nuwax_msgs.GoalEnvelope";
const CANCEL_TYPE: &str = "nuwax_msgs.CancelNavigation";
const ODOMETRY_TYPE: &str = "nav_msgs.Odometry";
const POINT_CLOUD_TYPE: &str = "sensor_msgs.PointCloud2";
const RESULT_TYPE: &str = "nuwax_msgs.NavigationState";
const EVENT_TYPE: &str = "nuwax_msgs.NavigationState";
const PRIVATE_RESULT_MAGIC: &[u8; 4] = b"FAR2";
const PRIVATE_RESULT_VERSION: u8 = 2;
const PRIVATE_RESULT_KIND_PATH: u8 = 1;
const MAX_PRIVATE_PATH_BYTES: usize = 4 * 1024 * 1024;

/// FAR 私有 result v2 的 READY 合同标记。
pub const FAR_RESULT_V2_CONTRACT: &str = "far-private-result-v2";

/// worker READY 事件。
pub const FAR_EVENT_READY: i32 = 1;
/// worker 成功终态。
pub const FAR_EVENT_SUCCEEDED: i32 = 2;
/// worker 取消终态。
pub const FAR_EVENT_CANCELLED: i32 = 3;
/// worker 超时终态。
pub const FAR_EVENT_TIMEOUT: i32 = 4;
/// worker 拒绝非法输入或无路径终态。
pub const FAR_EVENT_REJECTED: i32 = 5;
/// worker 算法任务崩溃终态。
pub const FAR_EVENT_CRASHED: i32 = 6;

/// FAR worker 使用的 typed topic。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WorkerTopics {
    /// 版本化导航目标输入。
    pub goal: String,
    /// 精确关联取消输入。
    pub cancel: String,
    /// 机器人当前位姿输入。
    pub odometry: String,
    /// FAR 扩展地形点云输入。
    pub terrain_map_ext: String,
    /// FAR 地形点云输入。
    pub terrain_map: String,
    /// FAR 注册扫描点云输入。
    pub registered_scan: String,
    /// FAR 私有规划结果；不是 canonical `PathEnvelope`。
    pub planner_result: String,
    /// READY、取消和终态事件。
    pub worker_event: String,
}

impl Default for WorkerTopics {
    fn default() -> Self {
        Self {
            goal: "/nav/far/goal".into(),
            cancel: "/nav/far/cancel".into(),
            odometry: "/nav/far/odometry".into(),
            terrain_map_ext: "/nav/far/terrain_map_ext".into(),
            terrain_map: "/nav/far/terrain_map".into(),
            registered_scan: "/nav/far/registered_scan".into(),
            planner_result: "/nav/far/planner_result".into(),
            worker_event: "/nav/far/worker_event".into(),
        }
    }
}

/// FAR READY v2 的完整结构化合同。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FarReadyContract {
    /// 固定 backend 身份。
    pub backend: String,
    /// supervisor 分配的单调 generation。
    pub generation: i64,
    /// GoalEnvelope 输入通道。
    pub goal: String,
    /// CancelNavigation 输入通道。
    pub cancel: String,
    /// Odometry 输入通道。
    pub odometry: String,
    /// 扩展地形点云输入通道。
    pub terrain_map_ext: String,
    /// 地形点云输入通道。
    pub terrain_map: String,
    /// 注册扫描点云输入通道。
    pub registered_scan: String,
    /// backend 私有结果通道。
    pub planner_result: String,
    /// worker 事件通道。
    pub worker_event: String,
    /// backend 私有结果合同。
    pub result_contract: String,
}

impl FarReadyContract {
    fn from_options(options: &WorkerOptions) -> Self {
        Self {
            backend: options.backend_id.clone(),
            generation: options.planner_generation,
            goal: options.topics.goal.clone(),
            cancel: options.topics.cancel.clone(),
            odometry: options.topics.odometry.clone(),
            terrain_map_ext: options.topics.terrain_map_ext.clone(),
            terrain_map: options.topics.terrain_map.clone(),
            registered_scan: options.topics.registered_scan.clone(),
            planner_result: options.topics.planner_result.clone(),
            worker_event: options.topics.worker_event.clone(),
            result_contract: FAR_RESULT_V2_CONTRACT.into(),
        }
    }

    /// 编码唯一 canonical FAR READY v2 表示。
    #[must_use]
    pub fn encode_v2(&self) -> String {
        format!(
            "far-ready-v2;backend={};generation={};goal={}#{};cancel={}#{};odometry={}#{};terrain_ext={}#{};terrain={}#{};scan={}#{};result={}#{};event={}#{};result_contract={}",
            self.backend,
            self.generation,
            self.goal,
            GOAL_TYPE,
            self.cancel,
            CANCEL_TYPE,
            self.odometry,
            ODOMETRY_TYPE,
            self.terrain_map_ext,
            POINT_CLOUD_TYPE,
            self.terrain_map,
            POINT_CLOUD_TYPE,
            self.registered_scan,
            POINT_CLOUD_TYPE,
            self.planner_result,
            RESULT_TYPE,
            self.worker_event,
            EVENT_TYPE,
            self.result_contract,
        )
    }
}

/// 严格解码 canonical FAR READY v2，拒绝尾随、重复及非 canonical 表示。
pub fn decode_ready_contract_v2(message: &str) -> Result<FarReadyContract, WorkerError> {
    let fields: Vec<&str> = message.split(';').collect();
    if fields.len() != 12 || fields[0] != "far-ready-v2" {
        return Err(WorkerError::Contract(
            "FAR READY v2 field count or prefix mismatch".into(),
        ));
    }
    let value = |index: usize, prefix: &str| {
        fields[index].strip_prefix(prefix).ok_or_else(|| {
            WorkerError::Contract(format!("FAR READY v2 field {index} is noncanonical"))
        })
    };
    let typed = |index: usize, prefix: &str, type_name: &str| {
        value(index, prefix)?
            .strip_suffix(&format!("#{type_name}"))
            .map(str::to_owned)
            .ok_or_else(|| {
                WorkerError::Contract(format!("FAR READY v2 typed field {index} mismatch"))
            })
    };
    let contract = FarReadyContract {
        backend: value(1, "backend=")?.to_owned(),
        generation: value(2, "generation=")?
            .parse()
            .map_err(|_| WorkerError::Contract("FAR READY v2 generation is invalid".into()))?,
        goal: typed(3, "goal=", GOAL_TYPE)?,
        cancel: typed(4, "cancel=", CANCEL_TYPE)?,
        odometry: typed(5, "odometry=", ODOMETRY_TYPE)?,
        terrain_map_ext: typed(6, "terrain_ext=", POINT_CLOUD_TYPE)?,
        terrain_map: typed(7, "terrain=", POINT_CLOUD_TYPE)?,
        registered_scan: typed(8, "scan=", POINT_CLOUD_TYPE)?,
        planner_result: typed(9, "result=", RESULT_TYPE)?,
        worker_event: typed(10, "event=", EVENT_TYPE)?,
        result_contract: value(11, "result_contract=")?.to_owned(),
    };
    if contract.generation <= 0 || contract.encode_v2() != message {
        return Err(WorkerError::Contract(
            "FAR READY v2 is not canonical".into(),
        ));
    }
    Ok(contract)
}

/// FAR typed worker 配置。
#[derive(Debug, Clone)]
pub struct WorkerOptions {
    /// 真实 liblcm provider URL。
    pub provider_url: String,
    /// worker typed topic。
    pub topics: WorkerTopics,
    /// 可审计 visibility graph 配置。
    pub planner: FarCoreConfig,
    /// 单次规划的硬超时。
    pub plan_timeout: Duration,
    /// supervisor 分配的单调 planner generation。
    pub planner_generation: i64,
    /// 固定 backend 身份。
    pub backend_id: String,
}

impl Default for WorkerOptions {
    fn default() -> Self {
        Self {
            provider_url: "udpm://239.255.76.67:7667?ttl=0".into(),
            topics: WorkerTopics::default(),
            planner: FarCoreConfig::default(),
            plan_timeout: Duration::from_secs(2),
            planner_generation: 1,
            backend_id: "far-visibility-graph".into(),
        }
    }
}

/// FAR worker 错误。
#[derive(Debug, thiserror::Error)]
pub enum WorkerError {
    /// worker 配置或 topic 合同非法。
    #[error("invalid FAR worker contract: {0}")]
    Contract(String),
    /// typed codec 拒绝输入或输出。
    #[error("FAR worker codec error: {0}")]
    Codec(String),
    /// liblcm transport 失败。
    #[error("FAR worker transport error: {0}")]
    Transport(String),
}

/// 严格解码 FAR 私有 result v2，并重算 Path body 摘要。
pub fn decode_private_result_v2(state: &NavigationState) -> Result<Path, WorkerError> {
    if state.state != FAR_EVENT_SUCCEEDED
        || state.runtime_generation <= 0
        || state.path_revision <= 0
        || state.goal_payload_sha256 == [0; 32]
        || state.path_payload_sha256 == [0; 32]
    {
        return Err(WorkerError::Codec(
            "invalid private result correlation fields".into(),
        ));
    }
    let private = hex_decode(&state.message)?;
    let minimum = PRIVATE_RESULT_MAGIC.len() + 1 + 1 + 4;
    if private.len() < minimum || &private[..4] != PRIVATE_RESULT_MAGIC {
        return Err(WorkerError::Codec("invalid private result magic".into()));
    }
    if private[4] != PRIVATE_RESULT_VERSION || private[5] != PRIVATE_RESULT_KIND_PATH {
        return Err(WorkerError::Codec(
            "unsupported private result version or kind".into(),
        ));
    }
    let declared = u32::from_be_bytes(
        private[6..10]
            .try_into()
            .expect("private result length prefix bounds checked"),
    ) as usize;
    if declared > MAX_PRIVATE_PATH_BYTES || private.len() != minimum + declared {
        return Err(WorkerError::Codec(
            "private result length mismatch or exceeds limit".into(),
        ));
    }
    let path_body = &private[minimum..];
    if nuwax_codec::sha256_bytes(path_body) != state.path_payload_sha256 {
        return Err(WorkerError::Codec(
            "private result path digest mismatch".into(),
        ));
    }
    let mut encoded = schema::PATH_FINGERPRINT.to_vec();
    encoded.extend_from_slice(path_body);
    schema::decode_path(&encoded)
        .map_err(|error| WorkerError::Codec(format!("private result path codec:{error}")))
}

/// 将 Path 严格封入 FAR 私有 result v2 `NavigationState`。
pub fn seal_private_result_v2(
    mut state: NavigationState,
    path: &Path,
) -> Result<NavigationState, WorkerError> {
    if state.state != FAR_EVENT_SUCCEEDED
        || state.runtime_generation <= 0
        || state.path_revision <= 0
        || state.goal_payload_sha256 == [0; 32]
    {
        return Err(WorkerError::Codec(
            "invalid private result correlation fields".into(),
        ));
    }
    let path_body = schema::encode_path_body(path)
        .map_err(|error| WorkerError::Codec(format!("private result path codec:{error}")))?;
    if path_body.len() > MAX_PRIVATE_PATH_BYTES {
        return Err(WorkerError::Codec(
            "private result path exceeds size limit".into(),
        ));
    }
    state.path_payload_sha256 = nuwax_codec::sha256_bytes(&path_body);
    let mut private = Vec::with_capacity(10 + path_body.len());
    private.extend_from_slice(PRIVATE_RESULT_MAGIC);
    private.push(PRIVATE_RESULT_VERSION);
    private.push(PRIVATE_RESULT_KIND_PATH);
    private.extend_from_slice(&(path_body.len() as u32).to_be_bytes());
    private.extend_from_slice(&path_body);
    state.message = hex_encode(&private);
    Ok(state)
}

#[derive(Debug, Clone, Copy)]
struct PathCorrelation {
    revision: i64,
    digest: [u8; 32],
}

#[derive(Debug)]
struct PlanCompletion {
    goal: GoalEnvelope,
    start: [f64; 3],
    outcome: PlanOutcome,
}

#[derive(Debug)]
enum PlanOutcome {
    Path(Path),
    Timeout,
    Crashed(String),
    Rejected(String),
}

struct ActivePlan {
    goal: GoalEnvelope,
    start: [f64; 3],
    task: tokio::task::JoinHandle<()>,
}

struct ObstacleCloud {
    frame_id: String,
    obstacles: Vec<CircleObstacle>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct GoalOwner {
    session_id: String,
    owner_kind: i32,
    parent_request_id: String,
    producer_runtime_generation: i64,
}

impl From<&GoalEnvelope> for GoalOwner {
    fn from(goal: &GoalEnvelope) -> Self {
        Self {
            session_id: goal.session_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            producer_runtime_generation: goal.producer_runtime_generation,
        }
    }
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

/// 创建真实 liblcm transport 并运行 worker。
pub async fn run_worker(options: WorkerOptions) -> Result<(), WorkerError> {
    validate_options(&options)?;
    let transport = LcmTransport::new_real(&options.provider_url)
        .map_err(|error| WorkerError::Transport(error.to_string()))?;
    run_with_transport(transport, options).await
}

/// 在已有 transport 上运行完全相同的 typed worker 合同。
pub async fn run_with_transport(
    transport: LcmTransport,
    options: WorkerOptions,
) -> Result<(), WorkerError> {
    validate_options(&options)?;

    // owned subscribe 只有在 liblcm owner 完成真实注册后才返回成功。
    let mut cancel = subscribe(&transport, &options.topics.cancel, CANCEL_TYPE).await?;
    let mut goal = subscribe(&transport, &options.topics.goal, GOAL_TYPE).await?;
    let mut odometry = subscribe(&transport, &options.topics.odometry, ODOMETRY_TYPE).await?;
    let mut terrain_ext = subscribe(
        &transport,
        &options.topics.terrain_map_ext,
        POINT_CLOUD_TYPE,
    )
    .await?;
    let mut terrain = subscribe(&transport, &options.topics.terrain_map, POINT_CLOUD_TYPE).await?;
    let mut scan = subscribe(
        &transport,
        &options.topics.registered_scan,
        POINT_CLOUD_TYPE,
    )
    .await?;
    let commit_fence_topic = format!("/far/fence/{}", std::process::id());
    let mut commit_fence = subscribe(&transport, &commit_fence_topic, EVENT_TYPE).await?;

    publish_event(
        &transport,
        &options,
        FAR_EVENT_READY,
        None,
        None,
        0,
        "ready:all-input-subscriptions-acknowledged",
    )
    .await?;

    let (completion_tx, mut completion_rx) = mpsc::unbounded_channel();
    let mut latest_odometry = None;
    let mut terrain_ext_obstacles: Option<ObstacleCloud> = None;
    let mut terrain_obstacles: Option<ObstacleCloud> = None;
    let mut scan_obstacles: Option<ObstacleCloud> = None;
    let mut active: Option<ActivePlan> = None;
    let mut pending_completion: Option<PlanCompletion> = None;
    let mut cancelled: HashMap<GoalIdentity, i64> = HashMap::new();
    let mut terminal_watermarks: HashMap<GoalOwner, i64> = HashMap::new();
    let mut next_path_revision = 1_i64;

    loop {
        // liblcm owner 保留数据报全序；biased select 再确保已排队 cancel 先于
        // goal 和算法完成提交，从而不依赖固定 sleep 窗口。
        tokio::select! {
            biased;
            frame = cancel.recv() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("cancel stream: {error}")))?;
                let value = decode_cancel(&frame.payload)?;
                if value.cancel_generation <= 0 {
                    publish_rejected(&transport, &options, "cancel-generation-must-be-positive").await?;
                    continue;
                }
                let identity = GoalIdentity::from(&value);
                cancelled
                    .entry(identity)
                    .and_modify(|generation| *generation = (*generation).max(value.cancel_generation))
                    .or_insert(value.cancel_generation);
                if active
                    .as_ref()
                    .is_some_and(|current| cancel_matches(&value, &current.goal))
                {
                    let current = active.take().expect("matching active plan checked");
                    current.task.abort();
                    pending_completion = None;
                    record_terminal(&mut terminal_watermarks, &current.goal);
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_CANCELLED,
                        Some(&current.goal),
                        Some(current.start),
                        value.cancel_generation,
                        "cancelled:correlated",
                    )
                    .await?;
                }
            }
            frame = goal.recv() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("goal stream: {error}")))?;
                let value = match schema::decode_goal_envelope_body(&frame.payload) {
                    Ok(value) => value,
                    Err(error) => {
                        publish_rejected(&transport, &options, &format!("goal-codec:{error}")).await?;
                        continue;
                    }
                };
                let owner = GoalOwner::from(&value);
                if terminal_watermarks
                    .get(&owner)
                    .is_some_and(|generation| *generation >= value.goal_generation)
                {
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_REJECTED,
                        Some(&value),
                        None,
                        0,
                        "rejected:stale-terminal-goal-generation",
                    )
                    .await?;
                    continue;
                }
                if let Some(cancel_generation) = cancelled.get(&GoalIdentity::from(&value)).copied() {
                    record_terminal(&mut terminal_watermarks, &value);
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_CANCELLED,
                        Some(&value),
                        None,
                        cancel_generation,
                        "cancelled:correlated-pre-dispatch",
                    )
                    .await?;
                    continue;
                }
                if active.is_some() {
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_REJECTED,
                        Some(&value),
                        None,
                        0,
                        "rejected:planner-busy",
                    )
                    .await?;
                    continue;
                }
                let Some(current_odometry) = latest_odometry.as_ref() else {
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_REJECTED,
                        Some(&value),
                        None,
                        0,
                        "rejected:odometry-unavailable",
                    )
                    .await?;
                    continue;
                };
                let start = match validate_goal_against_odometry(&value, current_odometry) {
                    Ok(start) => start,
                    Err(reason) => {
                        publish_event(
                            &transport,
                            &options,
                            FAR_EVENT_REJECTED,
                            Some(&value),
                            None,
                            0,
                            &format!("rejected:{reason}"),
                        )
                        .await?;
                        continue;
                    }
                };
                let Some(terrain_ext) = terrain_ext_obstacles.as_ref() else {
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_REJECTED,
                        Some(&value),
                        Some(start),
                        0,
                        "rejected:terrain-map-ext-unavailable",
                    )
                    .await?;
                    continue;
                };
                let Some(terrain) = terrain_obstacles.as_ref() else {
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_REJECTED,
                        Some(&value),
                        Some(start),
                        0,
                        "rejected:terrain-map-unavailable",
                    )
                    .await?;
                    continue;
                };
                if terrain_ext.frame_id != value.committed_map_frame_id
                    || terrain.frame_id != value.committed_map_frame_id
                    || scan_obstacles
                        .as_ref()
                        .is_some_and(|scan| scan.frame_id != value.committed_map_frame_id)
                {
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_REJECTED,
                        Some(&value),
                        Some(start),
                        0,
                        "rejected:obstacle-frame-mismatch",
                    )
                    .await?;
                    continue;
                }
                let mut obstacles = terrain_ext.obstacles.clone();
                obstacles.extend_from_slice(&terrain.obstacles);
                if let Some(scan) = scan_obstacles.as_ref() {
                    obstacles.extend_from_slice(&scan.obstacles);
                }
                let task = spawn_plan(
                    value.clone(),
                    start,
                    obstacles,
                    &options,
                    completion_tx.clone(),
                );
                active = Some(ActivePlan { goal: value, start, task });
            }
            frame = odometry.recv() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("odometry stream: {error}")))?;
                latest_odometry = Some(decode_odometry(&frame.payload)?);
            }
            frame = terrain_ext.recv() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("terrain ext stream: {error}")))?;
                terrain_ext_obstacles = Some(decode_obstacles(&frame.payload, options.planner.clearance)?);
            }
            frame = terrain.recv() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("terrain stream: {error}")))?;
                terrain_obstacles = Some(decode_obstacles(&frame.payload, options.planner.clearance)?);
            }
            frame = scan.recv() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("registered scan stream: {error}")))?;
                scan_obstacles = Some(decode_obstacles(&frame.payload, options.planner.clearance)?);
            }
            Some(completion) = completion_rx.recv(), if active.is_some() && pending_completion.is_none() => {
                let Some(current) = active.as_ref() else {
                    continue;
                };
                if !same_goal(&current.goal, &completion.goal) {
                    continue;
                }
                publish_commit_fence(
                    &transport,
                    &commit_fence_topic,
                    &options,
                    &completion.goal,
                )
                .await?;
                pending_completion = Some(completion);
            }
            frame = commit_fence.recv(), if pending_completion.is_some() => {
                let frame = frame.map_err(|error| WorkerError::Transport(format!("commit fence stream: {error}")))?;
                let fence = schema::decode_navigation_state_body(&frame.payload)
                    .map_err(|error| WorkerError::Codec(format!("commit-fence-codec:{error}")))?;
                let Some(completion) = pending_completion.as_ref() else {
                    continue;
                };
                if fence.runtime_generation != options.planner_generation
                    || fence.goal_generation != completion.goal.goal_generation
                    || fence.parent_request_id != completion.goal.parent_request_id
                    || fence.goal_payload_sha256 != completion.goal.payload_sha256
                {
                    continue;
                }
                let completion = pending_completion.take().expect("matching fence checked");
                let current = active.take().expect("active plan retained through fence");
                if let Some(cancel_generation) = cancelled.get(&GoalIdentity::from(&current.goal)).copied() {
                    record_terminal(&mut terminal_watermarks, &current.goal);
                    publish_event(
                        &transport,
                        &options,
                        FAR_EVENT_CANCELLED,
                        Some(&current.goal),
                        Some(completion.start),
                        cancel_generation,
                        "cancelled:correlated-commit-fence",
                    )
                    .await?;
                    continue;
                }
                record_terminal(&mut terminal_watermarks, &current.goal);
                match completion.outcome {
                    PlanOutcome::Path(path) => {
                        let correlation = publish_private_result_v2(
                            &transport,
                            &options,
                            &current.goal,
                            completion.start,
                            next_path_revision,
                            path,
                        )
                        .await?;
                        publish_success_event(
                            &transport,
                            &options,
                            &current.goal,
                            completion.start,
                            correlation,
                        )
                        .await?;
                        next_path_revision = next_path_revision.saturating_add(1);
                    }
                    PlanOutcome::Timeout => {
                        publish_event(
                            &transport,
                            &options,
                            FAR_EVENT_TIMEOUT,
                            Some(&current.goal),
                            Some(completion.start),
                            0,
                            "timeout:terminal",
                        )
                        .await?;
                    }
                    PlanOutcome::Crashed(reason) => {
                        publish_event(
                            &transport,
                            &options,
                            FAR_EVENT_CRASHED,
                            Some(&current.goal),
                            Some(completion.start),
                            0,
                            &format!("crashed:{reason}"),
                        )
                        .await?;
                    }
                    PlanOutcome::Rejected(reason) => {
                        publish_event(
                            &transport,
                            &options,
                            FAR_EVENT_REJECTED,
                            Some(&current.goal),
                            Some(completion.start),
                            0,
                            &format!("rejected:{reason}"),
                        )
                        .await?;
                    }
                }
            }
        }
    }
}

fn validate_options(options: &WorkerOptions) -> Result<(), WorkerError> {
    if options.planner_generation <= 0 {
        return Err(WorkerError::Contract(
            "planner_generation must be positive".into(),
        ));
    }
    if options.backend_id.trim().is_empty() {
        return Err(WorkerError::Contract("backend_id is empty".into()));
    }
    if options.plan_timeout.is_zero() {
        return Err(WorkerError::Contract("plan_timeout is zero".into()));
    }
    if !options.planner.clearance.is_finite()
        || options.planner.clearance < 0.0
        || options.planner.samples_per_obstacle < 4
    {
        return Err(WorkerError::Contract("invalid FAR core config".into()));
    }
    for (name, value) in [
        ("goal", &options.topics.goal),
        ("cancel", &options.topics.cancel),
        ("odometry", &options.topics.odometry),
        ("terrain_map_ext", &options.topics.terrain_map_ext),
        ("terrain_map", &options.topics.terrain_map),
        ("registered_scan", &options.topics.registered_scan),
        ("planner_result", &options.topics.planner_result),
        ("worker_event", &options.topics.worker_event),
    ] {
        topic(value).map_err(|error| WorkerError::Contract(format!("{name}: {error}")))?;
    }
    for message_type_name in [
        GOAL_TYPE,
        CANCEL_TYPE,
        ODOMETRY_TYPE,
        POINT_CLOUD_TYPE,
        RESULT_TYPE,
        EVENT_TYPE,
    ] {
        message_type(message_type_name)?;
    }
    Ok(())
}

fn spawn_plan(
    goal: GoalEnvelope,
    start: [f64; 3],
    obstacles: Vec<CircleObstacle>,
    options: &WorkerOptions,
    tx: mpsc::UnboundedSender<PlanCompletion>,
) -> tokio::task::JoinHandle<()> {
    let config = options.planner;
    let timeout = options.plan_timeout;
    tokio::spawn(async move {
        let deadline = tokio::time::Instant::now() + timeout;
        tokio::task::yield_now().await;
        let outcome = if tokio::time::Instant::now() >= deadline {
            PlanOutcome::Timeout
        } else {
            let planning_goal = goal.clone();
            let task = tokio::task::spawn_blocking(move || {
                plan_goal(&planning_goal, start, &obstacles, config)
            });
            match tokio::time::timeout_at(deadline, task).await {
                Err(_) => PlanOutcome::Timeout,
                Ok(Err(error)) => PlanOutcome::Crashed(error.to_string()),
                Ok(Ok(Ok(path))) => PlanOutcome::Path(path),
                Ok(Ok(Err(reason))) => PlanOutcome::Rejected(reason),
            }
        };
        let _ = tx.send(PlanCompletion {
            goal,
            start,
            outcome,
        });
    })
}

fn plan_goal(
    goal: &GoalEnvelope,
    start: [f64; 3],
    obstacles: &[CircleObstacle],
    config: FarCoreConfig,
) -> Result<Path, String> {
    let goal_position = goal.goal.pose.position;
    let points = plan(
        Point2 {
            x: start[0],
            y: start[1],
        },
        Point2 {
            x: goal_position[0],
            y: goal_position[1],
        },
        obstacles,
        config,
    )
    .ok_or_else(|| "no-path".to_string())?;

    if !goal.boundary_points.is_empty()
        && points
            .iter()
            .any(|point| !inside_boundary(*point, &goal.boundary_points))
    {
        return Err("path-leaves-authoritative-boundary".into());
    }

    let denominator = points.len().saturating_sub(1).max(1) as f64;
    let poses = points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let ratio = index as f64 / denominator;
            let is_goal = index + 1 == points.len();
            PoseStamped {
                header: goal.goal.header.clone(),
                pose: Pose {
                    position: [
                        point.x,
                        point.y,
                        start[2] + (goal_position[2] - start[2]) * ratio,
                    ],
                    orientation_xyzw: if is_goal {
                        goal.goal.pose.orientation_xyzw
                    } else {
                        [0.0, 0.0, 0.0, 1.0]
                    },
                },
            }
        })
        .collect();
    Ok(Path {
        header: goal.goal.header.clone(),
        poses,
    })
}

fn inside_boundary(point: Point2, boundary: &[[f64; 3]]) -> bool {
    if boundary.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut previous = boundary.len() - 1;
    for current in 0..boundary.len() {
        let a = boundary[current];
        let b = boundary[previous];
        let crosses = (a[1] > point.y) != (b[1] > point.y)
            && point.x < (b[0] - a[0]) * (point.y - a[1]) / (b[1] - a[1]) + a[0];
        if crosses {
            inside = !inside;
        }
        previous = current;
    }
    inside
}

fn validate_goal_against_odometry(
    goal: &GoalEnvelope,
    odometry: &Odometry,
) -> Result<[f64; 3], String> {
    if odometry.header.frame_id != goal.committed_map_frame_id
        || goal.goal.header.frame_id != goal.committed_map_frame_id
    {
        return Err("frame-mismatch".into());
    }
    let start = odometry.pose.position;
    if start.iter().any(|value| !value.is_finite()) {
        return Err("non-finite-odometry".into());
    }
    let orientation = odometry.pose.orientation_xyzw;
    if orientation.iter().any(|value| !value.is_finite()) {
        return Err("non-finite-odometry-orientation".into());
    }
    if !goal.boundary_points.is_empty()
        && !inside_boundary(
            Point2 {
                x: start[0],
                y: start[1],
            },
            &goal.boundary_points,
        )
    {
        return Err("odometry-outside-authoritative-boundary".into());
    }
    Ok(start)
}

fn record_terminal(watermarks: &mut HashMap<GoalOwner, i64>, goal: &GoalEnvelope) {
    watermarks
        .entry(GoalOwner::from(goal))
        .and_modify(|generation| *generation = (*generation).max(goal.goal_generation))
        .or_insert(goal.goal_generation);
}

fn same_goal(left: &GoalEnvelope, right: &GoalEnvelope) -> bool {
    left.session_id == right.session_id
        && left.owner_kind == right.owner_kind
        && left.parent_request_id == right.parent_request_id
        && left.goal_generation == right.goal_generation
        && left.producer_runtime_generation == right.producer_runtime_generation
        && left.payload_sha256 == right.payload_sha256
}

fn cancel_matches(cancel: &CancelNavigation, goal: &GoalEnvelope) -> bool {
    cancel.cancel_generation > 0
        && cancel.session_id == goal.session_id
        && cancel.owner_kind == goal.owner_kind
        && cancel.parent_request_id == goal.parent_request_id
        && cancel.goal_generation == goal.goal_generation
        && cancel.producer_runtime_generation == goal.producer_runtime_generation
}

async fn subscribe(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
) -> Result<OwnedLcmSubscription, WorkerError> {
    transport
        .subscribe_owned(topic(channel)?, Some(message_type(type_name)?))
        .await
        .map_err(|error| WorkerError::Transport(error.to_string()))
}

fn decode_cancel(body: &[u8]) -> Result<CancelNavigation, WorkerError> {
    schema::decode_cancel_navigation_body(body)
        .map_err(|error| WorkerError::Codec(format!("cancel-codec:{error}")))
}

fn decode_odometry(body: &[u8]) -> Result<Odometry, WorkerError> {
    let mut encoded = schema::ODOMETRY_FINGERPRINT.to_vec();
    encoded.extend_from_slice(body);
    schema::decode_odometry(&encoded)
        .map_err(|error| WorkerError::Codec(format!("odometry-codec:{error}")))
}

fn decode_obstacles(body: &[u8], radius: f64) -> Result<ObstacleCloud, WorkerError> {
    let mut encoded = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
    encoded.extend_from_slice(body);
    let cloud = schema::decode_point_cloud2(&encoded)
        .map_err(|error| WorkerError::Codec(format!("pointcloud-codec:{error}")))?;
    let points = extract_xy(&cloud)?;
    Ok(ObstacleCloud {
        frame_id: cloud.header.frame_id,
        obstacles: points
            .into_iter()
            .map(|center| CircleObstacle { center, radius })
            .collect(),
    })
}

fn extract_xy(cloud: &PointCloud2) -> Result<Vec<Point2>, WorkerError> {
    if cloud.width < 0 || cloud.height < 0 || cloud.point_step <= 0 || cloud.row_step <= 0 {
        return Err(WorkerError::Codec("invalid PointCloud2 dimensions".into()));
    }
    let width = usize::try_from(cloud.width)
        .map_err(|_| WorkerError::Codec("negative PointCloud2 width".into()))?;
    let height = usize::try_from(cloud.height)
        .map_err(|_| WorkerError::Codec("negative PointCloud2 height".into()))?;
    let point_step = usize::try_from(cloud.point_step)
        .map_err(|_| WorkerError::Codec("negative PointCloud2 point_step".into()))?;
    let row_step = usize::try_from(cloud.row_step)
        .map_err(|_| WorkerError::Codec("negative PointCloud2 row_step".into()))?;
    let row_payload = width
        .checked_mul(point_step)
        .ok_or_else(|| WorkerError::Codec("PointCloud2 row payload overflow".into()))?;
    if row_step < row_payload {
        return Err(WorkerError::Codec(
            "PointCloud2 row_step is smaller than width*point_step".into(),
        ));
    }
    let required = height
        .checked_mul(row_step)
        .ok_or_else(|| WorkerError::Codec("PointCloud2 data length overflow".into()))?;
    if cloud.data.len() != required {
        return Err(WorkerError::Codec(format!(
            "PointCloud2 data has {} bytes, expected {required}",
            cloud.data.len()
        )));
    }
    let x = float32_field_offset(cloud, "x", point_step)?;
    let y = float32_field_offset(cloud, "y", point_step)?;
    let mut points = Vec::new();
    let count = width.saturating_mul(height);
    let stride = (count / 512).max(1);
    for index in (0..count).step_by(stride) {
        let row = index / width.max(1);
        let column = index % width.max(1);
        let base = row * row_step + column * point_step;
        let x = read_f32(&cloud.data, base + x, cloud.is_bigendian);
        let y = read_f32(&cloud.data, base + y, cloud.is_bigendian);
        if x.is_finite() && y.is_finite() {
            points.push(Point2 { x, y });
        }
    }
    Ok(points)
}

#[allow(clippy::unnecessary_map_or)] // Rust 1.75 尚无 Option::is_none_or。
fn float32_field_offset(
    cloud: &PointCloud2,
    name: &str,
    point_step: usize,
) -> Result<usize, WorkerError> {
    let field = cloud
        .fields
        .iter()
        .find(|field| field.name == name)
        .ok_or_else(|| WorkerError::Codec(format!("missing PointCloud2 {name} field")))?;
    if field.datatype != 7 || field.count != 1 || field.offset < 0 {
        return Err(WorkerError::Codec(format!(
            "PointCloud2 {name} must be one float32"
        )));
    }
    let offset = usize::try_from(field.offset)
        .map_err(|_| WorkerError::Codec(format!("negative PointCloud2 {name} offset")))?;
    if offset.checked_add(4).map_or(true, |end| end > point_step) {
        return Err(WorkerError::Codec(format!(
            "PointCloud2 {name} field exceeds point_step"
        )));
    }
    Ok(offset)
}

fn read_f32(data: &[u8], offset: usize, big_endian: bool) -> f64 {
    let bytes: [u8; 4] = data[offset..offset + 4]
        .try_into()
        .expect("PointCloud2 field bounds checked");
    if big_endian {
        f64::from(f32::from_be_bytes(bytes))
    } else {
        f64::from(f32::from_le_bytes(bytes))
    }
}

async fn publish_commit_fence(
    transport: &LcmTransport,
    channel: &str,
    options: &WorkerOptions,
    goal: &GoalEnvelope,
) -> Result<(), WorkerError> {
    let fence = NavigationState {
        session_id: goal.session_id.clone(),
        robot_frame: options.backend_id.clone(),
        state: FAR_EVENT_READY,
        message: "far-private-commit-fence".into(),
        runtime_generation: options.planner_generation,
        owner_kind: goal.owner_kind,
        parent_request_id: goal.parent_request_id.clone(),
        goal_generation: goal.goal_generation,
        goal_payload_sha256: goal.payload_sha256,
        ..NavigationState::default()
    };
    let body = schema::encode_navigation_state_body(&fence)
        .map_err(|error| WorkerError::Codec(format!("commit-fence-codec:{error}")))?;
    publish(transport, channel, EVENT_TYPE, body).await
}

async fn publish_rejected(
    transport: &LcmTransport,
    options: &WorkerOptions,
    reason: &str,
) -> Result<(), WorkerError> {
    publish_event(
        transport,
        options,
        FAR_EVENT_REJECTED,
        None,
        None,
        0,
        &format!("rejected:{reason}"),
    )
    .await
}

async fn publish_private_result_v2(
    transport: &LcmTransport,
    options: &WorkerOptions,
    goal: &GoalEnvelope,
    current: [f64; 3],
    path_revision: i64,
    path: Path,
) -> Result<PathCorrelation, WorkerError> {
    let state = navigation_state(
        options,
        FAR_EVENT_SUCCEEDED,
        Some(goal),
        Some(current),
        0,
        PathCorrelation {
            revision: path_revision,
            digest: [0; 32],
        },
        String::new(),
    );
    let state = seal_private_result_v2(state, &path)?;
    let correlation = PathCorrelation {
        revision: state.path_revision,
        digest: state.path_payload_sha256,
    };
    let body = schema::encode_navigation_state_body(&state)
        .map_err(|error| WorkerError::Codec(error.to_string()))?;
    publish(transport, &options.topics.planner_result, RESULT_TYPE, body).await?;
    Ok(correlation)
}

async fn publish_success_event(
    transport: &LcmTransport,
    options: &WorkerOptions,
    goal: &GoalEnvelope,
    current: [f64; 3],
    correlation: PathCorrelation,
) -> Result<(), WorkerError> {
    let state = navigation_state(
        options,
        FAR_EVENT_SUCCEEDED,
        Some(goal),
        Some(current),
        0,
        correlation,
        "succeeded:private-result-v2-published".into(),
    );
    let body = schema::encode_navigation_state_body(&state)
        .map_err(|error| WorkerError::Codec(error.to_string()))?;
    publish(transport, &options.topics.worker_event, EVENT_TYPE, body).await
}

#[allow(clippy::too_many_arguments)]
fn navigation_state(
    options: &WorkerOptions,
    state: i32,
    goal: Option<&GoalEnvelope>,
    current: Option<[f64; 3]>,
    cancel_generation: i64,
    correlation: PathCorrelation,
    message: String,
) -> NavigationState {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    NavigationState {
        session_id: goal.map_or_else(String::new, |value| value.session_id.clone()),
        robot_frame: options.backend_id.clone(),
        stamp_sec: i64::try_from(now.as_secs()).unwrap_or(i64::MAX),
        stamp_nsec: now.subsec_nanos() as i32,
        state,
        goal: goal.map_or([0.0; 3], |value| value.goal.pose.position),
        current: current.unwrap_or([0.0; 3]),
        progress: f64::from(state == FAR_EVENT_SUCCEEDED),
        message,
        runtime_generation: options.planner_generation,
        guard_generation: goal.map_or(0, |value| value.motion_guard_generation),
        pose_source_timestamp_ns: 0,
        costmap_source_timestamp_ns: 0,
        sink_ready: i8::from(state == FAR_EVENT_READY),
        planner_map_frame: goal
            .map_or_else(String::new, |value| value.committed_map_frame_id.clone()),
        planner_pose_frame: goal
            .map_or_else(String::new, |value| value.goal.header.frame_id.clone()),
        cancel_generation,
        zero_ack_id: String::new(),
        zero_applied_monotonic_ns: 0,
        owner_kind: goal.map_or(0, |value| value.owner_kind),
        parent_request_id: goal.map_or_else(String::new, |value| value.parent_request_id.clone()),
        goal_generation: goal.map_or(0, |value| value.goal_generation),
        path_revision: correlation.revision,
        goal_payload_sha256: goal.map_or([0; 32], |value| value.payload_sha256),
        path_payload_sha256: correlation.digest,
    }
}

#[allow(clippy::too_many_arguments)]
async fn publish_event(
    transport: &LcmTransport,
    options: &WorkerOptions,
    state: i32,
    goal: Option<&GoalEnvelope>,
    current: Option<[f64; 3]>,
    cancel_generation: i64,
    detail: &str,
) -> Result<(), WorkerError> {
    let message = if state == FAR_EVENT_READY {
        FarReadyContract::from_options(options).encode_v2()
    } else {
        format!(
            "backend={};generation={};goal={}#{};cancel={}#{};odometry={}#{};terrain_ext={}#{};terrain={}#{};scan={}#{};result={}#{};result_contract={};event={}#{};{detail}",
            options.backend_id,
            options.planner_generation,
            options.topics.goal,
            GOAL_TYPE,
            options.topics.cancel,
            CANCEL_TYPE,
            options.topics.odometry,
            ODOMETRY_TYPE,
            options.topics.terrain_map_ext,
            POINT_CLOUD_TYPE,
            options.topics.terrain_map,
            POINT_CLOUD_TYPE,
            options.topics.registered_scan,
            POINT_CLOUD_TYPE,
            options.topics.planner_result,
            RESULT_TYPE,
            FAR_RESULT_V2_CONTRACT,
            options.topics.worker_event,
            EVENT_TYPE,
        )
    };
    let message = navigation_state(
        options,
        state,
        goal,
        current,
        cancel_generation,
        PathCorrelation {
            revision: 0,
            digest: [0; 32],
        },
        message,
    );
    let body = schema::encode_navigation_state_body(&message)
        .map_err(|error| WorkerError::Codec(error.to_string()))?;
    publish(transport, &options.topics.worker_event, EVENT_TYPE, body).await
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
    if value.len() % 2 != 0 || value.len() > (MAX_PRIVATE_PATH_BYTES + 10) * 2 {
        return Err(WorkerError::Codec(
            "invalid private result hex length".into(),
        ));
    }
    let bytes = value.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len() / 2);
    for pair in bytes.chunks_exact(2) {
        let high = hex_nibble(pair[0])?;
        let low = hex_nibble(pair[1])?;
        decoded.push((high << 4) | low);
    }
    Ok(decoded)
}

fn hex_nibble(value: u8) -> Result<u8, WorkerError> {
    match value {
        b'0'..=b'9' => Ok(value - b'0'),
        b'a'..=b'f' => Ok(value - b'a' + 10),
        _ => Err(WorkerError::Codec(
            "private result hex must be lowercase canonical encoding".into(),
        )),
    }
}

async fn publish(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
    payload: Vec<u8>,
) -> Result<(), WorkerError> {
    let frame = RawLcmFrameBuilder::new(topic(channel)?, message_type(type_name)?)
        .payload(payload)
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
            "unregistered LCM type {value}"
        )));
    }
    Ok(message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cancel_requires_exact_goal_identity() {
        let goal = GoalEnvelope {
            session_id: "session".into(),
            owner_kind: 1,
            parent_request_id: "request".into(),
            goal_generation: 7,
            producer_runtime_generation: 9,
            ..GoalEnvelope::default()
        };
        let cancel = CancelNavigation {
            session_id: "session".into(),
            cancel_generation: 1,
            owner_kind: 1,
            parent_request_id: "request".into(),
            goal_generation: 7,
            producer_runtime_generation: 9,
            ..CancelNavigation::default()
        };
        assert!(cancel_matches(&cancel, &goal));
        assert!(!cancel_matches(
            &CancelNavigation {
                goal_generation: 8,
                ..cancel
            },
            &goal
        ));
    }

    #[test]
    fn ready_v2_decoder_rejects_trailing_duplicate_and_noncanonical_forms() {
        let options = WorkerOptions::default();
        let encoded = FarReadyContract::from_options(&options).encode_v2();
        assert_eq!(
            decode_ready_contract_v2(&encoded).expect("decode canonical READY"),
            FarReadyContract::from_options(&options)
        );

        let invalid = [
            format!("{encoded};trailing=value"),
            encoded.replace(
                ";cancel=",
                ";goal=/duplicate#nuwax_msgs.GoalEnvelope;cancel=",
            ),
            encoded.replace("generation=1", "generation=01"),
            encoded.replace(";goal=", ";cancel_alias="),
            encoded.replace(";goal=", ";cancel="),
            encoded.replace(
                ";goal=/nav/far/goal#nuwax_msgs.GoalEnvelope;cancel=/nav/far/cancel#nuwax_msgs.CancelNavigation",
                ";cancel=/nav/far/cancel#nuwax_msgs.CancelNavigation;goal=/nav/far/goal#nuwax_msgs.GoalEnvelope",
            ),
        ];
        for message in invalid {
            assert!(
                decode_ready_contract_v2(&message).is_err(),
                "non-canonical READY unexpectedly decoded: {message}"
            );
        }

        assert!(decode_ready_contract_v2(
            &encoded.replace("odometry=/nav/far/odometry", "odometry=/nav/far/wrong",)
        )
        .is_ok());
    }

    #[test]
    fn boundary_gate_rejects_outside_point() {
        let boundary = vec![
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
        ];
        assert!(inside_boundary(Point2 { x: 1.0, y: 1.0 }, &boundary));
        assert!(!inside_boundary(Point2 { x: 3.0, y: 1.0 }, &boundary));
    }

    #[test]
    fn private_result_v2_recomputes_digest_and_rejects_trailing_bytes() {
        let path = Path {
            header: schema::Header {
                seq: 1,
                stamp_sec: 2,
                stamp_nsec: 3,
                frame_id: "map".into(),
            },
            poses: Vec::new(),
        };
        let state = NavigationState {
            session_id: "session".into(),
            robot_frame: "far-visibility-graph".into(),
            state: FAR_EVENT_SUCCEEDED,
            runtime_generation: 7,
            owner_kind: 1,
            parent_request_id: "request".into(),
            goal_generation: 9,
            path_revision: 2,
            goal_payload_sha256: [0x11; 32],
            ..NavigationState::default()
        };
        let sealed = seal_private_result_v2(state, &path).expect("seal private result v2");
        assert_eq!(
            decode_private_result_v2(&sealed).expect("decode private result v2"),
            path
        );

        let mut wrong_digest = sealed.clone();
        wrong_digest.path_payload_sha256[0] ^= 1;
        assert!(decode_private_result_v2(&wrong_digest).is_err());

        let mut trailing = sealed;
        let mut private = hex_decode(&trailing.message).expect("decode private bytes");
        private.push(0);
        trailing.message = hex_encode(&private);
        assert!(decode_private_result_v2(&trailing).is_err());
    }
}
