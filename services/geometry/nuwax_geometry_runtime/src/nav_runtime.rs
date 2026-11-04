#![allow(clippy::uninlined_format_args)]

//! Phase 4:nav runtime adapter —— 把 `SetNavigationGoal` / `CancelNavigation` /
//! `SubscribeNavigationState` / `GetNavigationReadiness` 从 `UNIMPLEMENTED` 变成
//! 真实导航执行链。
//!
//! ## 核心职责
//!
//! 1. 捕获两条固定完整链：二维使用`simple-2d`，三维使用`pct-scan-3d`。
//!    导航配置与建图链冲突时拒绝启动，不回退其它维度。
//! 2. 用 `nuwax_nav_stack::NavStackBuilder` 构造 `ModuleGraph`,把节点列表
//!    写入 diagnostics / nav state message 的 `message` 字段(便于 health
//!    check / 日志 / CI 守门观测)。
//! 3. goal/cancel 仅提交到真实 `nuwax-nav-stack` LCM 输入；本 facade 不持有
//!    planner/follower,也不在 set-goal 路径发布 `/cmd_vel`。
//! 4. `/nav/state` heartbeat/state 是 planner health、goal lifecycle 与 terminal
//!    状态的唯一权威来源；未观测到新鲜状态时 readiness 必须失败。
//!
//! ## 边界
//!
//! - 不会启动 native binary 进程(CMU / FAR / local / path-follower /
//!   PGO / TARE / Terrain),仅返回 module graph;P1 阶段在 native bridge
//!   进程生命周期 + supervisor 实装后再开启 cmu-native / exploration-native。
//! - 不持有 LCM 订阅线程；TF consumer、planner/scheduler 与 guard client
//!   必须通过 typed observation API 写入各自真实消费结果。SharedState 的
//!   producer-side pose/map 不能替代跨进程 consumer acknowledgement。
//! - facade 维护提交与取消安全门；`PLANNING/RUNNING/SUCCEEDED` 由 stack
//!   state 推进；取消清目标并发布一次取消请求。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use nuwax_codec::lcm::schema;
#[cfg(test)]
use nuwax_nav_stack::{NavStackHandle, NavStackPreset};
use nuwax_nav_stack::{
    graph::ModuleNode, NavStackBuilder, NavStackProfile,
};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
use nuwax_transport::{LcmTransport, TransportError};
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{info, warn};
use uuid::Uuid;

use crate::channel_registry;
use crate::lcm_runtime::{GeometryRuntimeHandle, NavigationUpdate, NavState, PoseState, SharedState};
use crate::sequence_state::{SequenceStateError, SequenceStateStore};
use crate::typed_codec;

/// 二维完整链的导航配置名。
pub const PROFILE_SIMPLE_2D: &str = "simple-2d";
/// `replanning-astar` profile 字符串。
pub const PROFILE_REPLANNING_ASTAR: &str = "replanning-astar";
/// `cmu-native` profile 字符串(P1:依赖 native bridge 进程,本期未实装)。
pub const PROFILE_CMU_NATIVE: &str = "cmu-native";
/// `exploration-native` profile 字符串(P1:依赖 native bridge 进程,本期未实装)。
pub const PROFILE_EXPLORATION_NATIVE: &str = "exploration-native";
/// 已接入统一运动守卫的多层表面导航配置名。
/// 注意:与 `NavStackProfile::Mls3D.as_str()` 一致,使用 kebab-case "pct-scan-3d"
/// 而非 "mls3d",否则 `build_nav_stack_handle_renders_for_all_profiles`
/// 测试中 `text.contains(p)` 会失败。
pub const PROFILE_MLS3D: &str = "pct-scan-3d";

#[cfg(test)]
static NEXT_NAV_RUNTIME_GENERATION: AtomicU64 = AtomicU64::new(0);

/// 目标准入只需持有地图生命周期 fence、唯一已提交地图权威与只读地图名称投影。
#[derive(Clone)]
pub(crate) struct NavAdmissionContext {
    mapping_adapter: Option<Arc<parking_lot::Mutex<crate::mapping_adapter::MappingAdapter>>>,
    allow_previous_map_revisions: bool,
    pub(crate) mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    pub(crate) committed_map_authority: Arc<StdMutex<Option<schema::CommittedMapIdentity>>>,
    map_revision_latest_rx:
        tokio::sync::watch::Receiver<Option<crate::lcm_runtime::MapRevisionEvent>>,
    expected_map_name: Option<String>,
}

impl NavAdmissionContext {
    pub(crate) fn new(
        mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
        committed_map_authority: Arc<StdMutex<Option<schema::CommittedMapIdentity>>>,
        map_revision_latest_rx: tokio::sync::watch::Receiver<
            Option<crate::lcm_runtime::MapRevisionEvent>,
        >,
    ) -> Self {
        let expected_map_name = map_revision_latest_rx
            .borrow()
            .as_ref()
            .map(|event| event.map_name.clone());
        Self::with_expected_map_name(
            mapping_lifecycle_lock,
            committed_map_authority,
            map_revision_latest_rx,
            expected_map_name,
        )
    }

    pub(crate) fn with_expected_map_name(
        mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
        committed_map_authority: Arc<StdMutex<Option<schema::CommittedMapIdentity>>>,
        map_revision_latest_rx: tokio::sync::watch::Receiver<
            Option<crate::lcm_runtime::MapRevisionEvent>,
        >,
        expected_map_name: Option<String>,
    ) -> Self {
        Self {
            mapping_lifecycle_lock,
            committed_map_authority,
            map_revision_latest_rx,
            expected_map_name,
            mapping_adapter: None,
            allow_previous_map_revisions: false,
        }
    }

    pub(crate) fn with_mapping_adapter(mut self, adapter: Option<Arc<parking_lot::Mutex<crate::mapping_adapter::MappingAdapter>>>) -> Self {
        self.mapping_adapter = adapter;
        self
    }
    pub(crate) fn with_previous_map_revisions(mut self, allowed: bool) -> Self {
        self.allow_previous_map_revisions = allowed;
        self
    }
    pub(crate) fn map_frame_generation(&self) -> i64 {
        self.mapping_adapter.as_ref().map_or(0, |a| i64::try_from(a.lock().global_frame_generation()).unwrap_or(-1))
    }

    fn map_name_unchanged(&self) -> bool {
        self.expected_map_name.as_ref().is_some_and(|expected| {
            self.map_revision_latest_rx
                .borrow()
                .as_ref()
                .is_some_and(|current| current.map_name == *expected)
        })
    }
}

/// 目标准入所需的最小运行时 I/O（输入/输出）句柄。
#[derive(Clone)]
pub(crate) struct NavAdmissionIo {
    shared_state: Arc<RwLock<SharedState>>,
    transport: LcmTransport,
    transport_backend_name: &'static str,
    lcm_backend_available: bool,
    body_reference_frame_id: Arc<str>,
}

impl NavAdmissionIo {
    pub(crate) fn new(
        shared_state: Arc<RwLock<SharedState>>,
        transport: LcmTransport,
        transport_backend_name: &'static str,
        lcm_backend_available: bool,
        body_reference_frame_id: Arc<str>,
    ) -> Self {
        Self {
            shared_state,
            transport,
            transport_backend_name,
            lcm_backend_available,
            body_reference_frame_id,
        }
    }
}

/// `/nav/goal` typed channel 平台级 channel name。
pub const GOAL_CHANNEL_PLATFORM: &str = "NUWAX_LCM_CHANNEL_GOAL_POSE";
/// `/nav/goal` 内置 default。
pub const GOAL_CHANNEL_DEFAULT: &str = "/nav/goal#nuwax_msgs.GoalEnvelope";
/// `/nav/path` versioned planner output platform key.
pub const PATH_CHANNEL_PLATFORM: &str = "NUWAX_LCM_CHANNEL_NAV_PATH";
/// `/nav/path` immutable envelope default.
pub const PATH_CHANNEL_DEFAULT: &str = "/nav/path#nuwax_msgs.PathEnvelope";

#[cfg(test)]
/// GoalEnvelope message type validated once at startup.
/// 失败应直接 panic(因为这是 typed protocol 层,不可能 fail)。
fn goal_msg_type() -> LcmMessageType {
    LcmMessageType::new("nuwax_msgs.GoalEnvelope").expect("typed nuwax_msgs.GoalEnvelope")
}

/// 单一目标状态机阶段。`status` 字段直接对应 gRPC `NavigationState.status`,
/// 这样 Python 侧 `get_navigation_state` 不用再翻译。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavPhase {
    /// 空闲(初始 / 已被 cancel / 已被 succeed 后复位)。
    Idle,
    /// 收到 set_goal,已写 active goal + 已发布 /nav/goal,等待进入规划。
    Accepted,
    /// 本次已安装目标的发送效果未确认；保留 exact 执行责任，非终态。
    DispatchUnresolved,
    /// 正在规划路径。
    Planning,
    /// 规划成功,正在跟随路径生成 cmd_vel。
    Running,
    /// 到达目标点(距离 < arrival_tolerance_m)。
    Succeeded,
    /// 用户取消，且唯一 motion guard actuator path 已确认 ZEROED。
    Cancelled,
    /// cancel 已提交，等待 motion guard ZEROED acknowledgement。
    /// 失败(readiness 不通过 / 路径无法生成 / 等)。
    Failed,
}

impl NavPhase {
    /// 序列化成 gRPC `NavigationState.status` 字符串。
    pub fn as_str(self) -> &'static str {
        match self {
            NavPhase::Idle => "IDLE",
            NavPhase::Accepted => "ACCEPTED",
            NavPhase::DispatchUnresolved => "UNCONFIRMED",
            NavPhase::Planning => "PLANNING",
            NavPhase::Running => "RUNNING",
            NavPhase::Succeeded => "SUCCEEDED",
            NavPhase::Cancelled => "CANCELLED",
            NavPhase::Failed => "FAILED",
        }
    }

    fn from_stack_status(status: &str) -> Option<Self> {
        match status.trim().to_ascii_uppercase().as_str() {
            "IDLE" => Some(Self::Idle),
            "ACCEPTED" => Some(Self::Accepted),
            "PLANNING" => Some(Self::Planning),
            "MOVING" | "RUNNING" => Some(Self::Running),
            "ARRIVED" | "SUCCEEDED" => Some(Self::Succeeded),
            "CANCELLED" | "CANCELED" => Some(Self::Cancelled),
            "FAILED" => Some(Self::Failed),
            _ => None,
        }
    }

    fn is_terminal(self) -> bool {
        matches!(self, Self::Succeeded | Self::Cancelled | Self::Failed)
    }
}

/// Immutable identity and planner correlation fields for one goal lease.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GoalLease {
    /// Public business/session identifier.
    pub session_id: String,
    /// `NAVIGATION_OWNER_KIND_*` wire value.
    pub owner_kind: i32,
    /// Public request that owns the goal.
    pub parent_request_id: String,
    /// Monotonic goal generation.
    pub goal_generation: u64,
    /// Geometry runtime generation that sealed the goal.
    pub producer_runtime_generation: u64,
    /// Last scheduler-confirmed planner path revision.
    pub path_revision: u64,
    /// Actual scheduler-confirmed planner incarnation; zero before a path.
    pub planner_runtime_generation: u64,
}

impl GoalLease {
    fn identity_matches(&self, other: &Self) -> bool {
        self.session_id == other.session_id
            && self.owner_kind == other.owner_kind
            && self.parent_request_id == other.parent_request_id
            && self.goal_generation == other.goal_generation
            && self.producer_runtime_generation == other.producer_runtime_generation
    }
}

/// 当前 active goal 的内存表示。`id` 是唯一 public session id。
#[derive(Clone)]
pub struct ActiveGoal {
    /// 目标所属的运动命令。
    pub motion_admission: MotionAdmissionProof,
    /// 目标点 ID(用户传或自动生成 UUID)。
    pub id: String,
    /// frame_id(`map` 默认)。
    pub frame_id: String,
    /// 目标位姿(在 `frame_id` 坐标系下)。
    pub target: PoseState,
    pub options: nuwax_nav_types_messages::GoalOptions,
    pub producer: String,
    pub producer_epoch: i64,

    pub boundary_owner: Option<Arc<crate::explore_runtime::ExploreStartConfig>>,
    /// Immutable assigned identity used by goal/path/cancel CAS.
    pub lease: GoalLease,
}

/// 原始终态保存实际退休来源；自然终态和已确认普通停车均不能冒充到达。
/// 旧路径回执可保留展示，但不会因此退休当前执行。
struct RetainedTerminalReceipt {
    update: NavigationUpdate,
    retired_without_child_cancel: bool,
}

impl RetainedTerminalReceipt {
    fn matches(&self, input: &CancelGoalInput) -> bool {
        let update = &self.update;
        update.state.goal_id == input.goal_id
            && update.feedback.owner_kind == input.owner_kind
            && update.feedback.parent_request_id == input.parent_request_id
            && update.feedback.goal_generation == input.goal_generation
            && update.producer_runtime_generation == input.producer_runtime_generation
    }
}

/// `NavRuntime` 的内部 slot:facade submission phase + active goal。
///
/// 一次只记录一个 active goal；新 `set_goal` 覆盖 facade 记录并由真实 stack
/// 决定 preemption，facade 不在本地运行第二套 follower。
pub struct NavRuntimeSlot {
    /// 当前阶段。
    pub phase: NavPhase,
    /// 失败原因(message 字段用)。
    pub message: String,
    /// 当前 active goal。
    pub goal: Option<ActiveGoal>,
    /// Goal incarnation associated with the pending planner path candidates.
    path_candidate_goal: Option<GoalLease>,
    /// Published planner paths awaiting scheduler confirmation.
    path_candidates: VecDeque<nuwax_nav_types_messages::PathKey>,
    /// 最近同目标的计算路径，仅用于展示，不能替代执行回执。
    display_path: Option<(GoalLease, Vec<[f64; 3]>)>,
    /// 已核验的停车终态和明确无租约拒绝，供取消重试和订阅重连读取。
    terminal_receipts: VecDeque<RetainedTerminalReceipt>,
    /// Highest goal generation ever reserved by this runtime.
    pub last_goal_generation: u64,
    /// module graph 模块节点 ID 列表(diagnostics 暴露)。
    pub module_nodes: Vec<String>,
    /// 当前 profile 字符串(env 解析后)。
    pub profile: &'static str,
}

impl NavRuntimeSlot {
    /// 构造默认 slot(`Idle` + simple-2d)。
    pub fn new_default(profile: &'static str) -> Self {
        Self {
            phase: NavPhase::Idle,
            message: "空闲".into(),
            goal: None,
            path_candidate_goal: None,
            path_candidates: VecDeque::new(),
            display_path: None,
            terminal_receipts: VecDeque::new(),
            last_goal_generation: 0,
            module_nodes: Vec::new(),
            profile,
        }
    }
}

/// Nav runtime 状态容器:由 `GeometryRuntimeService` 持有,与 LCM handle
/// 解耦;server.rs 通过 `Arc<NavRuntimeState>` 访问。
#[derive(Clone)]
pub struct NavRuntimeState {
    /// 启动时捕获的配置，在地图生命周期锁内读取也不获取导航状态锁。
    configured_profile: &'static str,
    inner: Arc<RwLock<NavRuntimeSlot>>,
    stack_observation: Arc<StdMutex<StackObservation>>,
    runtime_observations: Arc<StdMutex<RuntimeObservations>>,
    transaction_gate: Arc<tokio::sync::Mutex<()>>,
    publish_in_flight: Arc<StdMutex<Vec<PublishInFlight>>>,
    next_publish_attempt: Arc<AtomicU64>,
    #[cfg(test)]
    transaction_after_correlation: Arc<StdMutex<Option<Arc<tokio::sync::Barrier>>>>,
    #[cfg(test)]
    terminal_before_retention: Arc<StdMutex<Option<Arc<tokio::sync::Barrier>>>>,
    producer_runtime_generation: u64,
    next_goal_generation: Arc<AtomicU64>,
    next_cancel_generation: Arc<AtomicU64>,
}

// DRAFT ONLY. Insert at module scope in nav_runtime.rs.
// The whole addition is cfg(test). It reads the existing Vec under its original
// mutex; it cannot clear a fence, notify a waiter, replace a lease, or add state.
#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TestNavPublishKind {
    Goal,
}

#[cfg(test)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct TestNavPublishInFlight {
    pub kind: TestNavPublishKind,
    pub lease: GoalLease,
    pub generation: u64,
    pub attempt: u64,
}

#[cfg(test)]
impl NavRuntimeState {
    pub(crate) fn publish_in_flight_for_test(
        &self,
    ) -> Result<Vec<TestNavPublishInFlight>, &'static str> {
        let records = self
            .publish_in_flight
            .lock()
            .map_err(|_| "NAV_PUBLISH_STATE_UNAVAILABLE")?;
        Ok(records
            .iter()
            .map(|record| TestNavPublishInFlight {
                kind: match record.kind {
                    PublishKind::Goal => TestNavPublishKind::Goal,
                },
                lease: record.lease.clone(),
                generation: record.generation,
                attempt: record.attempt,
            })
            .collect())
    }
}

#[derive(Clone, Default)]
struct StackObservation {
    observed_at: Option<Instant>,
    state: Option<NavState>,
}

const DEFAULT_STACK_STATE_TTL: Duration = Duration::from_secs(2);
const DEFAULT_TF_TTL: Duration = Duration::ZERO;
const DEFAULT_PLANNER_INPUT_TTL: Duration = Duration::ZERO;
const DEFAULT_MOTION_SINK_TTL: Duration = Duration::from_secs(1);
const MAX_PENDING_PATH_CANDIDATES: usize = 4096;

/// One complete transform-chain observation produced by the real TF consumer.
///
/// The caller must describe the actual map -> odom -> base chain it consumed;
/// an odometry sample alone is not a TF observation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TfChainObservation {
    /// Producer generation. Zero is invalid; lower generations are rejected.
    pub generation: u64,
    /// Global map frame observed by the consumer.
    pub map_frame: String,
    /// Local odometry frame observed by the consumer.
    pub odom_frame: String,
    /// Robot base frame observed by the consumer.
    pub base_frame: String,
    /// Whether the consumer has a usable map -> odom transform.
    pub map_to_odom: bool,
    /// Whether the consumer has a usable odom -> base transform.
    pub odom_to_base: bool,
}

/// Planner-side input observation. This must be written by the component that
/// actually consumed the map and transformed pose, not by the map producer.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlannerInputObservation {
    /// Planner/scheduler generation. Zero is invalid.
    pub generation: u64,
    /// Occupancy revision actually observed by the planner.
    pub map_revision: u64,
    /// Frame carried by the planner's occupancy input.
    pub map_frame: String,
    /// Frame of the pose actually supplied to the planner.
    pub pose_frame: String,
    /// Whether the planner reports that this input set is usable.
    pub ready: bool,
}

/// Motion-sink health written by the guard-backed scheduler/client.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotionSinkObservation {
    /// Guard client/scheduler generation. Zero is invalid.
    pub generation: u64,
    /// Whether the private UDS execution chain is currently usable.
    pub ready: bool,
}

/// Typed observation update failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationError {
    /// Generation zero cannot identify a producer incarnation.
    InvalidGeneration,
    /// A delayed observation from an older producer incarnation was rejected.
    StaleGeneration {
        /// Current accepted generation.
        current: u64,
        /// Rejected generation.
        received: u64,
    },
    /// Internal observation storage was poisoned.
    StateUnavailable,
}

#[derive(Clone)]
struct TimedObservation<T> {
    value: Option<T>,
    observed_at: Option<Instant>,
}

impl<T> Default for TimedObservation<T> {
    fn default() -> Self {
        Self {
            value: None,
            observed_at: None,
        }
    }
}

#[derive(Clone, Default)]
struct RuntimeObservations {
    tf: TimedObservation<TfChainObservation>,
    planner: TimedObservation<PlannerInputObservation>,
    motion_sink: TimedObservation<MotionSinkObservation>,
    tf_count: u64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PublishKind {
    Goal,
}

#[derive(Clone)]
struct PublishInFlight {
    kind: PublishKind,
    lease: GoalLease,
    generation: u64,
    attempt: u64,
}

#[derive(Clone)]
pub(crate) enum GoalProducerEpochAuthority {
    PublicNavigation,
    MappingCloud,
    Wavefront {
        current_session_generation: Arc<AtomicU64>,
        boundary_owner: Arc<crate::explore_runtime::ExploreStartConfig>,
    },
}

#[derive(Default)]
struct RuntimeObservationSnapshot {
    tf_observed: bool,
    tf_fresh: bool,
    tf_complete: bool,
    tf_generation: u64,
    tf_count: u64,
    planner_observed: bool,
    planner_fresh: bool,
    planner_ready: bool,
    planner_generation: u64,
    map_revision: u64,
    map_frame: String,
    pose_frame: String,
    planner_frame_matches: bool,
    motion_sink_observed: bool,
    motion_sink_fresh: bool,
    motion_sink_ready: bool,
    motion_sink_generation: u64,
}

impl NavRuntimeState {
    #[cfg(test)]
    /// 新建 nav runtime:读 env profile、build module graph、把节点 ID
    /// 写入 slot。
    pub fn from_env() -> Self {
        let profile = resolve_profile_from_env().expect("有效的建图探索导航执行链");
        let producer_runtime_generation = NEXT_NAV_RUNTIME_GENERATION
            .fetch_update(AtomicOrdering::AcqRel, AtomicOrdering::Acquire, |current| {
                current
                    .checked_add(1)
                    .filter(|next| *next <= i64::MAX as u64)
            })
            .map(|previous| previous + 1)
            .expect("process-local test producer generation exhausted");
        Self::with_profile_and_producer_runtime_generation(profile, producer_runtime_generation)
    }

    #[cfg(test)]
    /// Construct the production runtime only after durably reserving a producer
    /// epoch that remains newer after a geometry process restart.
    pub(crate) fn try_from_sequence_store(
        sequence_store: &SequenceStateStore,
    ) -> Result<Self, String> {
        let pipeline = nuwax_nav_stack::ProductionPipeline::from_env()
            .map_err(|error| error.to_string())?;
        Self::try_from_sequence_store_for_pipeline(sequence_store, pipeline)
            .map_err(|error| error.to_string())
    }

    /// 使用启动时已捕获的固定配对，分配代次后不重读环境。
    pub(crate) fn try_from_sequence_store_for_pipeline(
        sequence_store: &SequenceStateStore,
        pipeline: nuwax_nav_stack::ProductionPipeline,
    ) -> Result<Self, SequenceStateError> {
        let producer_runtime_generation = sequence_store.reserve_producer_runtime_generation()?;
        Ok(Self::with_profile_and_producer_runtime_generation(
            pipeline.navigation_profile().as_str(),
            producer_runtime_generation,
        ))
    }

    fn with_profile_and_producer_runtime_generation(
        profile: &'static str,
        producer_runtime_generation: u64,
    ) -> Self {
        let module_nodes = match NavStackBuilder::new(profile_enum(profile)).build() {
            Ok(graph) => graph
                .nodes()
                .iter()
                .map(|n: &ModuleNode| n.id.clone())
                .collect::<Vec<_>>(),
            Err(e) => {
                warn!(
                    "NavStackBuilder::build 失败 (profile={}): {}; module_nodes 留空",
                    profile, e,
                );
                Vec::new()
            }
        };
        info!(
            "NavRuntimeState 初始化: profile={} module_nodes={}",
            profile,
            module_nodes.len()
        );
        let mut slot = NavRuntimeSlot::new_default(profile);
        slot.module_nodes = module_nodes;
        Self {
            configured_profile: profile,
            inner: Arc::new(RwLock::new(slot)),
            stack_observation: Arc::new(StdMutex::new(StackObservation::default())),
            runtime_observations: Arc::new(StdMutex::new(RuntimeObservations::default())),
            transaction_gate: Arc::new(tokio::sync::Mutex::new(())),
            publish_in_flight: Arc::new(StdMutex::new(Vec::new())),
            next_publish_attempt: Arc::new(AtomicU64::new(0)),
            #[cfg(test)]
            transaction_after_correlation: Arc::new(StdMutex::new(None)),
            #[cfg(test)]
            terminal_before_retention: Arc::new(StdMutex::new(None)),
            producer_runtime_generation,
            next_goal_generation: Arc::new(AtomicU64::new(0)),
            next_cancel_generation: Arc::new(AtomicU64::new(0)),
        }
    }

    #[cfg(test)]
    pub(crate) fn inject_transaction_after_correlation_barrier(
        &self,
        barrier: Arc<tokio::sync::Barrier>,
    ) {
        *self
            .transaction_after_correlation
            .lock()
            .expect("transaction correlation barrier") = Some(barrier);
    }

    #[cfg(test)]
    pub(crate) fn inject_terminal_before_retention_barrier(&self, barrier: Arc<tokio::sync::Barrier>) {
        *self.terminal_before_retention.lock().expect("terminal retention barrier") = Some(barrier);
    }

    #[cfg(test)]
    pub(crate) async fn pause_terminal_before_retention_for_test(&self) {
        let barrier = self.terminal_before_retention.lock().expect("terminal retention barrier").take();
        if let Some(barrier) = barrier {
            barrier.wait().await;
            barrier.wait().await;
        }
    }

    #[cfg(test)]
    pub(crate) async fn project_late_goal_completion_for_test(&self, lease: &GoalLease) -> SetGoalOutcome {
        let _transaction_guard = self.transaction_gate.lock().await;
        project_installed_goal_completion(self, lease, 0.0,
            GoalDispatchEffect::EffectUnconfirmed, "test-only late generic backend error".into(), false).await
    }

    #[cfg(test)]
    pub(crate) async fn replace_active_goal_for_test(&self, goal: ActiveGoal) {
        let _transaction_guard = self.transaction_gate.lock().await;
        let mut slot = self.inner.write().await;
        slot.phase = NavPhase::Running;
        slot.goal = Some(goal);
    }

    #[cfg(test)]
    /// 读内部 slot 句柄(给 server.rs 写 diagnostics 用)。
    pub fn inner(&self) -> Arc<RwLock<NavRuntimeSlot>> {
        self.inner.clone()
    }

    /// 收敛接收边界已解码的明确无租约拒绝；它不生成停车证明。
    pub(crate) async fn retire_unleased_goal(
        &self, mut update: NavigationUpdate,
    ) -> Option<NavigationUpdate> {
        let _transaction_guard = self.transaction_gate.lock().await;
        let mut slot = self.inner.write().await;
        let active = slot.goal.as_ref()?;
        let lease = &active.lease;
        if lease.session_id != update.state.goal_id
            || lease.goal_generation != update.feedback.goal_generation
            || lease.producer_runtime_generation != update.feedback.producer_runtime_generation
            || matches!(slot.phase, NavPhase::Running | NavPhase::Succeeded)
        {
            return None;
        }
        update.producer_runtime_generation = lease.producer_runtime_generation;
        update.feedback.cancel_phase = "IDLE".into();
        update.feedback.actuator_zeroed = false;
        slot.phase = NavPhase::Failed;
        slot.message = update.state.message.clone();
        slot.goal = None;
        slot.path_candidate_goal = None;
        slot.path_candidates.clear();
        slot.display_path = None;
        slot.terminal_receipts.push_back(RetainedTerminalReceipt {
            update: update.clone(),
            retired_without_child_cancel: true,
        });
        while slot.terminal_receipts.len() > 32 { slot.terminal_receipts.pop_front(); }
        drop(slot);
        self.observe_unowned_stack_heartbeat(update.state.clone());
        Some(update)
    }

    /// 只保留已经沿执行输入校验的停车证明，不从本地错误构造终态。
    pub(crate) async fn retain_terminal_receipt(&self, update: &NavigationUpdate) {
        if !matches!(update.state.status.as_str(), "SUCCEEDED" | "FAILED" | "CANCELLED")
            || update.state.goal_id.is_empty() || update.feedback.goal_generation == 0
            || update.producer_runtime_generation == 0 {
            return;
        }
        let _transaction_guard = self.transaction_gate.lock().await;
        let mut slot=self.inner.write().await;
        // 失败先保留原拥有方；仅同目标、同路径的真实停稳证明才能退休它。
        // 状态关联后若已切换目标或规划器代际，旧证明不能清掉新的拥有方。
        let retires_active = slot.goal.as_ref().is_some_and(|active| {
            let lease = &active.lease;
            lease.session_id == update.state.goal_id
                && lease.owner_kind == update.feedback.owner_kind
                && lease.parent_request_id == update.feedback.parent_request_id
                && lease.goal_generation == update.feedback.goal_generation
                && lease.producer_runtime_generation == update.producer_runtime_generation
                && lease.path_revision == update.feedback.path_revision
                && lease.planner_runtime_generation == update.feedback.planner_runtime_generation
        });
        let retired_without_child_cancel = retires_active && update.feedback.cancel_generation == 0;
        if retires_active {
            slot.goal = None;
        }
        if let Some(index) = slot.terminal_receipts.iter().position(|old| {
            old.update.state.goal_id == update.state.goal_id
                && old.update.feedback.owner_kind == update.feedback.owner_kind
                && old.update.feedback.parent_request_id == update.feedback.parent_request_id
                && old.update.feedback.goal_generation == update.feedback.goal_generation
                && old.update.producer_runtime_generation == update.producer_runtime_generation
        }) {
            // A late old-path body must not replace an authoritative retirement,
            // nor inherit its fact. Keep the original body and fact together.
            if slot.terminal_receipts[index].retired_without_child_cancel {
                return;
            }
            slot.terminal_receipts.remove(index);
        }
        slot.terminal_receipts.push_back(RetainedTerminalReceipt {
            update: update.clone(),
            retired_without_child_cancel,
        });
        while slot.terminal_receipts.len() > 32 { slot.terminal_receipts.pop_front(); }
    }

    /// 订阅恢复只重放已验证的有界终态；客户端继续按完整目标身份消费。
    pub(crate) async fn terminal_receipts(&self) -> Vec<NavigationUpdate> {
        self.inner.read().await.terminal_receipts.iter().map(|receipt| receipt.update.clone()).collect()
    }

    /// 旧目标的精确取消重试只读取原停车回执，不触碰活动的新目标。
    pub(crate) async fn terminal_receipt_for(&self, input: &CancelGoalInput) -> Option<NavigationUpdate> {
        self.inner.read().await.terminal_receipts.iter().rev()
            .find(|receipt| receipt.matches(input)).map(|receipt| receipt.update.clone())
    }

    /// 读取已实际退休的原自然终态、明确无租约结果或普通停车结果。
    /// 旧路径回执及当前目标缺席均不能自行获得此资格。
    pub(crate) async fn settled_without_child_cancel_terminal_for(&self, input: &CancelGoalInput) -> Option<NavigationUpdate> {
        self.inner.read().await.terminal_receipts.iter().rev()
            .find(|receipt| receipt.retired_without_child_cancel && receipt.matches(input))
            .map(|receipt| receipt.update.clone())
    }

    /// 当前拥有方整个生命周期内保持不变的生产代次。
    pub fn producer_runtime_generation(&self) -> u64 {
        self.producer_runtime_generation
    }

    /// 不持状态锁读取已捕获配置，用于规范地图发布等已有生命周期围栏内的接线。
    pub fn configured_profile(&self) -> &'static str {
        self.configured_profile
    }

    fn reserve_publish_attempt(&self) -> Result<u64, ()> {
        self.next_publish_attempt
            .fetch_update(AtomicOrdering::AcqRel, AtomicOrdering::Acquire, |current| {
                current.checked_add(1)
            })
            .map(|previous| previous + 1)
            .map_err(|_| ())
    }

    fn begin_publish(
        &self,
        kind: PublishKind,
        lease: GoalLease,
        generation: u64,
    ) -> Result<u64, &'static str> {
        let mut in_flight = self
            .publish_in_flight
            .lock()
            .map_err(|_| "NAV_PUBLISH_STATE_UNAVAILABLE")?;
        if kind == PublishKind::Goal && !in_flight.is_empty() {
            return Err("NAV_PUBLISH_IN_FLIGHT_CONFLICT");
        }
        let attempt = self
            .reserve_publish_attempt()
            .map_err(|_| "NAV_PUBLISH_ATTEMPT_EXHAUSTED")?;
        in_flight.push(PublishInFlight {
            kind,
            lease,
            generation,
            attempt,
        });
        Ok(attempt)
    }

    fn finish_publish(
        &self,
        kind: PublishKind,
        lease: &GoalLease,
        generation: u64,
        attempt: u64,
    ) -> bool {
        let Ok(mut in_flight) = self.publish_in_flight.lock() else {
            return false;
        };
        let Some(index) = in_flight.iter().position(|current| {
            current.kind == kind
                && current.lease == *lease
                && current.generation == generation
                && current.attempt == attempt
        }) else {
            return false;
        };
        in_flight.remove(index);
        drop(in_flight);
        true
    }

    fn reserve_goal_generation(&self, requested: u64) -> Result<u64, ()> {
        if requested > 0 {
            if requested > i64::MAX as u64 {
                return Err(());
            }
            self.next_goal_generation
                .fetch_max(requested, AtomicOrdering::AcqRel);
            return Ok(requested);
        }
        self.next_goal_generation
            .fetch_update(AtomicOrdering::AcqRel, AtomicOrdering::Acquire, |current| {
                current
                    .checked_add(1)
                    .filter(|next| *next <= i64::MAX as u64)
            })
            .map(|previous| previous + 1)
            .map_err(|_| ())
    }

    /// Snapshot the unique active goal lease, if one exists.
    pub async fn active_goal_lease(&self) -> Option<GoalLease> {
        self.inner
            .read()
            .await
            .goal
            .as_ref()
            .map(|goal| goal.lease.clone())
    }

    /// 查询同一目标身份的完整空间目标；真正状态提交仍须通过原租约校验。
    pub(crate) async fn spatial_feedback_target(
        &self,
        session_id: &str,
        owner_kind: i32,
        parent_request_id: &str,
        goal_generation: u64,
        producer_runtime_generation: u64,
    ) -> Option<PoseState> {
        if self.configured_profile != PROFILE_MLS3D {
            return None;
        }
        let slot = self.inner.read().await;
        let goal = slot.goal.as_ref()?;
        let lease = &goal.lease;
        (lease.session_id == session_id
            && lease.owner_kind == owner_kind
            && lease.parent_request_id == parent_request_id
            && lease.goal_generation == goal_generation
            && lease.producer_runtime_generation == producer_runtime_generation)
            .then(|| goal.target.clone())
    }

    /// Validate and retain planner output without replacing a path already
    /// confirmed by the scheduler.
    pub async fn observe_path_envelope(
        &self,
        path: &schema::PathEnvelope,
    ) -> Result<GoalLease, String> {
        let _transaction_guard = self.transaction_gate.lock().await;
        let key = path.path_key();
        let mut slot = self.inner.write().await;
        let confirmed = slot.goal.as_ref().map(|active| active.lease.clone())
            .ok_or_else(|| "PATH_LEASE_REJECTED: no active goal lease".to_string())?;
        if path.session_id != confirmed.session_id
            || key.goal.goal_generation != confirmed.goal_generation
            || key.goal.producer_runtime_generation != confirmed.producer_runtime_generation
        {
            return Err("PATH_LEASE_REJECTED: assigned goal does not match active lease".to_string());
        }
        if !slot.path_candidate_goal.as_ref().is_some_and(|goal| goal.identity_matches(&confirmed)) {
            slot.path_candidate_goal = Some(confirmed.clone());
            slot.path_candidates.clear();
        }
        // Retain the pre-existing per-goal revision watermark across planner
        // incarnations. A new planner value is not itself execution authority.
        let latest_revision = slot.path_candidates.back().map_or(confirmed.path_revision, |candidate| candidate.path_revision);
        if key.path_revision < latest_revision {
            return Err("PATH_LEASE_REJECTED: stale path revision".to_string());
        }
        let mut observed = confirmed;
        observed.path_revision = key.path_revision;
        observed.planner_runtime_generation = key.planner_runtime_generation;
        if slot.path_candidates.contains(&key)
            || (observed.planner_runtime_generation == slot.goal.as_ref().unwrap().lease.planner_runtime_generation
                && observed.path_revision == slot.goal.as_ref().unwrap().lease.path_revision)
        {
            // Geometry only correlates display candidates. The scheduler owns
            // execution-content mutation detection at the same assigned key.
            return Ok(observed);
        }
        if key.path_revision == latest_revision {
            return Err("PATH_LEASE_REJECTED: conflicting planner identity at same revision".to_string());
        }
        if slot.path_candidates.len() == MAX_PENDING_PATH_CANDIDATES {
            slot.path_candidates.pop_front();
        }
        slot.path_candidates.push_back(key);
        slot.display_path = Some((observed.clone(), path.path.poses.iter().map(|p| p.pose.position).collect()));
        Ok(observed)
    }

    /// 仅返回仍属于本次目标代次的已计算路线；终态由调用者清空。
    pub async fn display_path_for(&self, lease: &GoalLease, include_confirmed:bool) -> Vec<[f64; 3]> {
        let slot = self.inner.read().await;
        slot.display_path.as_ref().filter(|(owner, _)|
            owner.identity_matches(lease)
            && (include_confirmed || owner.path_revision>lease.path_revision)
            && owner.producer_runtime_generation == lease.producer_runtime_generation
        ).map(|(_, points)| points.clone()).unwrap_or_default()
    }

    #[cfg(test)]
    /// Correlate a typed NavigationState with either the active or retained
    /// cancellation lease. Empty-session heartbeats are accepted as unowned.
    #[allow(clippy::too_many_arguments)]
    pub async fn correlate_navigation_state(
        &self,
        session_id: &str,
        owner_kind: i32,
        parent_request_id: &str,
        goal_generation: u64,
        path_revision: u64,
        producer_runtime_generation: u64,
        planner_runtime_generation: u64,
    ) -> Result<Option<GoalLease>, String> {
        let _transaction_guard = self.transaction_gate.lock().await;
        self.correlate_navigation_state_inner(
            session_id,
            owner_kind,
            parent_request_id,
            goal_generation,
            path_revision,
            producer_runtime_generation,
            planner_runtime_generation,
        )
        .await
    }

    #[allow(clippy::too_many_arguments)]
    async fn correlate_navigation_state_inner(
        &self,
        session_id: &str,
        owner_kind: i32,
        parent_request_id: &str,
        goal_generation: u64,
        path_revision: u64,
        producer_runtime_generation: u64,
        planner_runtime_generation: u64,
    ) -> Result<Option<GoalLease>, String> {
        if session_id.is_empty() {
            return Ok(None);
        }
        let mismatched_fields = |lease: &GoalLease| {
            let mut fields = Vec::new();
            if session_id != lease.session_id {
                fields.push("session_id");
            }
            if owner_kind != lease.owner_kind {
                fields.push("owner_kind");
            }
            if parent_request_id != lease.parent_request_id {
                fields.push("parent_request_id");
            }
            if goal_generation != lease.goal_generation {
                fields.push("goal_generation");
            }
            if producer_runtime_generation != lease.producer_runtime_generation {
                fields.push("producer_runtime_generation");
            }
            if path_revision != lease.path_revision {
                fields.push("path_revision");
            }
            if planner_runtime_generation != lease.planner_runtime_generation {
                fields.push("planner_runtime_generation");
            }
            fields
        };
        let rejection = |lease: &GoalLease| {
            format!(
                "NAV_STATE_LEASE_REJECTED: mismatched fields: {}",
                mismatched_fields(lease).join(",")
            )
        };

        let mut slot = self.inner.write().await;
        let Some(confirmed) = slot.goal.as_ref().map(|active| active.lease.clone()) else {
            drop(slot);
            return Err("NAV_STATE_LEASE_REJECTED: no matching lease".into());
        };
        let goal_identity_matches = session_id == confirmed.session_id
            && owner_kind == confirmed.owner_kind
            && parent_request_id == confirmed.parent_request_id
            && goal_generation == confirmed.goal_generation
            && producer_runtime_generation == confirmed.producer_runtime_generation;
        let confirmed_path_matches = path_revision == confirmed.path_revision
            && planner_runtime_generation == confirmed.planner_runtime_generation;
        if goal_identity_matches && confirmed_path_matches {
            return Ok(Some(confirmed));
        }
        if !goal_identity_matches {
            return Err(rejection(&confirmed));
        }

            let candidate_goal_matches =
                slot.path_candidate_goal
                    .as_ref()
                    .is_some_and(|candidate_goal| {
                        candidate_goal.identity_matches(&confirmed)
                            && candidate_goal.producer_runtime_generation
                                == confirmed.producer_runtime_generation
                    });
            let candidate_matches = candidate_goal_matches
                && slot.path_candidates.iter().any(|key| {
                    key.path_revision == path_revision && key.planner_runtime_generation == planner_runtime_generation
                });
            if candidate_matches {
                let mut promoted = confirmed;
                promoted.path_revision = path_revision;
                promoted.planner_runtime_generation = planner_runtime_generation;
                slot.goal.as_mut().expect("active goal checked above").lease = promoted.clone();
                while slot
                    .path_candidates
                    .front()
                    .is_some_and(|key| key.path_revision <= path_revision)
                {
                    slot.path_candidates.pop_front();
                }
                return Ok(Some(promoted));
            }

        Err(rejection(&confirmed))
    }

    /// Apply one owned typed NavigationState as a single FIFO transaction.
    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn apply_correlated_navigation_state(
        &self,
        observed: NavState,
        session_id: &str,
        owner_kind: i32,
        parent_request_id: &str,
        goal_generation: u64,
        path_revision: u64,
        producer_runtime_generation: u64,
        planner_runtime_generation: u64,
        planner_observation: Option<PlannerInputObservation>,
        motion_sink_observation: Option<MotionSinkObservation>,
    ) -> Result<(NavState, GoalLease), String> {
        let _transaction_guard = self.transaction_gate.lock().await;
        let lease = self
            .correlate_navigation_state_inner(
                session_id,
                owner_kind,
                parent_request_id,
                goal_generation,
                path_revision,
                producer_runtime_generation,
                planner_runtime_generation,
            )
            .await?
            .ok_or_else(|| {
                "NAV_STATE_LEASE_REJECTED: owned navigation state has no lease".to_string()
            })?;
        #[cfg(test)]
        {
            let barrier = self
                .transaction_after_correlation
                .lock()
                .expect("transaction correlation barrier")
                .take();
            if let Some(barrier) = barrier {
                barrier.wait().await;
                barrier.wait().await;
            }
        }
        let state = self
            .observe_stack_state_inner(
                observed,
                Some(&lease),
                planner_observation,
                motion_sink_observation,
            )
            .await?;
        Ok((state, lease))
    }

    /// 当前 profile(用于 diagnostics 暴露)。
    pub async fn profile(&self) -> &'static str {
        self.inner.read().await.profile
    }

    /// 当前 module graph 节点列表(diagnostics 用)。
    pub async fn module_nodes(&self) -> Vec<String> {
        self.inner.read().await.module_nodes.clone()
    }

    #[cfg(test)]
    /// Record one state emitted by the real stack and normalize an omitted
    /// session id to the facade's currently active goal.
    pub async fn observe_stack_state(&self, observed: NavState) -> NavState {
        let _transaction_guard = self.transaction_gate.lock().await;
        self.observe_stack_state_inner(observed, None, None, None)
            .await
            .expect("an unconstrained stack observation cannot be superseded")
    }

    /// Record an unowned scheduler heartbeat for liveness only. It must never
    /// be projected onto the active goal slot or cancellation transaction.
    pub(crate) fn observe_unowned_stack_heartbeat(&self, observed: NavState) -> NavState {
        if let Ok(mut stack) = self.stack_observation.lock() {
            stack.observed_at = Some(Instant::now());
            stack.state = Some(observed.clone());
        }
        observed
    }

    async fn observe_stack_state_inner(
        &self,
        mut observed: NavState,
        expected_lease: Option<&GoalLease>,
        planner_observation: Option<PlannerInputObservation>,
        motion_sink_observation: Option<MotionSinkObservation>,
    ) -> Result<NavState, String> {
        let previous_state = self
            .stack_observation
            .lock()
            .ok()
            .and_then(|stack| stack.state.clone());
        let observed_phase = NavPhase::from_stack_status(&observed.status);
        {
            let mut slot = self.inner.write().await;
            if expected_lease.is_some_and(|expected| {
                !slot
                    .goal
                    .as_ref()
                    .is_some_and(|active| active.lease == *expected)
            }) {
                return Err(
                    "NAV_STATE_LEASE_REJECTED: correlated active incarnation was superseded"
                        .to_string(),
                );
            }
            if let Some(observation) = planner_observation {
                if let Err(error) = self.observe_planner_input(observation) {
                    warn!(?error, "typed planner observation rejected");
                }
            }
            if let Some(observation) = motion_sink_observation {
                if let Err(error) = self.observe_motion_sink(observation) {
                    warn!(?error, "typed motion-sink observation rejected");
                }
            }
            if observed.goal_id.is_empty() {
                if let Some(goal) = slot.goal.as_ref() {
                    observed.goal_id = goal.id.clone();
                } else if let Some(previous) = previous_state.as_ref().filter(|previous| {
                    !previous.goal_id.is_empty()
                        && previous.status.eq_ignore_ascii_case(&observed.status)
                }) {
                    observed.goal_id = previous.goal_id.clone();
                }
            }
            if let Some(phase) = observed_phase {
                slot.phase = phase;
                slot.message = observed.message.clone();
                if phase.is_terminal() && expected_lease.is_none() {
                    slot.goal = None;
                }
            }
        }
        if let Ok(mut stack) = self.stack_observation.lock() {
            stack.observed_at = Some(Instant::now());
            stack.state = Some(observed.clone());
        }
        Ok(observed)
    }

    fn stack_health_snapshot(&self) -> StackHealthSnapshot {
        let Ok(stack) = self.stack_observation.lock() else {
            return StackHealthSnapshot::default();
        };
        let Some(observed_at) = stack.observed_at else {
            return StackHealthSnapshot::default();
        };
        let age = observed_at.elapsed();
        let status = stack
            .state
            .as_ref()
            .map(|state| state.status.clone())
            .unwrap_or_default();
        StackHealthSnapshot {
            observed: true,
            fresh: age <= stack_state_ttl(),
            age_ms: age.as_millis().min(u64::MAX as u128) as u64,
            status,
        }
    }

    /// Record a real TF consumer observation.
    pub fn observe_tf_chain(
        &self,
        observation: TfChainObservation,
    ) -> Result<(), ObservationError> {
        validate_generation(observation.generation)?;
        let mut runtime = self
            .runtime_observations
            .lock()
            .map_err(|_| ObservationError::StateUnavailable)?;
        reject_stale_generation(
            runtime.tf.value.as_ref().map(|value| value.generation),
            observation.generation,
        )?;
        runtime.tf.value = Some(observation);
        runtime.tf.observed_at = Some(Instant::now());
        runtime.tf_count = runtime.tf_count.saturating_add(1);
        Ok(())
    }

    /// Record the map revision and transformed pose actually consumed by the planner.
    pub fn observe_planner_input(
        &self,
        observation: PlannerInputObservation,
    ) -> Result<(), ObservationError> {
        validate_generation(observation.generation)?;
        let mut runtime = self
            .runtime_observations
            .lock()
            .map_err(|_| ObservationError::StateUnavailable)?;
        reject_stale_generation(
            runtime.planner.value.as_ref().map(|value| value.generation),
            observation.generation,
        )?;
        runtime.planner.value = Some(observation);
        runtime.planner.observed_at = Some(Instant::now());
        Ok(())
    }

    /// Record guard-backed motion-sink health. A bare LCM publisher is not a sink.
    pub fn observe_motion_sink(
        &self,
        observation: MotionSinkObservation,
    ) -> Result<(), ObservationError> {
        validate_generation(observation.generation)?;
        let mut runtime = self
            .runtime_observations
            .lock()
            .map_err(|_| ObservationError::StateUnavailable)?;
        reject_stale_generation(
            runtime
                .motion_sink
                .value
                .as_ref()
                .map(|value| value.generation),
            observation.generation,
        )?;
        runtime.motion_sink.value = Some(observation);
        runtime.motion_sink.observed_at = Some(Instant::now());
        Ok(())
    }

    #[cfg(test)]
    /// 测试注入无运动观察，验证诊断缺失本身不会阻断有效导航输入。
    pub(crate) fn observe_idle_motion_sink(&self, ready: bool) -> Result<(), ObservationError> {
        let mut runtime = self
            .runtime_observations
            .lock()
            .map_err(|_| ObservationError::StateUnavailable)?;
        runtime.motion_sink.value = Some(MotionSinkObservation {
            generation: 0,
            ready,
        });
        runtime.motion_sink.observed_at = Some(Instant::now());
        Ok(())
    }

    /// 进程诊断复用实际观察分类，只报告本次缺失或过期的条件。
    pub(crate) fn execution_chain_blockers(&self) -> Vec<NavigationReadinessCode> {
        let observations = self.runtime_observation_snapshot();
        let stack = self.stack_health_snapshot();
        let profile = self.inner.try_read().ok().map(|slot| profile_has_guarded_execution(slot.profile));
        let mut blockers = observation_blockers(profile.unwrap_or(true), &stack, &observations);
        if profile.is_none() {
            // 配置正在更新时保持原不就绪语义，不把锁忙误报为不支持的配置。
            blockers.push(NavigationReadinessCode::Unknown);
        }
        blockers
    }

    /// 同步诊断与实际缺失项使用同一份观察判断。
    #[cfg(test)]
    pub fn execution_chain_is_live(&self) -> bool {
        self.execution_chain_blockers().is_empty()
    }

    pub fn cancel_generation(&self) -> u64 {
        self.next_cancel_generation.load(AtomicOrdering::Acquire)
    }

    fn runtime_observation_snapshot(&self) -> RuntimeObservationSnapshot {
        let Ok(runtime) = self.runtime_observations.lock() else {
            return RuntimeObservationSnapshot::default();
        };
        let mut snapshot = RuntimeObservationSnapshot {
            tf_count: runtime.tf_count,
            ..RuntimeObservationSnapshot::default()
        };
        if let Some(tf) = runtime.tf.value.as_ref() {
            snapshot.tf_observed = true;
            snapshot.tf_fresh = observation_is_fresh(runtime.tf.observed_at, tf_ttl());
            snapshot.tf_complete = tf.map_to_odom
                && tf.odom_to_base
                && !tf.map_frame.trim().is_empty()
                && !tf.odom_frame.trim().is_empty()
                && !tf.base_frame.trim().is_empty()
                && tf.map_frame != tf.odom_frame
                && tf.odom_frame != tf.base_frame;
            snapshot.tf_generation = tf.generation;
        }
        if let Some(planner) = runtime.planner.value.as_ref() {
            snapshot.planner_observed = true;
            snapshot.planner_fresh =
                observation_is_fresh(runtime.planner.observed_at, planner_input_ttl());
            snapshot.planner_ready = planner.ready;
            snapshot.planner_generation = planner.generation;
            snapshot.map_revision = planner.map_revision;
            snapshot.map_frame = planner.map_frame.clone();
            snapshot.pose_frame = planner.pose_frame.clone();
            snapshot.planner_frame_matches =
                !planner.map_frame.trim().is_empty() && planner.map_frame == planner.pose_frame;
        }
        if let Some(sink) = runtime.motion_sink.value.as_ref() {
            snapshot.motion_sink_observed = true;
            snapshot.motion_sink_fresh =
                observation_is_fresh(runtime.motion_sink.observed_at, motion_sink_ttl());
            snapshot.motion_sink_ready = sink.ready;
            snapshot.motion_sink_generation = sink.generation;
        }
        snapshot
    }

    #[cfg(test)]
    fn expire_stack_observation_for_test(&self) {
        if let Ok(mut stack) = self.stack_observation.lock() {
            stack.observed_at =
                Instant::now().checked_sub(stack_state_ttl() + Duration::from_millis(1));
        }
    }

    #[cfg(test)]
    pub(crate) fn expire_tf_for_test(&self) {
        // 只让当前测试实例的坐标变换观察过期，不修改进程共享的TTL配置。
        if let Ok(mut runtime) = self.runtime_observations.lock() {
            runtime.tf.observed_at =
                Instant::now().checked_sub(tf_ttl() + Duration::from_millis(1));
        }
    }

    #[cfg(test)]
    pub(crate) fn clear_motion_sink_for_test(&self) {
        // 单独模拟未收到执行端观察；零代际仍是合法的空闲状态。
        if let Ok(mut runtime) = self.runtime_observations.lock() {
            runtime.motion_sink = TimedObservation::default();
        }
    }

    #[cfg(test)]
    fn expire_motion_sink_for_test(&self) {
        if let Ok(mut runtime) = self.runtime_observations.lock() {
            runtime.motion_sink.observed_at =
                Instant::now().checked_sub(motion_sink_ttl() + Duration::from_millis(1));
        }
    }
}

#[derive(Default)]
struct StackHealthSnapshot {
    observed: bool,
    fresh: bool,
    age_ms: u64,
    status: String,
}

fn stack_state_ttl() -> Duration {
    std::env::var("NUWAX_NAV_STACK_STATE_TTL_MS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .map(Duration::from_millis)
        .unwrap_or(DEFAULT_STACK_STATE_TTL)
}

fn tf_ttl() -> Duration {
    duration_from_env("NUWAX_NAV_TF_TTL_MS", DEFAULT_TF_TTL)
}

fn planner_input_ttl() -> Duration {
    duration_from_env("NUWAX_NAV_PLANNER_INPUT_TTL_MS", DEFAULT_PLANNER_INPUT_TTL)
}

fn motion_sink_ttl() -> Duration {
    duration_from_env("NUWAX_NAV_MOTION_SINK_TTL_MS", DEFAULT_MOTION_SINK_TTL)
}

fn duration_from_env(name: &str, default: Duration) -> Duration {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .map(Duration::from_millis)
        .unwrap_or(default)
}

fn observation_is_fresh(observed_at: Option<Instant>, ttl: Duration) -> bool {
    observed_at.is_some_and(|observed_at| ttl.is_zero() || observed_at.elapsed() <= ttl)
}

fn validate_generation(generation: u64) -> Result<(), ObservationError> {
    if generation == 0 {
        Err(ObservationError::InvalidGeneration)
    } else {
        Ok(())
    }
}

fn reject_stale_generation(current: Option<u64>, received: u64) -> Result<(), ObservationError> {
    if let Some(current) = current.filter(|current| received < *current) {
        Err(ObservationError::StaleGeneration { current, received })
    } else {
        Ok(())
    }
}

#[cfg(test)]
/// 解析固定执行链；显式错配或非法值保留错误。
pub fn resolve_profile_from_env() -> Result<&'static str, nuwax_nav_stack::NavStackError> {
    nuwax_nav_stack::ProductionPipeline::from_env()
        .map(|pipeline| pipeline.navigation_profile().as_str())
}

/// 把 profile 字符串映射到 `NavStackProfile` enum(用于 `NavStackBuilder::new`)。
fn profile_enum(profile: &str) -> NavStackProfile {
    match profile {
        PROFILE_SIMPLE_2D => NavStackProfile::Simple2D,
        PROFILE_REPLANNING_ASTAR => NavStackProfile::ReplanningAStar,
        PROFILE_CMU_NATIVE => NavStackProfile::CmuNative,
        PROFILE_EXPLORATION_NATIVE => NavStackProfile::ExplorationNative,
        PROFILE_MLS3D => NavStackProfile::Mls3D,
        _ => NavStackProfile::Simple2D,
    }
}

#[cfg(test)]
/// 启动时构造 module graph,返回 `NavStackHandle` 给 diagnostics 渲染用。
///
/// 失败时返回 `None`,调用方按 not_ready 处理。
pub fn build_nav_stack_handle(profile: &str) -> Option<NavStackHandle> {
    let builder = NavStackBuilder::new(profile_enum(profile)).preset(NavStackPreset::None);
    match builder.build_handle() {
        Ok(h) => Some(h),
        Err(e) => {
            warn!("build_nav_stack_handle 失败: {}", e);
            None
        }
    }
}

#[cfg(test)]
/// 把 `NavStackHandle` 渲染成 dry-run 字符串(给 diagnostics 字段用)。
pub fn render_dry_run(handle: &NavStackHandle) -> String {
    handle.dry_run()
}

/// Stable machine-readable readiness result code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum NavigationReadinessCode {
    /// 配置锁忙时无法取得当前就绪诊断。
    Unknown,
    /// Every required observation is fresh and usable.
    Ready,
    /// The real LCM backend is unavailable.
    LcmBackendUnavailable,
    /// No real nav-stack state heartbeat has been observed.
    StackStateUnobserved,
    /// The nav-stack heartbeat exceeded its freshness deadline.
    StackStateStale,
    /// No TF consumer has reported a chain.
    TfUnobserved,
    /// The TF observation exceeded its freshness deadline.
    TfStale,
    /// The observed map -> odom -> base chain is incomplete.
    TfIncomplete,
    /// The real planner consumer has not reported an input set.
    PlannerInputUnobserved,
    /// The planner input observation exceeded its freshness deadline.
    PlannerInputStale,
    /// Planner map and transformed-pose frames do not match.
    PlannerFrameMismatch,
    /// The planner has not observed a non-zero occupancy revision.
    CostmapUnavailable,
    /// The planner explicitly reported that its observed input is unusable.
    PlannerNotReady,
    /// The selected profile is not qualified for the production path.
    ProfileUnsupported,
    /// A goal frame does not match the planner's observed map frame.
    GoalFrameMismatch,
}

impl NavigationReadinessCode {
    /// Stable ASCII code used by compatibility projections and logs.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unknown => "UNKNOWN",
            Self::Ready => "READY",
            Self::LcmBackendUnavailable => "LCM_BACKEND_UNAVAILABLE",
            Self::StackStateUnobserved => "STACK_STATE_UNOBSERVED",
            Self::StackStateStale => "STACK_STATE_STALE",
            Self::TfUnobserved => "TF_UNOBSERVED",
            Self::TfStale => "TF_STALE",
            Self::TfIncomplete => "TF_INCOMPLETE",
            Self::PlannerInputUnobserved => "PLANNER_INPUT_UNOBSERVED",
            Self::PlannerInputStale => "PLANNER_INPUT_STALE",
            Self::PlannerFrameMismatch => "PLANNER_FRAME_MISMATCH",
            Self::CostmapUnavailable => "COSTMAP_UNAVAILABLE",
            Self::PlannerNotReady => "PLANNER_NOT_READY",
            Self::ProfileUnsupported => "PROFILE_UNSUPPORTED",
            Self::GoalFrameMismatch => "GOAL_FRAME_MISMATCH",
        }
    }
}

/// Detailed navigation readiness. Typed fields are authoritative; `reason`
/// remains a human-readable compatibility projection for the current proto.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigationReadinessDetail {
    /// 整体 ready。
    pub ready: bool,
    /// 顶层 reason(中文 + 关键 blocker 标签)。
    pub reason: String,
    /// 当前就绪诊断结果。
    pub reason_code: NavigationReadinessCode,
    /// All currently active blockers in deterministic priority order.
    #[serde(default)]
    pub blockers: Vec<NavigationReadinessCode>,
    /// LCM backend 是否可用。
    pub lcm_backend: bool,
    /// Whether the real TF consumer reported a fresh complete chain.
    pub tf: bool,
    /// Whether the real planner consumer observed a fresh non-zero map revision.
    pub costmap: bool,
    /// Whether profile, planner input, and stack heartbeat are all usable.
    pub planner: bool,
    /// 当前 profile(给 driver / debug 用)。
    pub profile: String,
    /// module graph 节点 ID 数量(给 operator 看是否成功 build graph)。
    pub module_node_count: usize,
    /// 在 default profile 之外的,本阶段未启用的 native profile 列表。
    pub p1_native_profiles: Vec<String>,
    // === Phase 7 增量字段 ===
    /// canonical LCM backend 名（字段值固定为 "RealLcm"）。
    pub backend: String,
    /// Number of accepted real TF consumer observations.
    pub tf_count: u64,
    /// Map revision actually observed by the planner.
    pub costmap_revision: u64,
    /// Fresh guard-backed motion-sink health.
    pub cmd_vel_sink_ready: bool,
    /// 当前 profile canonical 名字(给 Python diagnostics operator 用)。
    pub active_profile: String,
    /// 是否至少收到过一帧真实 stack state/heartbeat。
    #[serde(default)]
    pub stack_state_observed: bool,
    /// 最近 stack state 是否仍在 heartbeat TTL 内。
    #[serde(default)]
    pub stack_state_fresh: bool,
    /// 最近 stack state 距当前的本地单调时钟年龄。
    #[serde(default)]
    pub stack_last_seen_age_ms: u64,
    /// 最近观测到的 stack 状态字符串。
    #[serde(default)]
    pub stack_last_status: String,
    /// Generation of the accepted TF producer observation.
    #[serde(default)]
    pub tf_generation: u64,
    /// Generation of the planner/scheduler input consumer.
    #[serde(default)]
    pub planner_generation: u64,
    /// Generation of the guard-backed motion sink.
    #[serde(default)]
    pub motion_sink_generation: u64,
    /// Planner-observed map frame.
    #[serde(default)]
    pub map_frame: String,
    /// Planner-observed transformed pose frame.
    #[serde(default)]
    pub pose_frame: String,
    /// Whether a TF observation has ever been received.
    #[serde(default)]
    pub tf_observed: bool,
    /// Whether the TF observation remains fresh.
    #[serde(default)]
    pub tf_fresh: bool,
    /// Whether a planner input observation has ever been received.
    #[serde(default)]
    pub planner_input_observed: bool,
    /// Whether the planner input observation remains fresh.
    #[serde(default)]
    pub planner_input_fresh: bool,
    /// Whether a motion-sink health observation has ever been received.
    #[serde(default)]
    pub motion_sink_observed: bool,
    /// Whether the motion-sink health observation remains fresh.
    #[serde(default)]
    pub motion_sink_fresh: bool,
}

fn profile_has_guarded_execution(profile: &str) -> bool {
    nuwax_nav_stack::NavStackProfile::from_cli_str(profile)
        .ok()
        .is_some_and(|profile| {
            nuwax_nav_stack::ProfileManifest::for_profile(
                profile,
                &nuwax_nav_stack::NavTopicConfig::empty(),
            )
            .activation
            .is_production_guarded()
        })
}

const P1_NATIVE_PROFILES: &[&str] = &[PROFILE_CMU_NATIVE, PROFILE_EXPLORATION_NATIVE];

/// Compute readiness exclusively from observations written by real consumers.
pub async fn compute_readiness(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
) -> NavigationReadinessDetail {
    let profile = state.profile().await;
    let profile_supported = profile_has_guarded_execution(profile);
    let stack_health = state.stack_health_snapshot();
    let observations = state.runtime_observation_snapshot();
    let lcm_backend = handle.lcm_backend_available;
    let tf = observations.tf_fresh && observations.tf_complete;
    let costmap = observations.planner_observed && observations.planner_fresh;
    let cmd_vel_sink_ready = observations.motion_sink_fresh && observations.motion_sink_ready;
    let planar_admitted=handle.mapping_adapter.as_ref().is_none_or(|adapter|{
        let a=adapter.lock();
        !a.checkpoint_localization_required && a.planar.as_ref().is_none_or(|binding|a.is_active() && a.planar_ready
            && a.planar_error.is_none() && binding.navigation.fresh())
    });
    let planner = profile_supported
        && planar_admitted
        && stack_health.fresh
        && observations.planner_fresh
        && observations.planner_ready
        && observations.planner_frame_matches
        && costmap;
    let backend = match handle.transport_backend_name {
        "real-lcm" | "real-lcm-unavailable" => "RealLcm",
        other => {
            // 不在这里 panic;未知 backend 按当前 enum 语义映射并记 log。
            tracing::warn!("nav readiness: 未知 transport_backend_name={}", other);
            "Unknown"
        }
    };
    let ready = lcm_backend && tf && costmap && planner;
    let mut blockers = Vec::new();
    if !planar_admitted {blockers.push(NavigationReadinessCode::PlannerNotReady);}
    if !lcm_backend {
        blockers.push(NavigationReadinessCode::LcmBackendUnavailable);
    }
    blockers.extend(observation_blockers(profile_supported, &stack_health, &observations));
    let reason_code = blockers
        .first()
        .copied()
        .unwrap_or(NavigationReadinessCode::Ready);
    let reason = readiness_reason(reason_code).to_string();

    NavigationReadinessDetail {
        ready,
        reason,
        reason_code,
        blockers,
        lcm_backend,
        tf,
        costmap,
        planner,

        profile: profile.to_string(),
        module_node_count: state.module_nodes().await.len(),
        p1_native_profiles: P1_NATIVE_PROFILES.iter().map(|s| s.to_string()).collect(),
        backend: backend.to_string(),
        tf_count: observations.tf_count,
        costmap_revision: observations.map_revision,
        cmd_vel_sink_ready,
        active_profile: profile.to_string(),
        stack_state_observed: stack_health.observed,
        stack_state_fresh: stack_health.fresh,
        stack_last_seen_age_ms: stack_health.age_ms,
        stack_last_status: stack_health.status,
        tf_generation: observations.tf_generation,
        planner_generation: observations.planner_generation,
        motion_sink_generation: observations.motion_sink_generation,
        map_frame: observations.map_frame,
        pose_frame: observations.pose_frame,
        tf_observed: observations.tf_observed,
        tf_fresh: observations.tf_fresh,
        planner_input_observed: observations.planner_observed,
        planner_input_fresh: observations.planner_fresh,
        motion_sink_observed: observations.motion_sink_observed,
        motion_sink_fresh: observations.motion_sink_fresh,
    }
}

#[allow(clippy::manual_async_fn)]
fn compute_admission_readiness<'a>(
    io: &'a NavAdmissionIo,
    state: &'a NavRuntimeState,
) -> impl std::future::Future<Output = NavigationReadinessDetail> + 'a {
    async move {
        let profile = state.profile().await;
        let profile_supported = profile_has_guarded_execution(profile);
        let stack_health = state.stack_health_snapshot();
        let observations = state.runtime_observation_snapshot();
        let lcm_backend = io.lcm_backend_available;
        let tf = observations.tf_fresh && observations.tf_complete;
        let costmap = observations.planner_observed && observations.planner_fresh;
        let cmd_vel_sink_ready = observations.motion_sink_fresh && observations.motion_sink_ready;
        let planner = profile_supported
            && stack_health.fresh
            && observations.planner_fresh
            && observations.planner_ready
            && observations.planner_frame_matches
            && costmap;
        let backend = match io.transport_backend_name {
            "real-lcm" | "real-lcm-unavailable" => "RealLcm",
            other => {
                warn!("nav readiness: 未知 transport_backend_name={}", other);
                "Unknown"
            }
        };
        let mut blockers = Vec::new();
        if !lcm_backend {
            blockers.push(NavigationReadinessCode::LcmBackendUnavailable);
        }
        blockers.extend(observation_blockers(profile_supported, &stack_health, &observations));
        let reason_code = blockers
            .first()
            .copied()
            .unwrap_or(NavigationReadinessCode::Ready);
        NavigationReadinessDetail {
            ready: lcm_backend && tf && costmap && planner,
            reason: readiness_reason(reason_code).to_string(),
            reason_code,
            blockers,
            lcm_backend,
            tf,
            costmap,
            planner,

            profile: profile.to_string(),
            module_node_count: state.module_nodes().await.len(),
            p1_native_profiles: P1_NATIVE_PROFILES.iter().map(|s| s.to_string()).collect(),
            backend: backend.to_string(),
            tf_count: observations.tf_count,
            costmap_revision: observations.map_revision,
            cmd_vel_sink_ready,
            active_profile: profile.to_string(),
            stack_state_observed: stack_health.observed,
            stack_state_fresh: stack_health.fresh,
            stack_last_seen_age_ms: stack_health.age_ms,
            stack_last_status: stack_health.status,
            tf_generation: observations.tf_generation,
            planner_generation: observations.planner_generation,
            motion_sink_generation: observations.motion_sink_generation,
            map_frame: observations.map_frame,
            pose_frame: observations.pose_frame,
            tf_observed: observations.tf_observed,
            tf_fresh: observations.tf_fresh,
            planner_input_observed: observations.planner_observed,
            planner_input_fresh: observations.planner_fresh,
            motion_sink_observed: observations.motion_sink_observed,
            motion_sink_fresh: observations.motion_sink_fresh,
        }
    }
}

/// 按同一观察快照分类实际缺失项；不改变各来源的期限或运动准入。
fn observation_blockers(
    profile_supported: bool,
    stack_health: &StackHealthSnapshot,
    observations: &RuntimeObservationSnapshot,
) -> Vec<NavigationReadinessCode> {
    let mut blockers = Vec::new();
    if !stack_health.observed {
        blockers.push(NavigationReadinessCode::StackStateUnobserved);
    } else if !stack_health.fresh {
        blockers.push(NavigationReadinessCode::StackStateStale);
    }
    if !observations.tf_observed {
        blockers.push(NavigationReadinessCode::TfUnobserved);
    } else if !observations.tf_fresh {
        blockers.push(NavigationReadinessCode::TfStale);
    } else if !observations.tf_complete {
        blockers.push(NavigationReadinessCode::TfIncomplete);
    }
    if !observations.planner_observed {
        blockers.push(NavigationReadinessCode::PlannerInputUnobserved);
    } else if !observations.planner_fresh {
        blockers.push(NavigationReadinessCode::PlannerInputStale);
    } else {
        if !observations.map_frame.trim().is_empty()
            && !observations.pose_frame.trim().is_empty()
            && !observations.planner_frame_matches {
            blockers.push(NavigationReadinessCode::PlannerFrameMismatch);
        }
        if !observations.planner_ready || !observations.planner_frame_matches {
            blockers.push(NavigationReadinessCode::PlannerNotReady);
        }
    }
    if !profile_supported {
        blockers.push(NavigationReadinessCode::ProfileUnsupported);
    }
    blockers
}

fn readiness_reason(code: NavigationReadinessCode) -> &'static str {
    match code {
        NavigationReadinessCode::Unknown => "UNKNOWN: 当前就绪诊断不可用",
        NavigationReadinessCode::Ready => "READY: navigation inputs are ready",
        NavigationReadinessCode::LcmBackendUnavailable => {
            "LCM_BACKEND_UNAVAILABLE: real LCM backend unavailable"
        }
        NavigationReadinessCode::StackStateUnobserved => {
            "STACK_STATE_UNOBSERVED: nav-stack heartbeat missing"
        }
        NavigationReadinessCode::StackStateStale => {
            "STACK_STATE_STALE: nav-stack heartbeat expired"
        }
        NavigationReadinessCode::TfUnobserved => {
            "TF_UNOBSERVED: map-to-odom-to-base chain not observed"
        }
        NavigationReadinessCode::TfStale => "TF_STALE: transform chain observation expired",
        NavigationReadinessCode::TfIncomplete => {
            "TF_INCOMPLETE: map-to-odom-to-base chain incomplete"
        }
        NavigationReadinessCode::PlannerInputUnobserved => {
            "PLANNER_INPUT_UNOBSERVED: planner consumer has not reported inputs"
        }
        NavigationReadinessCode::PlannerInputStale => {
            "PLANNER_INPUT_STALE: planner consumer input observation expired"
        }
        NavigationReadinessCode::PlannerFrameMismatch => {
            "PLANNER_FRAME_MISMATCH: pose frame does not match map frame"
        }
        NavigationReadinessCode::CostmapUnavailable => {
            "COSTMAP_UNAVAILABLE: planner has not observed a map revision"
        }
        NavigationReadinessCode::PlannerNotReady => {
            "PLANNER_NOT_READY: planner rejected its observed input set"
        }
        NavigationReadinessCode::ProfileUnsupported => {
            "PROFILE_UNSUPPORTED: selected profile is not production-qualified"
        }
        NavigationReadinessCode::GoalFrameMismatch => {
            "GOAL_FRAME_MISMATCH: goal frame does not match planner map frame"
        }
    }
}

/// Immutable proof produced by the request-level motion admission check.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MotionAdmissionProof {

    /// 原守卫状态采样的启动随机数。
    pub guard_boot_nonce: [u8; 16],
    /// Ordinary global-stop generation observed by the request-level check.
    pub motion_stop_generation: u64,
}

impl MotionAdmissionProof {
    /// Parse and validate a proof received over a variable-length wire format.
    pub fn try_from_wire(guard_boot_nonce: &[u8], motion_stop_generation: u64) -> Result<Self, String> {
        Ok(Self {
            guard_boot_nonce: guard_boot_nonce.try_into()
                .map_err(|_| "motion_guard_boot_nonce must contain exactly 16 bytes".to_string())?,
            motion_stop_generation,
        })
    }

}

#[cfg(test)]
pub(crate) const fn test_motion_admission() -> MotionAdmissionProof {
    MotionAdmissionProof {

        guard_boot_nonce: [7; 16],
        motion_stop_generation: 1,
    }
}

/// `set_goal` 的输入(已从 gRPC `NavigationGoal` 提取并归一化)。
#[derive(Clone)]
pub struct SetGoalInput {
    /// 已归一化并随目标摘要封存的执行约束。
    pub options: nuwax_nav_types_messages::GoalOptions,
    /// 目标点 ID(用户传或自动生成)。
    pub goal_id: String,
    /// Lease owner kind; public RPC and exploration bridge use distinct values.
    pub owner_kind: i32,
    /// Parent request that owns this goal.
    pub parent_request_id: String,
    /// Requested CAS generation; zero asks the runtime to allocate one.
    pub goal_generation: u64,
    /// Immutable request-level motion admission proof.
    pub motion_admission: MotionAdmissionProof,
    /// frame_id(`map` 默认)。
    pub frame_id: String,
    /// 目标位姿(在 `frame_id` 坐标系下)。
    pub target: PoseState,
}

struct AdmittedGoalInput {
    map_frame_generation: i64,
    input: SetGoalInput,
    producer: String,
    producer_epoch: i64,
    committed_map: schema::CommittedMapIdentity,
    boundary_id: String,

    boundary_owner: Option<Arc<crate::explore_runtime::ExploreStartConfig>>,
}

/// Machine-readable reason that allows a navigation admission to be retried.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetGoalRetryReason {
    /// 普通提交先于候选计算前进，需要重新读取输入，不是目标不可达。
    MapRevisionAdvanced,
    /// A typed navigation readiness blocker rejected admission.
    NavigationNotReady(NavigationReadinessCode),
}

impl SetGoalRetryReason {
    /// 准入重试使用独立机器码，不能与执行阶段的普通就绪诊断混淆。
    pub fn reason_code(self) -> String {
        match self {
            Self::MapRevisionAdvanced => "GOAL_RETRY_MAP_REVISION_ADVANCED".into(),
            Self::NavigationNotReady(code) => {
                format!("GOAL_RETRY_NAVIGATION_NOT_READY_{}", code.as_str())
            }
        }
    }
}

/// 本次导航准入已明确拒绝的原因，不表示目标永久不可达。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetGoalRejectReason {
    /// 原目标所属的地图域或坐标代际已经撤销。
    MapFrameChanged,
    /// The active map name no longer matches the name frozen by the caller.
    ActiveMapNameChanged,
    GoalIntentInvalid,
    GoalFrameMismatch,
    GoalBoundaryInvalid,
    GoalLeaseInvalid,
    GoalEnvelopeRange,
    GoalTimestampInvalid,
    GoalGenerationStale,
    GoalEnvelopeInvalid,
}

impl SetGoalRejectReason {
    /// 安装目标之前的拒绝原因直接投影到协议，显示文本不参与分类。
    pub fn as_str(self) -> &'static str {
        match self {
            Self::MapFrameChanged => "GOAL_MAP_FRAME_CHANGED",
            Self::ActiveMapNameChanged => "GOAL_ACTIVE_MAP_NAME_CHANGED",
            Self::GoalIntentInvalid => "GOAL_INTENT_INVALID",
            Self::GoalFrameMismatch => "GOAL_FRAME_MISMATCH",
            Self::GoalBoundaryInvalid => "GOAL_BOUNDARY_INVALID",
            Self::GoalLeaseInvalid => "GOAL_LEASE_INVALID",
            Self::GoalEnvelopeRange => "GOAL_ENVELOPE_RANGE",
            Self::GoalTimestampInvalid => "GOAL_TIMESTAMP_INVALID",
            Self::GoalGenerationStale => "GOAL_GENERATION_STALE",
            Self::GoalEnvelopeInvalid => "GOAL_ENVELOPE_INVALID",
        }
    }
}

/// Source of the lease in THIS command result; Some alone never grants ownership.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GoalLeaseOrigin {
    /// No execution owner was installed by this attempt.
    #[default]
    NotInstalled,
    /// Sealed attempted identity rejected before installation (for example Busy).
    Attempted,
    /// Exact idempotent replay; this command did not install an owner.
    Existing,
    /// Exact replay of an owner whose responsibility is still unresolved.
    ExistingUnresolved,
    /// This command installed the owner; dispatch effect and any resolution remain separate facts.
    Installed,
    /// This command installed then definitely released its original provisional owner.
    InstalledReleased,
    /// This command installed an owner that still needs exact terminal proof.
    InstalledUnresolved,
}

/// Local dispatch fact, separate from authoritative execution acceptance or settlement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GoalDispatchEffect {
    /// No new Goal publish was attempted by this command.
    #[default]
    NotAttempted,
    /// An explicit source point proves this command was not dispatched.
    NotDispatched,
    /// The local publish call completed successfully; no remote guarantee is implied.
    LocallyCompleted,
    /// The backend cannot prove absence of external effects.
    EffectUnconfirmed,
}

/// A command projection is not a second execution-state producer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GoalPublicationPolicy {
    /// Preserve the existing ordinary accepted/rejected local notification path.
    #[default]
    Local,
    /// Unary/Bridge result only: never rewrite SharedState or broadcast this projection.
    NoPublish,
}

/// Machine reason constructed from typed control flow, never parsed from message text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalSubmissionDiagnostic {
    /// The installed attempt retains responsibility after uncertain completion.
    PublishUnconfirmed,
    /// A terminal observation exists but the full original receipt is not readable yet.
    TerminalProofPending,
}

impl GoalSubmissionDiagnostic {
    /// Existing wire reason_code; no new protobuf field is required.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::PublishUnconfirmed => "GOAL_PUBLISH_UNCONFIRMED",
            Self::TerminalProofPending => "GOAL_TERMINAL_PROOF_PENDING",
        }
    }
}

/// Submission facts plus a lossless command projection; settlement is independent of accepted.
#[derive(Default)]
pub struct SetGoalOutcome {
    /// Current execution projection for this exact attempted identity.
    pub state: NavState,
    /// Attempted/existing/installed lease; consult lease_origin before transferring responsibility.
    pub lease: Option<GoalLease>,
    /// Retryable admission cause; unrelated dispatch uncertainty never creates a retry budget.
    pub retry_reason: Option<SetGoalRetryReason>,
    /// 本次准入的明确拒绝原因，不包含未知发布结果。
    pub reject_reason: Option<SetGoalRejectReason>,
    /// Trusted source of the lease in this result.
    pub lease_origin: GoalLeaseOrigin,
    /// Local transport completion fact.
    #[cfg(test)] pub dispatch_effect: GoalDispatchEffect,
    /// Typed command-only reason when no complete original receipt is available.
    pub diagnostic: Option<GoalSubmissionDiagnostic>,
    /// Controls only local command projection, never authoritative consumer fanout.
    pub publication: GoalPublicationPolicy,
    /// Original authoritative terminal body, copied intact rather than reconstructed.
    pub resolved_receipt: Option<NavigationUpdate>,
}

impl SetGoalOutcome {
    /// Confirmed acceptance does not imply settlement; a receipt is evaluated separately.
    pub fn accepted(&self) -> bool {
        self.resolved_receipt.is_none()
            && matches!(self.state.status.as_str(), "ACCEPTED" | "PLANNING" | "RUNNING")
    }
}

/// 定向取消使用完整租约身份；全空身份由拥有方绑定当前目标。
#[derive(Clone)]
pub struct CancelGoalInput {
    /// Public business/session identifier.
    pub goal_id: String,
    /// Lease owner kind.
    pub owner_kind: i32,
    /// Lease parent request.
    pub parent_request_id: String,
    /// Lease goal generation.
    pub goal_generation: u64,
    /// Geometry producer incarnation that issued the goal lease.
    pub producer_runtime_generation: u64,
}

impl CancelGoalInput {
    /// 空取消没有缓存身份，由目标拥有方在事务锁内选择当前目标。
    fn targets_current(&self) -> bool {
        self.goal_id.trim().is_empty() && self.owner_kind == 0
            && self.parent_request_id.trim().is_empty() && self.goal_generation == 0
            && self.producer_runtime_generation == 0
    }
}

/// `cancel` 的输出:同 set_goal。
pub struct CancelOutcome {
    /// 新的 NavState。
    pub state: NavState,
    /// Lease targeted by this cancellation.
    pub lease: Option<GoalLease>,
}

/// Optional hard navigation boundary carried only by exploration child goals.
#[derive(Debug, Clone, PartialEq)]
pub struct GoalBoundaryConstraint {
    /// Canonical planner frame. Production exploration requires `map`.
    pub frame_id: String,
    /// Ordered polygon vertices; the closing edge is implicit.
    pub points: Vec<[f64; 3]>,
}

fn validate_committed_map_identity(
    committed_map: &schema::CommittedMapIdentity,
) -> Result<(), String> {
    if committed_map.session_id.trim().is_empty()
        || committed_map.session_id.trim() != committed_map.session_id
        || committed_map.session_generation <= 0
        || committed_map.revision <= 0
        || committed_map.commit_boot_nonce == [0; 16]
        || committed_map.commit_sequence <= 0
        || committed_map.source_timestamp_ns < 0
        || committed_map.frame_id.trim().is_empty()
        || committed_map.frame_id.trim() != committed_map.frame_id
    {
        return Err("COMMITTED_MAP_REJECTED: incomplete identity".to_string());
    }
    Ok(())
}

fn rejected_goal_intent(goal_id: String, reason: impl Into<String>) -> SetGoalOutcome {
    SetGoalOutcome {
        state: NavState {
            status: NavPhase::Failed.as_str().to_string(),
            message: format!("SET_GOAL_REJECTED[GOAL_INTENT_INVALID]: {}", reason.into()),
            goal_id,
            distance_remaining: 0.0,
            timestamp_ns: now_ns(),
        },
        lease: None,
        retry_reason: None,
        reject_reason: Some(SetGoalRejectReason::GoalIntentInvalid),
        ..SetGoalOutcome::default()
    }
}

/// Coordinate/commit domain shared by navigation admission and frozen Surface work.
/// Revision and receipt fields are not part of the coordinate/commit domain.
pub(crate) fn same_committed_map_domain(
    source: &schema::CommittedMapIdentity,
    current: &schema::CommittedMapIdentity,
) -> bool {
    source.session_id == current.session_id
        && source.session_generation == current.session_generation
        && source.frame_id == current.frame_id
        && source.commit_boot_nonce == current.commit_boot_nonce
}

fn goal_map_rejection(admission: &NavAdmissionContext, source: &schema::CommittedMapIdentity,
    frame_generation: i64, goal_id: &str, source_prevalidated: bool) -> Option<SetGoalOutcome> {
    if admission.mapping_adapter.as_ref().is_some_and(|a| a.lock().checkpoint_localization_required) {
        let mut outcome = rejected_goal_intent(goal_id.into(), "已激活地图正在等待真实定位提交");
        outcome.reject_reason = None;
        outcome.retry_reason = Some(SetGoalRetryReason::NavigationNotReady(NavigationReadinessCode::PlannerNotReady));
        return Some(outcome);
    }
    let current = admission.committed_map_authority.lock().ok().and_then(|m| m.clone());
    let Some(current) = current else {
        let mut outcome = rejected_goal_intent(goal_id.into(), "authoritative committed map is unavailable");
        outcome.reject_reason = None;
        outcome.retry_reason = Some(SetGoalRetryReason::MapRevisionAdvanced);
        return Some(outcome);
    };
    let same_domain = same_committed_map_domain(source, &current);
    if frame_generation < 0 || !same_domain || frame_generation != admission.map_frame_generation() {
        return Some(rejected_goal_intent_with_reason(goal_id.into(), "map session or coordinate generation changed before admission", SetGoalRejectReason::MapFrameChanged));
    }
    if source == &current { return None; }
    if source.revision < current.revision {
        if source_prevalidated || (admission.allow_previous_map_revisions && admission.mapping_adapter.is_some()) { return None; }
        let mut outcome = rejected_goal_intent(goal_id.into(), "committed map revision advanced before admission");
        outcome.reject_reason = None;
        outcome.retry_reason = Some(SetGoalRetryReason::MapRevisionAdvanced);
        return Some(outcome);
    }
    Some(rejected_goal_intent(goal_id.into(), "committed map revision or receipt conflicts with authority"))
}

fn rejected_goal_intent_with_reason(
    goal_id: String,
    reason: impl Into<String>,
    reject_reason: SetGoalRejectReason,
) -> SetGoalOutcome {
    let mut outcome = rejected_goal_intent(goal_id, reason);
    outcome.reject_reason = Some(reject_reason);
    outcome
}

async fn goal_intent_from_set_goal(
    admission: &NavAdmissionContext,
    state: &NavRuntimeState,
    mut input: SetGoalInput,
    boundary: Option<GoalBoundaryConstraint>,
) -> Result<schema::GoalIntent, String> {
    let _map_guard = admission.mapping_lifecycle_lock.lock().await;
    input.goal_id = input.goal_id.trim().to_string();
    input.parent_request_id = input.parent_request_id.trim().to_string();
    input.target.frame_id = input.frame_id.clone();
    if input.target.timestamp_ns == 0 {
        input.target.timestamp_ns = now_ns();
    }
    // 公开导航只观察 Geometry composition 已提交的 authority；候选事件/watch
    // 不得在 producer 侧提升权威性。
    let committed_map = admission
        .committed_map_authority
        .lock()
        .map_err(|_| "authoritative committed map state is unavailable".to_string())?
        .clone()
        .ok_or_else(|| "authoritative committed map is unavailable".to_string())?;
    let goal_generation = i64::try_from(input.goal_generation)
        .map_err(|_| "goal_generation exceeds i64".to_string())?;
    let cancel_generation = i64::try_from(state.cancel_generation())
        .map_err(|_| "cancel_generation exceeds i64".to_string())?;
    let producer_epoch = i64::try_from(state.producer_runtime_generation)
        .map_err(|_| "producer epoch exceeds i64".to_string())?;
    let (producer, priority) = match input.owner_kind {
        schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV => ("public-navigation", 0),
        #[cfg(test)]
        schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD => ("wavefront", 100),
        _ => return Err("unsupported navigation owner kind".to_string()),
    };
    let goal = schema::PoseStamped {
        header: schema::Header {
            seq: 0,
            stamp_sec: i32::try_from(input.target.timestamp_ns / 1_000_000_000)
                .map_err(|_| "goal timestamp seconds exceed i32".to_string())?,
            stamp_nsec: (input.target.timestamp_ns % 1_000_000_000) as i32,
            frame_id: input.frame_id.clone(),
        },
        pose: schema::Pose {
            position: [input.target.x, input.target.y, input.target.z],
            orientation_xyzw: [
                input.target.qx,
                input.target.qy,
                input.target.qz,
                input.target.qw,
            ],
        },
    };
    input.options.validate().map_err(|reason| nuwax_codec::CodecError::LcmEncode(reason).to_string())?;
    let (boundary_id, boundary_points) = match boundary {
        Some(boundary) => {
            if boundary.frame_id.trim() != committed_map.frame_id.trim() {
                return Err(
                    "GOAL_BOUNDARY_INVALID: boundary frame differs from committed map".to_string(),
                );
            }
            let boundary_id = format!("{}:boundary", input.parent_request_id);
            (boundary_id, boundary.points)
        }
        None => (String::new(), Vec::new()),
    };
    Ok(schema::GoalIntent {
        map_frame_generation: admission.map_frame_generation(),
        options: input.options,
        producer: producer.to_string(),
        producer_epoch,
        priority,
        session_id: input.goal_id,
        owner_kind: input.owner_kind,
        parent_request_id: input.parent_request_id,
        goal_generation,
        cancel_generation,
        committed_map,
        boundary_id,
        boundary_points,

        motion_guard_boot_nonce: input.motion_admission.guard_boot_nonce,
        motion_stop_generation: i64::try_from(input.motion_admission.motion_stop_generation).map_err(|_| "motion_stop_generation cannot fit LCM i64".to_string())?,
        goal,
    })
}

/// Submit one goal to the real stack.
///
/// 失败时返回带 `status=FAILED` 的 NavState(但 `goal_id` 保留),
/// 不写入 active goal 也不 publish `/nav/goal`。
pub async fn set_goal(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    input: SetGoalInput,
) -> SetGoalOutcome {
    set_goal_with_boundary(handle, state, input, None).await
}

/// 将旧公开输入适配为共享不可变目标 intent。
pub async fn set_goal_with_boundary(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    input: SetGoalInput,
    boundary: Option<GoalBoundaryConstraint>,
) -> SetGoalOutcome {
    let goal_id = input.goal_id.clone();
    if input.owner_kind != schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV {
        return rejected_goal_intent(
            goal_id,
            "public navigation adapter cannot assert a non-public producer epoch",
        );
    }
    let readiness = compute_readiness(handle, state).await;
    if !input.frame_id.trim().is_empty()
        && !readiness.map_frame.trim().is_empty()
        && input.frame_id.trim() != readiness.map_frame.trim()
    {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!(
                    "SET_GOAL_REJECTED[GOAL_FRAME_MISMATCH]: goal_frame={} planner_map_frame={}",
                    input.frame_id, readiness.map_frame,
                ),
                goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
            reject_reason: Some(SetGoalRejectReason::GoalFrameMismatch),
            ..SetGoalOutcome::default()
        };
    }
    let admission = NavAdmissionContext::new(
        handle.mapping_lifecycle_lock.clone(),
        handle.committed_map_authority.clone(),
        handle.map_revision_latest_tx.subscribe(),
    ).with_mapping_adapter(handle.mapping_adapter.clone());
    let io = NavAdmissionIo::new(
        handle.state.clone(),
        handle.transport.clone(),
        handle.transport_backend_name,
        handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
);
    let intent = match goal_intent_from_set_goal(&admission, state, input, boundary).await {
        Ok(intent) => intent,
        Err(reason) => return rejected_goal_intent(goal_id, reason),
    };
    admit_goal_intent(
        &admission,
        &io,
        state,
        intent,
        GoalProducerEpochAuthority::PublicNavigation,
    )
    .await
}

/// 校验共享 intent，并执行唯一目标租约 CAS 准入。
pub(crate) async fn admit_goal_intent(
    admission: &NavAdmissionContext,
    io: &NavAdmissionIo,
    state: &NavRuntimeState,
    mut intent: schema::GoalIntent,
    producer_authority: GoalProducerEpochAuthority,
) -> SetGoalOutcome {
    let goal_id = intent.session_id.clone();
    let producer_contract_valid = matches!(
        (intent.producer.as_str(), intent.owner_kind, intent.priority,),
        (
            "public-navigation",
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            0
        ) | ("mapping-cloud", schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV, 0)
            | (
                "wavefront",
                schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                100
            )
    );
    let boundary_owner = match &producer_authority {
        GoalProducerEpochAuthority::Wavefront { boundary_owner, .. } => Some(Arc::clone(boundary_owner)),
        _ => None,
    };
    let authoritative_producer_epoch = match producer_authority {
        GoalProducerEpochAuthority::PublicNavigation if intent.producer == "public-navigation" => {
            state.producer_runtime_generation()
        }
        GoalProducerEpochAuthority::MappingCloud if intent.producer == "mapping-cloud" => {
            state.producer_runtime_generation()
        }
        GoalProducerEpochAuthority::Wavefront {
            current_session_generation,
            ..
        } if intent.producer == "wavefront" => {
            current_session_generation.load(AtomicOrdering::Acquire)
        }
        _ => 0,
    };
    if !producer_contract_valid
        || authoritative_producer_epoch == 0
        || i64::try_from(authoritative_producer_epoch).ok() != Some(intent.producer_epoch)
        || intent.goal_generation < 0
        || intent.cancel_generation < 0
        || intent.session_id.trim().is_empty()
        || intent.session_id.trim() != intent.session_id
        || intent.parent_request_id.trim().is_empty()
        || intent.parent_request_id.trim() != intent.parent_request_id
    {
        return rejected_goal_intent(
            goal_id,
            "invalid producer, owner, priority, or generation identity",
        );
    }
    if let Err(reason) = validate_committed_map_identity(&intent.committed_map) {
        return rejected_goal_intent(goal_id, reason);
    }
    if let Some(rejection) = goal_map_rejection(admission, &intent.committed_map, intent.map_frame_generation, &goal_id, false) { return rejection; }
    if intent.goal.header.seq != 0
        || intent.goal.header.frame_id != intent.committed_map.frame_id
        || intent.goal.header.stamp_sec < 0
        || !(0..1_000_000_000).contains(&intent.goal.header.stamp_nsec)
        || (intent.goal.header.stamp_sec == 0 && intent.goal.header.stamp_nsec == 0)
        || intent
            .goal
            .pose
            .position
            .iter()
            .chain(intent.goal.pose.orientation_xyzw.iter())
            .any(|value| !value.is_finite())
    {
        return rejected_goal_intent(goal_id, "goal header, frame, timestamp, or pose is invalid");
    }
    let motion_admission = MotionAdmissionProof {

        guard_boot_nonce: intent.motion_guard_boot_nonce,
        motion_stop_generation: match u64::try_from(intent.motion_stop_generation) {
            Ok(value) => value,
            Err(_) => return rejected_goal_intent(goal_id, "motion stop generation is invalid"),
        },
    };

    if let Err(reason) = intent.options.validate() {
        return rejected_goal_intent(goal_id, nuwax_codec::CodecError::LcmEncode(reason).to_string());
    }
    if !boundary_matches_owner(&intent.committed_map.frame_id, &intent.boundary_points, boundary_owner.as_deref()) {
        return rejected_goal_intent(goal_id, "boundary differs from the original exploration owner");
    }
    let boundary = if intent.boundary_points.is_empty() {
        if !intent.boundary_id.is_empty() {
            return rejected_goal_intent(goal_id, "empty boundary carries non-empty identity");
        }
        None
    } else {
        if intent.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
            || intent.boundary_id != format!("{}:boundary", intent.parent_request_id)
            || intent.boundary_points.len() < 3
            || intent
                .boundary_points
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
        {
            return rejected_goal_intent(goal_id, "boundary identity or polygon is invalid");
        }
        Some(GoalBoundaryConstraint {
            frame_id: intent.committed_map.frame_id.clone(),
            points: std::mem::take(&mut intent.boundary_points),
        })
    };
    let timestamp_ns = if intent.goal.header.stamp_sec < 0 || intent.goal.header.stamp_nsec < 0 {
        return rejected_goal_intent(goal_id, "goal timestamp is negative");
    } else {
        (intent.goal.header.stamp_sec as u64)
            .saturating_mul(1_000_000_000)
            .saturating_add(intent.goal.header.stamp_nsec as u64)
    };
    let admitted = AdmittedGoalInput {
        map_frame_generation: intent.map_frame_generation,
        input: SetGoalInput {
            options: intent.options,
            goal_id: intent.session_id,
            owner_kind: intent.owner_kind,
            parent_request_id: intent.parent_request_id,
            goal_generation: intent.goal_generation as u64,
            motion_admission,
            frame_id: intent.committed_map.frame_id.clone(),
            target: PoseState {
                x: intent.goal.pose.position[0],
                y: intent.goal.pose.position[1],
                z: intent.goal.pose.position[2],
                qx: intent.goal.pose.orientation_xyzw[0],
                qy: intent.goal.pose.orientation_xyzw[1],
                qz: intent.goal.pose.orientation_xyzw[2],
                qw: intent.goal.pose.orientation_xyzw[3],
                frame_id: intent.goal.header.frame_id,
                timestamp_ns,
            },
        },
        producer: intent.producer,
        producer_epoch: intent.producer_epoch,
        committed_map: intent.committed_map,
        boundary_id: intent.boundary_id,
        boundary_owner,
    };
    admit_goal_input(io, state, admitted, boundary, admission).await
}

fn pose_action_matches(left: &PoseState, right: &PoseState) -> bool {
    [left.x, left.y, left.z, left.qx, left.qy, left.qz, left.qw]
        == [right.x, right.y, right.z, right.qx, right.qy, right.qz, right.qw]
}

fn boundary_matches_owner(
    frame_id: &str,
    points: &[[f64; 3]],
    owner: Option<&crate::explore_runtime::ExploreStartConfig>,
) -> bool {
    let boundary = owner.and_then(|owner| owner.boundary.as_ref());
    match boundary {
        None => points.is_empty(),
        Some(boundary) => !frame_id.is_empty()
            && points.len() == boundary.vertices().len()
            && points.iter().zip(boundary.vertices()).all(|(point, vertex)|
                point[0] == vertex.x && point[1] == vertex.y && point[2] == 0.0),
    }
}

async fn admit_goal_input(
    io: &NavAdmissionIo,
    state: &NavRuntimeState,
    admitted: AdmittedGoalInput,
    boundary: Option<GoalBoundaryConstraint>,
    admission: &NavAdmissionContext,
) -> SetGoalOutcome {
    let AdmittedGoalInput {
        map_frame_generation,
        mut input,
        producer,
        producer_epoch,
        committed_map,
        boundary_id,
        boundary_owner,
    } = admitted;
    input.goal_id = input.goal_id.trim().to_string();
    input.parent_request_id = input.parent_request_id.trim().to_string();
    input.target.frame_id = input.frame_id.clone();
    if input.target.timestamp_ns == 0 {
        input.target.timestamp_ns = now_ns();
    }
    let goal_frame = input.frame_id.trim();
    let boundary_invalid = boundary.as_ref().is_some_and(|boundary| {
        input.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
            || boundary.frame_id.trim() != goal_frame
            || boundary.points.len() < 3
            || boundary
                .points
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
    });
    if boundary_invalid {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: "SET_GOAL_REJECTED[GOAL_BOUNDARY_INVALID]".to_string(),
                goal_id: input.goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
            reject_reason: Some(SetGoalRejectReason::GoalBoundaryInvalid),
            ..SetGoalOutcome::default()
        };
    }

    if input.goal_id.is_empty()
        || input.parent_request_id.is_empty()
        || input.goal_generation > i64::MAX as u64
        || input.target.timestamp_ns > i64::MAX as u64
        || !matches!(
            input.owner_kind,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
                | schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        )
    {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: "SET_GOAL_REJECTED[GOAL_LEASE_INVALID]: incomplete owner identity"
                    .to_string(),
                goal_id: input.goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
            reject_reason: Some(SetGoalRejectReason::GoalLeaseInvalid),
            ..SetGoalOutcome::default()
        };
    }
    // 1. readiness 校验。
    let detail = compute_admission_readiness(io, state).await;
    if !detail.ready {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!(
                    "SET_GOAL_REJECTED[{}]: {}",
                    detail.reason_code.as_str(),
                    detail.reason,
                ),
                goal_id: input.goal_id.clone(),
                distance_remaining: distance_to_target_from_state(
                    &io.shared_state,
                    &input.target,
                    state.configured_profile() == PROFILE_MLS3D,
                )
                .await,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: Some(SetGoalRetryReason::NavigationNotReady(detail.reason_code)),
            reject_reason: None,
            ..SetGoalOutcome::default()
        };
    }
    if input.frame_id != detail.map_frame {
        let rejection = SetGoalRejectReason::GoalFrameMismatch;
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!(
                    "SET_GOAL_REJECTED[{}]: goal_frame={} planner_map_frame={}",
                    rejection.as_str(),
                    input.frame_id,
                    detail.map_frame,
                ),
                goal_id: input.goal_id.clone(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
            reject_reason: Some(rejection),
            ..SetGoalOutcome::default()
        };
    }
    let distance_remaining = distance_to_target_from_state(
        &io.shared_state,
        &input.target,
        state.configured_profile() == PROFILE_MLS3D,
    )
    .await;
    let transaction_guard = state.transaction_gate.lock().await;
    let map_goal_guard = admission.mapping_lifecycle_lock.lock().await;
    // 入口已核验源提交；等就绪期间的普通修订前进不撤销目标，坐标代际仍必须一致。
    if let Some(rejection) = goal_map_rejection(admission, &committed_map, map_frame_generation, &input.goal_id, true) { return rejection; }
    if !admission.map_name_unchanged() {
        return rejected_goal_intent_with_reason(
            input.goal_id,
            "active map name changed before goal lease admission",
            SetGoalRejectReason::ActiveMapNameChanged,
        );
    }
    let goal_generation = match state.reserve_goal_generation(input.goal_generation) {
        Ok(generation) => generation,
        Err(()) => {
            return SetGoalOutcome {
                state: NavState {
                    status: NavPhase::Failed.as_str().to_string(),
                    message: "SET_GOAL_REJECTED[GOAL_ENVELOPE_RANGE]".to_string(),
                    goal_id: input.goal_id,
                    distance_remaining: 0.0,
                    timestamp_ns: now_ns(),
                },
                lease: None,
                retry_reason: None,
                reject_reason: Some(SetGoalRejectReason::GoalEnvelopeRange),
                ..SetGoalOutcome::default()
            };
        }
    };
    let producer_runtime_generation = state.producer_runtime_generation();
    if goal_generation > i64::MAX as u64
        || producer_runtime_generation > i64::MAX as u64
        || input.target.timestamp_ns > i64::MAX as u64
    {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: "SET_GOAL_REJECTED[GOAL_ENVELOPE_RANGE]".to_string(),
                goal_id: input.goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
            reject_reason: Some(SetGoalRejectReason::GoalEnvelopeRange),
            ..SetGoalOutcome::default()
        };
    }
    let stamp_sec = match i32::try_from(input.target.timestamp_ns / 1_000_000_000) {
        Ok(value) => value,
        Err(_) => {
            return SetGoalOutcome {
                state: NavState {
                    status: NavPhase::Failed.as_str().to_string(),
                    message: "SET_GOAL_REJECTED[GOAL_TIMESTAMP_INVALID]".to_string(),
                    goal_id: input.goal_id,
                    distance_remaining: 0.0,
                    timestamp_ns: now_ns(),
                },
                lease: None,
                retry_reason: None,
                reject_reason: Some(SetGoalRejectReason::GoalTimestampInvalid),
                ..SetGoalOutcome::default()
            };
        }
    };
    let (boundary_frame_id, boundary_points) = boundary
        .map(|boundary| (boundary.frame_id.trim().to_string(), boundary.points))
        .unwrap_or_default();
    let motion_stop_generation = match i64::try_from(input.motion_admission.motion_stop_generation) {
        Ok(value) => value,
        Err(_) => return SetGoalOutcome {
            state: NavState { status: NavPhase::Failed.as_str().to_string(),
                message: "SET_GOAL_REJECTED[LCM_ENCODE]: motion_stop_generation cannot fit i64".into(),
                goal_id: input.goal_id, distance_remaining: 0.0, timestamp_ns: now_ns() },
            reject_reason: Some(SetGoalRejectReason::GoalEnvelopeRange),
            ..SetGoalOutcome::default()
        },
    };
    let envelope = schema::GoalEnvelope {
        map_frame_generation,
        body_reference_frame_id: io.body_reference_frame_id.to_string(),
        options: input.options,
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: input.goal_id.clone(),
        owner_kind: input.owner_kind,
        parent_request_id: input.parent_request_id.clone(),
        goal_generation: goal_generation as i64,
        producer: producer.clone(),
        producer_epoch,
        producer_runtime_generation: producer_runtime_generation as i64,
        committed_map_session_id: committed_map.session_id,
        committed_map_session_generation: committed_map.session_generation,
        committed_map_revision: committed_map.revision,
        committed_map_boot_nonce: committed_map.commit_boot_nonce,
        committed_map_commit_sequence: committed_map.commit_sequence,
        committed_map_source_timestamp_ns: committed_map.source_timestamp_ns,
        committed_map_frame_id: committed_map.frame_id,
        source_timestamp_ns: input.target.timestamp_ns as i64,
        boundary_id,

        motion_guard_boot_nonce: input.motion_admission.guard_boot_nonce,
        motion_stop_generation,
        boundary_frame_id,
        boundary_points,
        goal: schema::PoseStamped {
            header: schema::Header {
                seq: i32::try_from(goal_generation).unwrap_or(i32::MAX),
                stamp_sec,
                stamp_nsec: (input.target.timestamp_ns % 1_000_000_000) as i32,
                frame_id: input.frame_id.clone(),
            },
            pose: schema::Pose {
                position: [input.target.x, input.target.y, input.target.z],
                orientation_xyzw: [
                    input.target.qx,
                    input.target.qy,
                    input.target.qz,
                    input.target.qw,
                ],
            },
        },
        ..schema::GoalEnvelope::default()
    };
    let lease = GoalLease {
        session_id: envelope.session_id.clone(),
        owner_kind: envelope.owner_kind,
        parent_request_id: envelope.parent_request_id.clone(),
        goal_generation,
        producer_runtime_generation,
        path_revision: 0,
        planner_runtime_generation: 0,
    };

    let goal_payload;
    {
        let mut slot = state.inner.write().await;
        if let Some(active) = slot.goal.as_ref() {
            if active.motion_admission.guard_boot_nonce == input.motion_admission.guard_boot_nonce
                && input.motion_admission.motion_stop_generation < active.motion_admission.motion_stop_generation {
                return SetGoalOutcome {
                    state: NavState { status: "FAILED".into(), message: "SET_GOAL_REJECTED[GOAL_GENERATION_STALE]".into(),
                        goal_id: lease.session_id.clone(), distance_remaining, timestamp_ns: now_ns() },
                    lease: Some(lease), lease_origin: GoalLeaseOrigin::Attempted,
                    reject_reason: Some(SetGoalRejectReason::GoalGenerationStale), ..SetGoalOutcome::default()
                };
            }
            if active.lease.identity_matches(&lease)
                && active.frame_id == input.frame_id
                && pose_action_matches(&active.target, &input.target)
                && active.options == input.options
                && active.producer == producer
                && active.producer_epoch == producer_epoch
                && boundary_matches_owner(&input.frame_id, &envelope.boundary_points, active.boundary_owner.as_deref())
            {
                let unresolved = slot.phase == NavPhase::DispatchUnresolved;
                let proof_pending = slot.phase.is_terminal();
                let diagnostic = if proof_pending {
                    Some(GoalSubmissionDiagnostic::TerminalProofPending)
                } else if unresolved {
                    Some(GoalSubmissionDiagnostic::PublishUnconfirmed)
                } else {
                    None
                };
                return SetGoalOutcome {
                    state: NavState {
                        status: if proof_pending { "UNCONFIRMED" } else { slot.phase.as_str() }.to_string(),
                        message: if diagnostic.is_some() { slot.message.clone() }
                            else { "GOAL_LEASE_IDEMPOTENT: existing lease retained".to_string() },
                        goal_id: lease.session_id.clone(),
                        distance_remaining,
                        timestamp_ns: if diagnostic.is_some() { 0 } else { now_ns() },
                    },
                    lease: Some(active.lease.clone()),
                    lease_origin: if diagnostic.is_some() { GoalLeaseOrigin::ExistingUnresolved }
                        else { GoalLeaseOrigin::Existing },
                    diagnostic,
                    publication: if diagnostic.is_some() { GoalPublicationPolicy::NoPublish }
                        else { GoalPublicationPolicy::Local },
                    ..SetGoalOutcome::default()
                };
            }
        }
        if goal_generation <= slot.last_goal_generation {
            return SetGoalOutcome {
                state: NavState {
                    status: NavPhase::Failed.as_str().to_string(),
                    message: format!(
                        "SET_GOAL_REJECTED[GOAL_GENERATION_STALE]: received={} last={}",
                        goal_generation, slot.last_goal_generation
                    ),
                    goal_id: lease.session_id.clone(),
                    distance_remaining,
                    timestamp_ns: now_ns(),
                },
                lease: Some(lease),
                lease_origin: GoalLeaseOrigin::Attempted,
                retry_reason: None,
                reject_reason: Some(SetGoalRejectReason::GoalGenerationStale),
                ..SetGoalOutcome::default()
            };
        }
        goal_payload = match typed_codec::encode_goal_envelope_payload(&envelope) {
        Ok(payload) => payload,
        Err(error) => {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!("SET_GOAL_REJECTED[GOAL_ENVELOPE_INVALID]: {error}"),
                goal_id: input.goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
            reject_reason: Some(SetGoalRejectReason::GoalEnvelopeInvalid),
            ..SetGoalOutcome::default()
        };
    }
    };
        slot.last_goal_generation = goal_generation;
        slot.phase = NavPhase::Accepted;
        slot.message = format!(
            "导航目标已提交到真实 stack,等待 /nav/state (profile={})",
            slot.profile
        );
        slot.goal = Some(ActiveGoal {
            motion_admission: input.motion_admission,
            id: input.goal_id.clone(),
            frame_id: input.frame_id.clone(),
            target: input.target.clone(),
            options: input.options,
            producer,
            producer_epoch,
            boundary_owner,
            lease: lease.clone(),
        });

    }
    let publish_attempt =
        match state.begin_publish(PublishKind::Goal, lease.clone(), goal_generation) {
            Ok(attempt) => attempt,
            Err(reason) => {
                // Installation happened; begin_publish proves only that this command
                // never reached the backend. Release solely the original provisional owner.
                return project_installed_goal_completion(
                    state, &lease, distance_remaining, GoalDispatchEffect::NotDispatched,
                    format!("SET_GOAL_REJECTED[{reason}]"), true,
                ).await;
            }
        };
    drop(map_goal_guard);
    drop(transaction_guard);

    // 2. publish sealed GoalEnvelope without N, L, slot, or synchronous guards.
    let publish_result = publish_encoded_goal_frame_from_transport(&io.transport, goal_payload).await;
    let _transaction_guard = state.transaction_gate.lock().await;
    let finished = state.finish_publish(PublishKind::Goal, &lease, goal_generation, publish_attempt);
    let dispatch_effect = match &publish_result {
        Ok(()) => GoalDispatchEffect::LocallyCompleted,
        Err(error) if error.is_definitely_not_dispatched() => GoalDispatchEffect::NotDispatched,
        Err(_) => GoalDispatchEffect::EffectUnconfirmed,
    };
    if !finished || publish_result.is_err() {
        let message = match publish_result {
            Err(error) => format!("SetNavigationGoal publish completion: {error}; flight_finished={finished}"),
            Ok(()) => "SET_GOAL_COMPLETION_SUPERSEDED: local publish completed; ownership still requires proof".to_string(),
        };
        warn!(session_id = %lease.session_id, goal_generation = lease.goal_generation,
            producer_runtime_generation = lease.producer_runtime_generation,
            ?dispatch_effect, finished, cause = %message, "goal submission requires exact ownership resolution");
        return project_installed_goal_completion(
            state, &lease, distance_remaining, dispatch_effect, message, finished,
        ).await;
    }

    SetGoalOutcome {
        state: NavState {
            status: NavPhase::Accepted.as_str().to_string(),
            message: "导航目标已接受,等待真实 stack 状态".into(),
            goal_id: input.goal_id,
            distance_remaining,
            timestamp_ns: now_ns(),
        },
        lease: Some(lease),
        lease_origin: GoalLeaseOrigin::Installed,
        #[cfg(test)] dispatch_effect: GoalDispatchEffect::LocallyCompleted,
        ..SetGoalOutcome::default()
    }
}

/// Select facts while the caller holds the ORIGINAL transaction gate. This helper
/// never expands that gate across transport I/O or waits for a terminal receipt.
async fn project_installed_goal_completion(
    state: &NavRuntimeState,
    attempted: &GoalLease,
    distance_remaining: f32,
    dispatch_effect: GoalDispatchEffect,
    message: String,
    allow_definite_rollback: bool,
) -> SetGoalOutcome {
    let mut slot = state.inner.write().await;
    let input = CancelGoalInput {
        goal_id: attempted.session_id.clone(), owner_kind: attempted.owner_kind,
        parent_request_id: attempted.parent_request_id.clone(),
        goal_generation: attempted.goal_generation,
        producer_runtime_generation: attempted.producer_runtime_generation,
    };
    let exact_active = slot.goal.as_ref().filter(|goal| goal.lease.identity_matches(attempted));
    let receipt = slot.terminal_receipts.iter().rev().find(|receipt| {
        let settlement_proven = receipt.retired_without_child_cancel
            || (receipt.update.state.status == "CANCELLED" && exact_active.is_none());
        receipt.matches(&input) && settlement_proven && exact_active.is_none_or(|active| {
            active.lease.path_revision == receipt.update.feedback.path_revision
                && active.lease.planner_runtime_generation == receipt.update.feedback.planner_runtime_generation
        })
    }).map(|receipt| receipt.update.clone());
    if let Some(receipt) = receipt {
        // Never borrow the current (possibly foreign) CancelTracker to rebuild this body.
        let lease = GoalLease {
            session_id: receipt.state.goal_id.clone(), owner_kind: receipt.feedback.owner_kind,
            parent_request_id: receipt.feedback.parent_request_id.clone(),
            goal_generation: receipt.feedback.goal_generation,
            producer_runtime_generation: receipt.producer_runtime_generation,
            path_revision: receipt.feedback.path_revision,
            planner_runtime_generation: receipt.feedback.planner_runtime_generation,
        };
        return SetGoalOutcome {
            state: receipt.state.clone(), lease: Some(lease),
            lease_origin: GoalLeaseOrigin::Installed, #[cfg(test)] dispatch_effect,
            publication: GoalPublicationPolicy::NoPublish,
            resolved_receipt: Some(receipt), ..SetGoalOutcome::default()
        };
    }
    let provisional = slot.goal.as_ref().is_some_and(|goal| {
        goal.lease == *attempted && goal.lease.path_revision == 0
            && goal.lease.planner_runtime_generation == 0 && slot.phase == NavPhase::Accepted
    });
    if provisional && allow_definite_rollback && dispatch_effect == GoalDispatchEffect::NotDispatched {
            slot.goal = None;
            slot.phase = NavPhase::Failed;
            slot.message = message.clone();
            return SetGoalOutcome {
                state: NavState { status: "FAILED".into(), message, goal_id: attempted.session_id.clone(),
                    distance_remaining, timestamp_ns: now_ns() },
                lease: Some(attempted.clone()), lease_origin: GoalLeaseOrigin::InstalledReleased,
                #[cfg(test)] dispatch_effect, ..SetGoalOutcome::default()
            };

    }
    if provisional {
        slot.phase = NavPhase::DispatchUnresolved;
        slot.message = message.clone();
    }
    let exact_active = slot.goal.as_ref().filter(|goal| goal.lease.identity_matches(attempted));
    let proof_pending = exact_active.is_some() && slot.phase.is_terminal();
    let diagnostic = if proof_pending { GoalSubmissionDiagnostic::TerminalProofPending }
        else { GoalSubmissionDiagnostic::PublishUnconfirmed };
    // A candidate PathEnvelope alone cannot reach any of these slot phases.
    let progress = exact_active.is_some() && !proof_pending
        && matches!(slot.phase, NavPhase::Accepted | NavPhase::Planning | NavPhase::Running);
    // ZEROED may already have retired the active slot before the original receipt
    // is retained. Preserve its actual execution identity, never its partial ZERO body.
    let lease = exact_active.map(|goal| goal.lease.clone())
        .unwrap_or_else(|| attempted.clone());
    SetGoalOutcome {
        state: NavState {
            status: if progress { slot.phase.as_str() } else { "UNCONFIRMED" }.into(),
            message: if progress { slot.message.clone() } else { message },
            goal_id: attempted.session_id.clone(), distance_remaining,
            // Slot has no full source body/timestamp. Do not invent a source watermark.
            timestamp_ns: 0,
        },
        lease: Some(lease), lease_origin: GoalLeaseOrigin::InstalledUnresolved,
        #[cfg(test)] dispatch_effect, diagnostic: Some(diagnostic),
        publication: GoalPublicationPolicy::NoPublish, ..SetGoalOutcome::default()
    }
}

/// 按精确目标清理当前目标，并发布一次取消。
pub async fn cancel(handle: &GeometryRuntimeHandle, state: &NavRuntimeState, input: CancelGoalInput) -> CancelOutcome {
    let (lease, generation, outcome) = {
        let _transaction_guard = state.transaction_gate.lock().await;
        let _map_goal_guard = handle.mapping_lifecycle_lock.lock().await;
        let mut slot = state.inner.write().await;
        let Some(active) = slot.goal.as_ref() else {
            return cancel_outcome_with_phase(NavPhase::Failed, input.goal_id, "CANCEL_REJECTED[GOAL_LEASE_MISSING]".into(), None);
        };
        let lease = active.lease.clone();
        if !input.targets_current() && (input.goal_id != lease.session_id || input.owner_kind != lease.owner_kind
            || input.parent_request_id != lease.parent_request_id || input.goal_generation != lease.goal_generation
            || input.producer_runtime_generation != lease.producer_runtime_generation) {
            return cancel_outcome_with_phase(NavPhase::Failed, input.goal_id, "CANCEL_REJECTED[GOAL_LEASE_MISMATCH]".into(), None);
        }
        let generation = match state.next_cancel_generation.fetch_update(AtomicOrdering::AcqRel, AtomicOrdering::Acquire,
            |current| current.checked_add(1).filter(|next| *next <= i64::MAX as u64)) {
            Ok(previous) => previous + 1,
            Err(_) => return cancel_outcome_with_phase(NavPhase::Failed, input.goal_id,
                "CANCEL_REJECTED[CANCEL_GENERATION_EXHAUSTED]".into(), None),
        };
        slot.goal = None;
        slot.path_candidate_goal = None;
        slot.path_candidates.clear();
        slot.display_path = None;
        slot.phase = NavPhase::Cancelled;
        slot.message = "导航已取消".into();
        let outcome = cancel_outcome_from_slot(&slot, lease.session_id.clone(), slot.message.clone(), Some(lease.clone()));
        (lease, generation, outcome)
    };
    match publish_cancel_frame(handle, &lease, generation).await {
        Ok(()) => outcome,
        Err(error) => cancel_outcome_with_phase(NavPhase::Failed, lease.session_id.clone(), error, Some(lease)),
    }
}

fn cancel_outcome_with_phase(
    phase: NavPhase,
    goal_id: String,
    message: String,
    lease: Option<GoalLease>,
) -> CancelOutcome {
    CancelOutcome {
        state: NavState {
            status: phase.as_str().to_string(),
            message,
            goal_id,
            distance_remaining: 0.0,
            timestamp_ns: now_ns(),
        },
        lease,
    }
}

fn cancel_outcome_from_slot(
    slot: &NavRuntimeSlot,
    goal_id: String,
    message: String,
    lease: Option<GoalLease>,
) -> CancelOutcome {
    CancelOutcome {
        state: NavState {
            status: slot.phase.as_str().to_string(),
            message,
            goal_id,
            distance_remaining: 0.0,
            timestamp_ns: now_ns(),
        },
        lease,
    }
}

async fn distance_to_target_from_state(
    shared_state: &Arc<RwLock<SharedState>>,
    target: &PoseState,
    spatial: bool,
) -> f32 {
    let state = shared_state.read().await;
    let dx = state.pose.x - target.x;
    let dy = state.pose.y - target.y;
    let dz = if spatial {
        state.pose.z - target.z
    } else {
        0.0
    };
    dx.hypot(dy).hypot(dz) as f32
}

/// 生成或保留 goal_id。空字符串时生成 UUID v4。
pub fn ensure_goal_id(provided: &str) -> String {
    let trimmed = provided.trim();
    if trimmed.is_empty() {
        Uuid::new_v4().to_string()
    } else {
        trimmed.to_string()
    }
}

/// 把 readiness JSON 序列化成 gRPC `NavigationReadiness.reason` 字符串。
pub fn readiness_to_reason(detail: &NavigationReadinessDetail) -> String {
    serde_json::to_string(detail).unwrap_or_else(|e| {
        format!(
            "{{\"ready\":{},\"reason\":\"serialization_error:{}\"}}",
            detail.ready, e,
        )
    })
}

#[cfg(test)]
/// 把 readiness JSON 解析回来(Python 侧也用,这里给 Rust 单元测试用)。
pub fn readiness_from_reason(reason: &str) -> Option<NavigationReadinessDetail> {
    serde_json::from_str(reason).ok()
}

// ============================================================================
// 内部 helper
// ============================================================================

/// 当前墙钟(ns),失败时回退 0。
fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

/// Phase 4:把 `/topic#pkg.Msg` typed pattern 解析成 `LcmTopic` 与 suffix。
///
/// `LcmTopic::new` 强约束 channel 字符串不能含 `#`(typed suffix 分隔符);
/// `channel_registry::resolve_channel` 默认值 = `/cmd_vel#geometry_msgs.Twist`
/// 这种 typed pattern,所以需要先 split 再喂给 `LcmTopic::new`。
///
/// **Phase 4 提升为 `pub fn`**：lcm_runtime.rs / server.rs 等模块也用此
/// helper 解析 typed channel；原 `fn` 仅在 nav_runtime.rs 内部使用。
pub fn parse_channel_with_suffix(channel: &str) -> Result<(LcmTopic, Option<String>), String> {
    LcmTopic::parse_subscription_pattern(channel).map_err(|e| {
        format!(
            "LcmTopic::parse_subscription_pattern({}) 失败: {}",
            channel, e
        )
    })
}

fn resolve_strict_typed_binding(
    platform_key: &str,
    default_channel: &str,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let channel = channel_registry::resolve_channel(platform_key, default_channel);
    strict_typed_binding(&channel, expected_type)
}

fn strict_typed_binding(
    channel: &str,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, suffix) = parse_channel_with_suffix(channel)?;
    let suffix =
        suffix.ok_or_else(|| format!("typed channel {channel} is missing #message type"))?;
    let actual = LcmMessageType::new(&suffix)
        .map_err(|error| format!("invalid typed channel {channel}: {error}"))?;
    let expected = LcmMessageType::new(expected_type)
        .map_err(|error| format!("invalid expected type {expected_type}: {error}"))?;
    if actual != expected {
        return Err(format!(
            "legacy/raw typed channel rejected: {channel}; expected #{}",
            expected.fqn()
        ));
    }
    Ok((topic, actual))
}

/// Resolve the strict versioned path channel consumed by geometry runtime.
pub fn goal_envelope_binding() -> Result<(LcmTopic, LcmMessageType), String> {
    resolve_strict_typed_binding(
        GOAL_CHANNEL_PLATFORM,
        GOAL_CHANNEL_DEFAULT,
        "nuwax_msgs.GoalEnvelope",
    )
}

/// Resolve the strict versioned path channel consumed by geometry runtime.
pub fn path_envelope_binding() -> Result<(LcmTopic, LcmMessageType), String> {
    let channel = channel_registry::resolve_strict_channel(
        PATH_CHANNEL_PLATFORM,
        PATH_CHANNEL_DEFAULT,
    )
    .map_err(|error| error.to_string())?;
    strict_typed_binding(&channel, "nuwax_msgs.PathEnvelope")
}

async fn publish_encoded_goal_frame_from_transport(
    transport: &LcmTransport,
    payload: Vec<u8>,
) -> Result<(), TransportError> {
    let (topic, msg_type) = goal_envelope_binding().map_err(|error|
        TransportError::NotDispatched(Box::new(TransportError::Topic(error))))?;
    let frame: RawLcmFrame =
        typed_codec::build_typed_frame(topic, msg_type, payload, now_ns() & 0x0000_FFFF_FFFF_FFFF);
    transport.publish(frame).await
}

/// publish `/nav/cancel` typed frame。
async fn publish_cancel_frame(
    handle: &GeometryRuntimeHandle,
    lease: &GoalLease,
    cancel_generation: u64,
) -> Result<(), String> {
    let (topic, msg_type) = resolve_strict_typed_binding(
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel#nuwax_msgs.CancelNavigation",
        "nuwax_msgs.CancelNavigation",
    )?;
    let payload = typed_codec::encode_nav_cancel_payload(
        &lease.session_id,
        0,
        cancel_generation,
        lease.owner_kind,
        &lease.parent_request_id,
        lease.goal_generation,
        lease.producer_runtime_generation,
    )
    .map_err(|error| format!("CancelNavigation LCM encode failed: {error}"))?;
    let frame: RawLcmFrame =
        typed_codec::build_typed_frame(topic, msg_type, payload, seqno_for(handle).await);
    handle
        .transport
        .publish(frame)
        .await
        .map_err(|e| format!("LcmTransport::publish 失败: {}", e))
}

/// LCM publish sequence number:从 broadcast tx 简单递增。
///
/// LCM 协议只要求单调递增;我们不需要严格递序,只在每次 publish 时 +1。
async fn seqno_for(_handle: &GeometryRuntimeHandle) -> u64 {
    // 简化:用 system clock ns 末 48 位作为 seqno(LCM seqno 是 u64 但通常
    // 只看低 32/48 位,这里保险取低 48 位以保留时间序)。
    now_ns() & 0x0000_FFFF_FFFF_FFFF
}

// ============================================================================
// 单元测试
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn surface_distance_keeps_vertical_separation_at_same_xy() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let target = PoseState {
            z: 3.0,
            ..PoseState::default()
        };
        assert_eq!(
            distance_to_target_from_state(&state, &target, true).await,
            3.0
        );
        assert_eq!(
            distance_to_target_from_state(&state, &target, false).await,
            0.0
        );
    }
    use crate::lcm_runtime::{MapState, SharedState};
    use async_trait::async_trait;
    use nuwax_transport::{
        LcmBackend, LcmFrameStream, LcmTransport, TransportError, TransportResult,
    };
    use std::collections::VecDeque;
    use std::ffi::OsString;
    use std::sync::Mutex;
    use std::time::Duration;
    use tokio::sync::Semaphore;
    use tokio::time::sleep;

    static NAV_PROFILE_ENV_LOCK: Mutex<()> = Mutex::new(());
    static NEXT_SEQUENCE_TEST_PATH: std::sync::atomic::AtomicU64 =
        std::sync::atomic::AtomicU64::new(0);

    struct NavProfileEnvRestore(Option<OsString>, Option<OsString>);

    impl Drop for NavProfileEnvRestore {
        fn drop(&mut self) {
            match self.0.take() {
                Some(value) => std::env::set_var("NUWAX_NAV_PROFILE", value),
                None => std::env::remove_var("NUWAX_NAV_PROFILE"),
            }
            match self.1.take() {
                Some(value) => std::env::set_var("NUWAX_GEOMETRY_PIPELINE", value),
                None => std::env::remove_var("NUWAX_GEOMETRY_PIPELINE"),
            }
        }
    }

    fn set_nav_profile_for_test(value: Option<&str>) -> NavProfileEnvRestore {
        let previous = std::env::var_os("NUWAX_NAV_PROFILE");
        match value {
            Some(value) => std::env::set_var("NUWAX_NAV_PROFILE", value),
            None => std::env::remove_var("NUWAX_NAV_PROFILE"),
        }
        let previous_pipeline = std::env::var_os("NUWAX_GEOMETRY_PIPELINE");
        std::env::set_var("NUWAX_GEOMETRY_PIPELINE", if value == Some("simple-2d") { "slam_2d" } else { "lio_3d" });
        NavProfileEnvRestore(previous, previous_pipeline)
    }

    fn sequence_test_path(label: &str) -> std::path::PathBuf {
        std::env::temp_dir()
            .join(format!(
                "nuwax-nav-sequence-{}-{}-{label}",
                std::process::id(),
                NEXT_SEQUENCE_TEST_PATH.fetch_add(1, AtomicOrdering::Relaxed),
            ))
            .join("sequence_state.json")
    }

    #[test]
    fn captured_pipeline_binds_navigation_before_epoch_allocation() {
        let path = sequence_test_path("captured-pipeline");
        let store = SequenceStateStore::open(&path).expect("独立序号存储");
        let mut previous_generation = 0;
        for pipeline in [
            nuwax_nav_stack::ProductionPipeline::Slam2D,
            nuwax_nav_stack::ProductionPipeline::Lio3D,
        ] {
            let state = NavRuntimeState::try_from_sequence_store_for_pipeline(&store, pipeline)
                .expect("使用捕获的完整链");
            assert_eq!(state.configured_profile(), pipeline.navigation_profile().as_str());
            assert!(state.producer_runtime_generation() > previous_generation);
            previous_generation = state.producer_runtime_generation();
        }
        drop(store);
        std::fs::remove_dir_all(path.parent().unwrap()).expect("清理测试序号存储");
    }

    #[test]
    fn producer_epoch_is_durably_newer_after_runtime_restart() {
        let path = sequence_test_path("restart");
        let first_store = SequenceStateStore::open(&path).expect("first sequence store");
        let first =
            NavRuntimeState::try_from_sequence_store(&first_store).expect("first producer epoch");
        let first_generation = first.producer_runtime_generation();
        assert_eq!(first_generation, 1);
        drop(first);
        drop(first_store);

        let restarted_store = SequenceStateStore::open(&path).expect("restarted sequence store");
        let restarted = NavRuntimeState::try_from_sequence_store(&restarted_store)
            .expect("restarted producer epoch");
        assert!(
            restarted.producer_runtime_generation() > first_generation,
            "restarted geometry producer must sort after the prior incarnation"
        );
    }

    #[tokio::test]
    async fn goal_generation_overflow_fails_without_advancing_nav_state() {
        let state = NavRuntimeState::from_env();
        state
            .next_goal_generation
            .store(i64::MAX as u64, AtomicOrdering::Release);
        assert!(state.reserve_goal_generation(0).is_err());
        assert!(state.reserve_goal_generation(u64::MAX).is_err());
        assert_eq!(
            state.next_goal_generation.load(AtomicOrdering::Acquire),
            i64::MAX as u64
        );
        let slot = state.inner.read().await;
        assert_eq!(slot.last_goal_generation, 0);
        assert!(slot.goal.is_none());
        assert_eq!(slot.phase, NavPhase::Idle);
    }

    #[test]
    fn committed_map_requires_complete_receipt_without_content_digest() {
        let identity = schema::CommittedMapIdentity {
            session_id: "map-session".into(), session_generation: 1, revision: 2,
            commit_boot_nonce: [3; 16], commit_sequence: 4, source_timestamp_ns: 0,
            frame_id: "map".into(),
        };
        assert!(validate_committed_map_identity(&identity).is_ok());
        for field in 0..3 {
            let mut invalid = identity.clone();
            match field {
                0 => invalid.commit_boot_nonce = [0; 16],
                1 => invalid.commit_sequence = 0,
                _ => invalid.source_timestamp_ns = -1,
            }
            assert!(validate_committed_map_identity(&invalid).is_err());
        }
    }

    fn map_revision_fixture(
        map: &MapState,
        session_id: &str,
        map_name: &str,
    ) -> crate::lcm_runtime::MapRevisionEvent {
        crate::lcm_runtime::MapRevisionEvent::from_map_for_tests(
            map,
            &crate::mapping_adapter::MappingSessionSnapshot {
                phase: crate::mapping_adapter::MappingSessionPhase::Active,
                session_id: session_id.into(),
                map_name: map_name.into(),
                session_generation: 1,
                revision: map.revision,
                active: true,
                idempotent_replay: false,

                mapping_backend_kind: "voxel".into(),
            },
        )
    }

    async fn install_committed_map_fixture(
        handle: &GeometryRuntimeHandle,
        frame_id: &str,
        with_pose: bool,
    ) {
        let timestamp_ns = now_ns();
        let map = MapState {
            width: 10,
            height: 10,
            resolution: 0.5,
            origin_x: -2.5,
            origin_y: -2.5,
            frame_id: frame_id.to_string(),
            revision: 1,
            timestamp_ns,
            data: vec![0; 100],
        };
        let event = map_revision_fixture(&map, "nav-fixture-session", "nav-fixture-map");
        {
            let mut shared = handle.state.write().await;
            shared.map = map;
            if with_pose {
                shared.pose = PoseState {
                    qw: 1.0,
                    frame_id: frame_id.to_string(),
                    timestamp_ns,
                    ..PoseState::default()
                };
            }
        }
        let _ = handle.map_revision_tx.send(event.clone());
        handle
            .map_revision_latest_tx
            .send_replace(Some(event.clone()));
        *handle
            .committed_map_authority
            .lock()
            .expect("committed map fixture authority") = Some(
            crate::lcm_runtime::committed_map_identity_from_revision(&event)
                .expect("committed map fixture identity"),
        );
        let visible = handle
            .map_revision_latest_tx
            .borrow()
            .clone()
            .expect("committed map fixture must be visible before returning");
        assert_eq!(visible.revision, event.revision);
        assert_eq!(visible.frame_id, event.frame_id);
        assert_eq!(visible.receipt, event.receipt);
        assert_eq!(visible.raw_representation, event.raw_representation);
    }

    /// 构造一个测试 handle:test backend(走 real-lcm + test_lcm_url 隔离端口)+ 一个非空 pose 与 map。
    ///
    /// 注意:不能直接构造 `LcmTransport`(其字段非 pub);这里用 `build_handle_for_tests`
    /// 构造,backend 走 real-lcm,LCM publish 走隔离端口。
    /// 测试 handle 走真实 backend，默认 true。
    async fn make_handle() -> GeometryRuntimeHandle {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("build test handle");
        install_committed_map_fixture(&handle, "map", true).await;
        handle
    }

    #[tokio::test]
    async fn unlocalized_asset_blocks_even_prevalidated_goal_and_ready_observations() {
        let handle = make_handle().await;
        observe_ready_execution_chain(&handle.nav).await;
        let context = NavAdmissionContext::new(handle.mapping_lifecycle_lock.clone(),
            handle.committed_map_authority.clone(),handle.map_revision_latest_tx.subscribe())
            .with_mapping_adapter(handle.mapping_adapter.clone());
        let source = handle.committed_map_authority.lock().unwrap().clone().unwrap();
        let frame = context.map_frame_generation();
        assert!(compute_readiness(&handle,&handle.nav).await.ready);
        handle.mapping_adapter.as_ref().unwrap().lock().checkpoint_localization_required = true;
        assert!(!compute_readiness(&handle,&handle.nav).await.ready);
        for verified in [false,true] {
            assert_eq!(goal_map_rejection(&context,&source,frame,"before-localized",verified).unwrap().retry_reason,
                Some(SetGoalRetryReason::NavigationNotReady(NavigationReadinessCode::PlannerNotReady)));
        }
    }

    #[test]
    fn goal_map_rejection_previous_source_uses_authority_without_transport() {
        // Exercise the production admission function without GeometryRuntimeHandle,
        // LcmTransport, socket readiness, or a running mapping/navigation backend.
        let current = schema::CommittedMapIdentity {
            session_id: "surface-map-admission".into(),
            session_generation: 1,
            revision: 3,
            commit_boot_nonce: [7; 16],
            commit_sequence: 3,
            source_timestamp_ns: 3,
            frame_id: "map".into(),
        };
        let authority = Arc::new(StdMutex::new(Some(current.clone())));
        let (_revision_tx, revision_rx) = tokio::sync::watch::channel(None);
        let adapter = Arc::new(parking_lot::Mutex::new(
            crate::mapping_adapter::MappingAdapter::new(crate::mapping_adapter::MappingAdapterConfig {
                resolution: 0.1,
                initial_safe_radius_meters: 0.0,
            }),
        ));
        let context = NavAdmissionContext::new(
            Arc::new(tokio::sync::Mutex::new(())),
            authority.clone(),
            revision_rx,
        )
        .with_mapping_adapter(Some(adapter.clone()))
        .with_previous_map_revisions(true);
        let frame = context.map_frame_generation();
        let mut source = current.clone();
        source.revision = 2;
        source.commit_sequence = 2;
        source.source_timestamp_ns = 2;
        let original = source.clone();
        for prevalidated in [false, true] {
            assert!(goal_map_rejection(&context, &current, frame, "exact", prevalidated).is_none());
            assert!(goal_map_rejection(&context, &source, frame, "previous", prevalidated).is_none());
            for field in ["session", "session_generation", "frame_id", "boot"] {
                let mut foreign = source.clone();
                match field {
                    "session" => foreign.session_id.push_str("-replacement"),
                    "session_generation" => foreign.session_generation += 1,
                    "frame_id" => foreign.frame_id = "other-map".into(),
                    _ => foreign.commit_boot_nonce[0] ^= 1,
                }
                assert_eq!(
                    goal_map_rejection(&context, &foreign, frame, field, prevalidated)
                        .unwrap()
                        .reject_reason,
                    Some(SetGoalRejectReason::MapFrameChanged),
                    "{field} must not be qualified by an older revision"
                );
            }
            for wrong_frame in [-1, frame + 1] {
                assert_eq!(
                    goal_map_rejection(&context, &source, wrong_frame, "frame", prevalidated)
                        .unwrap()
                        .reject_reason,
                    Some(SetGoalRejectReason::MapFrameChanged)
                );
            }
            let mut conflict = source.clone();
            conflict.revision = current.revision;
            assert!(
                goal_map_rejection(&context, &conflict, frame, "conflict", prevalidated)
                    .unwrap()
                    .retry_reason
                    .is_none()
            );
            let mut future = current.clone();
            future.revision += 1;
            future.commit_sequence += 1;
            assert!(goal_map_rejection(&context, &future, frame, "future", prevalidated).is_some());
        }
        for strict in [
            context.clone().with_previous_map_revisions(false),
            context.clone().with_mapping_adapter(None),
        ] {
            let frame = strict.map_frame_generation();
            assert_eq!(
                goal_map_rejection(&strict, &source, frame, "strict", false)
                    .unwrap()
                    .retry_reason,
                Some(SetGoalRetryReason::MapRevisionAdvanced)
            );
            // The existing explicitly prevalidated path is independent of policy.
            assert!(goal_map_rejection(&strict, &source, frame, "verified", true).is_none());
        }
        adapter.lock().checkpoint_localization_required = true;
        for prevalidated in [false, true] {
            assert_eq!(
                goal_map_rejection(&context, &source, frame, "unlocalized", prevalidated)
                    .unwrap()
                    .retry_reason,
                Some(SetGoalRetryReason::NavigationNotReady(
                    NavigationReadinessCode::PlannerNotReady
                ))
            );
        }
        adapter.lock().checkpoint_localization_required = false;
        assert_eq!(
            source, original,
            "admission must never relabel old provenance"
        );
        assert_eq!(authority.lock().unwrap().as_ref(), Some(&current));
        *authority.lock().unwrap() = None;
        assert_eq!(
            goal_map_rejection(&context, &source, frame, "missing-authority", false)
                .unwrap()
                .retry_reason,
            Some(SetGoalRetryReason::MapRevisionAdvanced)
        );
    }

    #[tokio::test]
    async fn verified_source_revision_can_advance_without_losing_coordinate_fence() {
        let handle = make_handle().await;
        let mut current = handle.committed_map_authority.lock().unwrap().clone().unwrap();
        current.revision = 3;
        current.commit_sequence = 3;
        *handle.committed_map_authority.lock().unwrap() = Some(current.clone());
        let mut source = current.clone(); source.revision = 2; source.commit_sequence = 2;
        let context = NavAdmissionContext::new(handle.mapping_lifecycle_lock.clone(), handle.committed_map_authority.clone(), handle.map_revision_latest_tx.subscribe())
            .with_mapping_adapter(handle.mapping_adapter.clone()).with_previous_map_revisions(true);
        let frame = context.map_frame_generation();
        assert!(goal_map_rejection(&context, &source, frame, "old-source", false).is_none());
        let mut previous_boot = source.clone();
        previous_boot.commit_boot_nonce[0] ^= 1;
        for prevalidated in [false, true] {
            assert_eq!(goal_map_rejection(&context, &previous_boot, frame, "old-boot", prevalidated)
                .unwrap().reject_reason, Some(SetGoalRejectReason::MapFrameChanged));
        }
        assert_eq!(goal_map_rejection(&context, &source, frame + 1, "wrong-frame", false).unwrap().reject_reason, Some(SetGoalRejectReason::MapFrameChanged));
        source.revision = current.revision;
        assert!(goal_map_rejection(&context, &source, frame, "mutated-revision", false).unwrap().retry_reason.is_none());
        source.revision = 2;
        let strict = context.with_previous_map_revisions(false);
        assert_eq!(goal_map_rejection(&strict, &source, frame, "refresh-source", false).unwrap().retry_reason, Some(SetGoalRetryReason::MapRevisionAdvanced));
    }

    async fn set_exploration_goal_for_test(
        handle: &GeometryRuntimeHandle,
        state: &NavRuntimeState,
        input: SetGoalInput,
        boundary: Option<GoalBoundaryConstraint>,
    ) -> SetGoalOutcome {
        let goal_id = input.goal_id.clone();
        let admission = NavAdmissionContext::new(
            handle.mapping_lifecycle_lock.clone(),
            handle.committed_map_authority.clone(),
            handle.map_revision_latest_tx.subscribe(),
        );
        let io = NavAdmissionIo::new(
            handle.state.clone(),
            handle.transport.clone(),
            handle.transport_backend_name,
            handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
);
        let producer_epoch = state.producer_runtime_generation();
        if producer_epoch == 0 {
            return rejected_goal_intent(goal_id, "invalid fixture producer epoch");
        }
        let boundary_owner = Arc::new(crate::explore_runtime::ExploreStartConfig {
            boundary: boundary.as_ref().map(|boundary| nuwax_exploration::ExplorationBoundary::from_vertices(
                boundary.points.iter().map(|point| nuwax_nav_types_messages::geometry::Point2 { x: point[0], y: point[1] }).collect()
            ).expect("synthetic boundary owner")),
            ..Default::default()
        });
        let intent = match goal_intent_from_set_goal(&admission, state, input, boundary).await {
            Ok(intent) => intent,
            Err(reason) => return rejected_goal_intent(goal_id, reason),
        };
        admit_goal_intent(
            &admission,
            &io,
            state,
            intent,
            GoalProducerEpochAuthority::Wavefront {
                boundary_owner,
                current_session_generation: Arc::new(AtomicU64::new(producer_epoch)),
            },
        )
        .await
    }

    #[derive(Clone)]
    struct BlockingPublishBackend {
        frames: Arc<Mutex<Vec<RawLcmFrame>>>,
        publish_gate: Arc<Semaphore>,
        outcomes: Arc<Mutex<VecDeque<Option<TransportError>>>>,
    }

    impl BlockingPublishBackend {
        fn new(outcomes: impl IntoIterator<Item = Option<String>>) -> Self {
            Self {
                frames: Arc::new(Mutex::new(Vec::new())),
                publish_gate: Arc::new(Semaphore::new(0)),
                outcomes: Arc::new(Mutex::new(outcomes.into_iter().map(|outcome| outcome.map(TransportError::LcmBackend)).collect())),
            }
        }

        async fn wait_for_publish_count(&self, expected: usize) {
            tokio::time::timeout(Duration::from_secs(1), async {
                while self.frames.lock().expect("published frames").len() < expected {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .unwrap_or_else(|_| panic!("publish count did not reach {expected}"));
        }

        fn publish_count(&self) -> usize {
            self.frames.lock().expect("published frames").len()
        }

        fn published_message_types(&self) -> Vec<String> {
            self.frames
                .lock()
                .expect("published frames")
                .iter()
                .map(|frame| frame.msg_type.fqn().to_string())
                .collect()
        }

        fn release_one(&self) {
            self.publish_gate.add_permits(1);
        }
    }

    #[async_trait]
    impl LcmBackend for BlockingPublishBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            self.frames.lock().expect("published frames").push(frame);
            let permit = self
                .publish_gate
                .acquire()
                .await
                .map_err(|_| TransportError::ChannelClosed)?;
            permit.forget();
            match self
                .outcomes
                .lock()
                .expect("publish outcomes")
                .pop_front()
                .flatten()
            {
                Some(error) => Err(error),
                None => Ok(()),
            }
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::pending())
        }

        fn name(&self) -> &str {
            "blocking-test"
        }
    }

    async fn make_blocking_handle(
        outcomes: impl IntoIterator<Item = Option<String>>,
    ) -> (GeometryRuntimeHandle, BlockingPublishBackend) {
        let mut handle = make_handle().await;
        let backend = BlockingPublishBackend::new(outcomes);
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        (handle, backend)
    }

    // Explicit available external port for admission unit tests. The production
    // owner/codec still run; this fixture neither opens nor claims real LCM.
    async fn make_unit_blocking_handle(
        outcomes: impl IntoIterator<Item = Option<String>>,
    ) -> (GeometryRuntimeHandle, BlockingPublishBackend, crate::lcm_runtime::GeometryTestTaskOwner) {
        let backend = BlockingPublishBackend::new(outcomes);
        let (mut handle, owner) = crate::lcm_runtime::build_handle_for_tests_with_unit_transport(
            true, None, LcmTransport::new(Arc::new(backend.clone())),
        ).await.expect("in-process admission fixture");
        handle.lcm_backend_available = true;
        install_committed_map_fixture(&handle, "map", true).await;
        (handle, backend, owner)
    }

    #[tokio::test]
    async fn natural_retirement_records_cancel_authority_at_commit_only() {
        {
            let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
            let lease = test_lease("retirement-unknown-tracker", 1);
            state.replace_active_goal_for_test(test_active_goal_with_lease(lease.clone())).await;
            let receipt = NavigationUpdate {
                producer_runtime_generation: lease.producer_runtime_generation,
                state: NavState { goal_id: lease.session_id.clone(), status: "FAILED".into(),
                    ..NavState::default() },
                feedback: crate::lcm_runtime::NavigationFeedback {
                    owner_kind: lease.owner_kind, parent_request_id: lease.parent_request_id.clone(),
                    goal_generation: lease.goal_generation,
                    producer_runtime_generation: lease.producer_runtime_generation,
                    path_revision: lease.path_revision,
                    planner_runtime_generation: lease.planner_runtime_generation,
                    actuator_zeroed: true, zero_ack_id: "measured-stop".into(),
                    zero_applied_monotonic_ns: 1, ..Default::default()
                },
            };
            state.retain_terminal_receipt(&receipt).await;
            assert!(state.active_goal_lease().await.is_none(),
                "retirement itself keeps the existing measured full-lease rule");
            assert!(state.terminal_receipt_for(&cancel_input(&lease)).await.is_some());
            assert!(state.settled_without_child_cancel_terminal_for(&cancel_input(&lease)).await.is_some());
        }
    }

    fn test_goal_input(goal_id: &str) -> SetGoalInput {
        SetGoalInput {
            options: Default::default(),
            goal_id: goal_id.to_string(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: format!("request-{goal_id}"),
            goal_generation: 0,
            motion_admission: test_motion_admission(),
            frame_id: "map".to_string(),
            target: PoseState {
                qw: 1.0,
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                ..PoseState::default()
            },
        }
    }

    fn test_lease(goal_id: &str, generation: u64) -> GoalLease {
        GoalLease {
            session_id: goal_id.to_string(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: format!("request-{goal_id}"),
            goal_generation: generation,
            producer_runtime_generation: 1,

            path_revision: 0,
            planner_runtime_generation: 0,
        }
    }

    fn test_active_goal(goal_id: &str, generation: u64) -> ActiveGoal {
        test_active_goal_with_lease(test_lease(goal_id, generation))
    }

    fn test_active_goal_with_lease(lease: GoalLease) -> ActiveGoal {
        ActiveGoal {
            motion_admission: test_motion_admission(),
            options: Default::default(), producer: "public-navigation".into(), producer_epoch: 1,
            boundary_owner: None,
            id: lease.session_id.clone(),
            frame_id: "map".to_string(),
            target: PoseState {
                qw: 1.0,
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                ..PoseState::default()
            },
            lease,
        }
    }

    fn cancel_input(lease: &GoalLease) -> CancelGoalInput {
        CancelGoalInput {
            goal_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            producer_runtime_generation: lease.producer_runtime_generation,
        }
    }

    fn sealed_path(lease: &GoalLease, path_revision: u64, x: f64) -> schema::PathEnvelope {
        let path = schema::PathEnvelope {
            route_kind: 0,
            schema_version: schema::PATH_ENVELOPE_SCHEMA_VERSION,
            session_id: lease.session_id.clone(),

            goal_generation: lease.goal_generation as i64,

            producer_runtime_generation: lease.producer_runtime_generation as i64,

            planner_runtime_generation: 1,
            path_revision: path_revision as i64,

            path: schema::Path {
                header: schema::Header {
                    frame_id: "map".to_string(),
                    ..schema::Header::default()
                },
                poses: vec![schema::PoseStamped {
                    header: schema::Header {
                        frame_id: "map".to_string(),
                        ..schema::Header::default()
                    },
                    pose: schema::Pose {
                        position: [x, 0.0, 0.0],
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                }],
            },
            ..schema::PathEnvelope::default()
        };
        schema::validate_path_envelope(&path).expect("seal test path");
        path
    }

    #[test]
    fn resolve_profile_uses_default_when_unset() {
        let _lock = NAV_PROFILE_ENV_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let _restore = set_nav_profile_for_test(None);
        assert_eq!(resolve_profile_from_env().unwrap(), PROFILE_MLS3D);
    }

    #[test]
    fn resolve_profile_uses_set_value() {
        let _lock = NAV_PROFILE_ENV_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let _restore = set_nav_profile_for_test(Some("simple-2d"));
        assert_eq!(resolve_profile_from_env().unwrap(), PROFILE_SIMPLE_2D);
    }

    #[test]
    fn resolve_profile_rejects_unknown() {
        let _lock = NAV_PROFILE_ENV_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let _restore = set_nav_profile_for_test(Some("hologram"));
        assert!(resolve_profile_from_env().is_err());
    }

    #[test]
    fn dispatch_unresolved_phase_is_local_nonterminal_only() {
        assert_eq!(NavPhase::DispatchUnresolved.as_str(), "UNCONFIRMED");
        assert!(!NavPhase::DispatchUnresolved.is_terminal());
        assert_eq!(NavPhase::from_stack_status("UNCONFIRMED"), None);
    }

    #[tokio::test]
    async fn generic_goal_error_keeps_exact_owner_replay_and_cancel_without_second_goal() {
        let (handle, backend, owner) = make_unit_blocking_handle([
            Some("opaque backend error, no source-point proof".to_owned()), None,
        ]).await;
        let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
        observe_ready_execution_chain(&state).await;
        let mut input = test_goal_input("generic-error-keeps-owner");
        input.goal_generation = 61;
        backend.release_one();
        let unknown = set_goal(&handle, &state, input.clone()).await;
        assert_eq!(unknown.state.status, "UNCONFIRMED");
        assert_eq!(unknown.state.timestamp_ns, 0);
        assert_eq!(unknown.lease_origin, GoalLeaseOrigin::InstalledUnresolved);
        assert_eq!(unknown.dispatch_effect, GoalDispatchEffect::EffectUnconfirmed);
        assert_eq!(unknown.publication, GoalPublicationPolicy::NoPublish);
        assert_eq!(unknown.diagnostic, Some(GoalSubmissionDiagnostic::PublishUnconfirmed));
        assert!(!unknown.accepted());
        let lease = unknown.lease.clone().unwrap();
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));
        assert!(state.publish_in_flight_for_test().unwrap().is_empty());
        let replay = set_goal(&handle, &state, input).await;
        assert_eq!(replay.state.status, "UNCONFIRMED");
        assert_eq!(replay.lease_origin, GoalLeaseOrigin::ExistingUnresolved);
        assert_eq!(replay.dispatch_effect, GoalDispatchEffect::NotAttempted);
        assert_eq!(replay.publication, GoalPublicationPolicy::NoPublish);
        let mut foreign = test_goal_input("new-command-replaces-unknown");
        foreign.goal_generation = 62;
        backend.release_one();
        let replacement = set_goal(&handle, &state, foreign).await;
        assert!(replacement.accepted(), "{}", replacement.state.message);
        let successor = replacement.lease.unwrap();
        assert_eq!(state.active_goal_lease().await, Some(successor.clone()));
        let old_cancel = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(old_cancel.state.status, "FAILED");
        assert_eq!(backend.publish_count(), 2);
        backend.release_one();
        assert_eq!(cancel(&handle, &state, cancel_input(&successor)).await.state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
        assert_eq!(backend.published_message_types(), vec![
            "nuwax_msgs.GoalEnvelope".to_string(), "nuwax_msgs.GoalEnvelope".to_string(), "nuwax_msgs.CancelNavigation".to_string(),
        ]);
        drop(handle);
        owner.finish().await.unwrap();
    }

    #[tokio::test]
    async fn explicit_not_dispatched_unit_fixture_releases_only_provisional_owner() {
        let (handle, backend, owner) = make_unit_blocking_handle([]).await;
        backend.outcomes.lock().unwrap().push_back(Some(TransportError::NotDispatched(
            Box::new(TransportError::LcmBackend("explicit pre-dispatch fixture".into())))));
        backend.release_one();
        let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
        observe_ready_execution_chain(&state).await;
        let mut input = test_goal_input("definite-unit-control");
        input.goal_generation = 61;
        let rejected = set_goal(&handle, &state, input).await;
        assert_eq!(rejected.state.status, "FAILED");
        assert_eq!(rejected.lease_origin, GoalLeaseOrigin::InstalledReleased);
        assert_eq!(rejected.dispatch_effect, GoalDispatchEffect::NotDispatched);
        assert!(state.active_goal_lease().await.is_none());
        let cancel = cancel(&handle, &state, cancel_input(rejected.lease.as_ref().unwrap())).await;
        assert_eq!(cancel.state.status, "FAILED");
        assert!(cancel.state.message.contains("GOAL_LEASE_MISSING"));
        assert_eq!(backend.publish_count(), 1);
        drop(handle);
        owner.finish().await.unwrap();
    }

    #[tokio::test]
    async fn late_generic_goal_error_distinguishes_path_candidate_from_correlated_running() {
        for correlated_running in [false, true] {
            let (handle, backend, owner) = make_unit_blocking_handle([Some("late unknown".into())]).await;
            let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
            observe_ready_execution_chain(&state).await;
            let mut input = test_goal_input("candidate-versus-confirmed-path");
            input.goal_generation = 61;
            let mut submission = Box::pin(set_goal(&handle, &state, input));
            tokio::select! {
                _ = backend.wait_for_publish_count(1) => {},
                _ = &mut submission => panic!("submission must wait at the original backend gate"),
            }
            let original = state.active_goal_lease().await.unwrap();
            let path = sealed_path(&original, 1, 2.0);
            let candidate = state.observe_path_envelope(&path).await.unwrap();
            assert_eq!(state.active_goal_lease().await.as_ref(), Some(&original));
            if correlated_running {
                state.apply_correlated_navigation_state(
                    NavState { status: "RUNNING".into(), goal_id: original.session_id.clone(),
                        message: "actual correlated source progress".into(), timestamp_ns: 7, ..Default::default() },
                    &original.session_id, original.owner_kind, &original.parent_request_id,
                    original.goal_generation, candidate.path_revision, original.producer_runtime_generation,
                    candidate.planner_runtime_generation, None, None,
                ).await.unwrap();
            }
            backend.release_one();
            let outcome = submission.await;
            assert_eq!(outcome.lease_origin, GoalLeaseOrigin::InstalledUnresolved);
            assert_eq!(outcome.publication, GoalPublicationPolicy::NoPublish);
            assert_eq!(outcome.diagnostic, Some(GoalSubmissionDiagnostic::PublishUnconfirmed));
            assert_eq!(outcome.state.timestamp_ns, 0);
            if correlated_running {
                assert_eq!(outcome.state.status, "RUNNING");
                assert!(outcome.accepted());
                assert_eq!(outcome.lease.as_ref(), Some(&candidate));
                assert_eq!(state.inner.read().await.phase, NavPhase::Running);
            } else {
                assert_eq!(outcome.state.status, "UNCONFIRMED");
                assert!(!outcome.accepted());
                assert_eq!(outcome.lease.as_ref(), Some(&original));
                assert_eq!(state.inner.read().await.phase, NavPhase::DispatchUnresolved);
            }
            assert_eq!(backend.publish_count(), 1);
            drop(handle);
            owner.finish().await.unwrap();
        }
    }

    #[tokio::test]
    async fn superseded_finish_keeps_installed_attempt_and_does_not_clear_foreign_owner() {
        let (handle, backend, owner) = make_unit_blocking_handle([Some("unknown completion".into())]).await;
        let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
        observe_ready_execution_chain(&state).await;
        let mut input = test_goal_input("superseded-original");
        input.goal_generation = 61;
        let mut submission = Box::pin(set_goal(&handle, &state, input));
        tokio::select! {
            _ = backend.wait_for_publish_count(1) => {},
            _ = &mut submission => panic!("submission must wait at the original backend gate"),
        }
        let original = state.active_goal_lease().await.unwrap();
        let flight = state.publish_in_flight.lock().unwrap()[0].clone();
        assert!(state.finish_publish(PublishKind::Goal, &original, original.goal_generation, flight.attempt));
        let mut foreign = test_active_goal_with_lease(original.clone());
        foreign.lease.session_id = "foreign-new-owner".into();
        foreign.lease.goal_generation += 1;
        foreign.id = foreign.lease.session_id.clone();
        let foreign_lease = foreign.lease.clone();
        state.replace_active_goal_for_test(foreign).await;
        let cancel_before = state.cancel_generation();
        backend.release_one();
        let outcome = submission.await;
        assert_eq!(outcome.lease_origin, GoalLeaseOrigin::InstalledUnresolved);
        assert_eq!(outcome.lease.as_ref(), Some(&original));
        assert_eq!(outcome.state.status, "UNCONFIRMED");
        assert_eq!(outcome.publication, GoalPublicationPolicy::NoPublish);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&foreign_lease));
        assert_eq!(state.inner.read().await.phase, NavPhase::Running);
        assert_eq!(state.cancel_generation(), cancel_before);
        assert_eq!(backend.publish_count(), 1);
        drop(handle);
        owner.finish().await.unwrap();
    }

    #[test]
    fn phase_strings_match_proto() {
        // 锁住 gRPC `status` 字符串值,避免被 Python 侧误读。
        assert_eq!(NavPhase::Idle.as_str(), "IDLE");
        assert_eq!(NavPhase::Accepted.as_str(), "ACCEPTED");
        assert_eq!(NavPhase::Planning.as_str(), "PLANNING");
        assert_eq!(NavPhase::Running.as_str(), "RUNNING");
        assert_eq!(NavPhase::Succeeded.as_str(), "SUCCEEDED");
        assert_eq!(NavPhase::Cancelled.as_str(), "CANCELLED");
        assert_eq!(NavPhase::Failed.as_str(), "FAILED");
    }

    #[test]
    fn readiness_detail_json_roundtrip() {
        let d = NavigationReadinessDetail {
            ready: true,
            reason: "ok".into(),
            reason_code: NavigationReadinessCode::Ready,
            blockers: Vec::new(),
            lcm_backend: true,
            tf: true,
            costmap: true,
            planner: true,

            profile: PROFILE_SIMPLE_2D.into(),
            module_node_count: 4,
            p1_native_profiles: P1_NATIVE_PROFILES.iter().map(|s| s.to_string()).collect(),
            backend: "RealLcm".into(),
            tf_count: 7,
            costmap_revision: 3,
            cmd_vel_sink_ready: true,
            active_profile: PROFILE_SIMPLE_2D.into(),
            stack_state_observed: true,
            stack_state_fresh: true,
            stack_last_seen_age_ms: 10,
            stack_last_status: "IDLE".into(),
            tf_generation: 2,
            planner_generation: 3,
            motion_sink_generation: 4,
            map_frame: "map".into(),
            pose_frame: "map".into(),
            tf_observed: true,
            tf_fresh: true,
            planner_input_observed: true,
            planner_input_fresh: true,
            motion_sink_observed: true,
            motion_sink_fresh: true,
        };
        let s = readiness_to_reason(&d);
        let back = readiness_from_reason(&s).expect("parse");
        assert_eq!(back.ready, d.ready);
        assert_eq!(back.module_node_count, d.module_node_count);
        assert_eq!(back.p1_native_profiles, d.p1_native_profiles);
        // Phase 7 增量字段 roundtrip
        assert_eq!(back.backend, d.backend);
        assert_eq!(back.tf_count, d.tf_count);
        assert_eq!(back.costmap_revision, d.costmap_revision);
        assert_eq!(back.cmd_vel_sink_ready, d.cmd_vel_sink_ready);
        assert_eq!(back.active_profile, d.active_profile);
        assert_eq!(back.stack_state_observed, d.stack_state_observed);
        assert_eq!(back.stack_state_fresh, d.stack_state_fresh);
        assert_eq!(back.reason_code, NavigationReadinessCode::Ready);
        assert_eq!(back.planner_generation, 3);
        assert_eq!(back.motion_sink_generation, 4);
    }

    #[tokio::test]
    async fn legacy_handle_sink_bool_cannot_make_execution_chain_ready() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        handle.set_cmd_vel_sink_ready(true);
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.cmd_vel_sink_ready);
        assert_eq!(d.reason_code, NavigationReadinessCode::StackStateUnobserved);
    }

    #[tokio::test]
    async fn readiness_unready_lists_new_fields() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.ready);
        assert!(!d.tf);
        assert!(!d.costmap);
        assert_eq!(d.profile, PROFILE_MLS3D);
        // Phase 7 字段:初始 backend=RealLcm(test handle),counters 全 0
        assert_eq!(d.backend, "RealLcm");
        assert_eq!(d.tf_count, 0);
        assert_eq!(d.costmap_revision, 0);
        assert!(!d.cmd_vel_sink_ready);
        assert!(!d.motion_sink_observed);
        assert_eq!(d.active_profile, PROFILE_MLS3D);
    }

    #[test]
    fn ensure_goal_id_empty_generates_uuid() {
        let id = ensure_goal_id("");
        assert!(!id.is_empty());
        // uuid v4 长度 36
        assert_eq!(id.len(), 36);
    }

    #[test]
    fn ensure_goal_id_trims_and_preserves() {
        assert_eq!(ensure_goal_id("  abc  "), "abc");
        assert_eq!(ensure_goal_id("xyz-1"), "xyz-1");
    }

    #[test]
    fn motion_admission_wire_decodes_nonce_without_revalidating_guard_policy() {
        assert_eq!(MotionAdmissionProof::try_from_wire(&[7; 16], 1).unwrap(), test_motion_admission());
        assert!(MotionAdmissionProof::try_from_wire(&[7; 15], 1).unwrap_err().contains("16"));
        let proof = MotionAdmissionProof::try_from_wire(&[0; 16], u64::MAX).unwrap();
        assert_eq!(proof.guard_boot_nonce, [0; 16]);
        assert_eq!(proof.motion_stop_generation, u64::MAX);
    }

    #[test]
    fn production_goal_and_path_bindings_reject_legacy_raw_types() {
        assert!(strict_typed_binding(
            "/nav/goal#geometry_msgs.PoseStamped",
            "nuwax_msgs.GoalEnvelope"
        )
        .unwrap_err()
        .contains("legacy/raw"));
        assert!(
            strict_typed_binding("/nav/path#nav_msgs.Path", "nuwax_msgs.PathEnvelope")
                .unwrap_err()
                .contains("legacy/raw")
        );
        assert!(strict_typed_binding(GOAL_CHANNEL_DEFAULT, "nuwax_msgs.GoalEnvelope").is_ok());
        assert!(strict_typed_binding(PATH_CHANNEL_DEFAULT, "nuwax_msgs.PathEnvelope").is_ok());
    }

    #[tokio::test]
    async fn set_goal_failed_when_no_pose() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        install_committed_map_fixture(&handle, "map", false).await;
        // 保留 committed map，只不注入 pose，以精确验证 readiness.tf=false。
        let state = NavRuntimeState::from_env();
        let mut input = test_goal_input("g2");
        input.target.x = 1.0;
        input.target.timestamp_ns = 0;
        let outcome = set_goal(&handle, &state, input).await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("SET_GOAL_REJECTED"));
        assert!(matches!(
            outcome.retry_reason,
            Some(SetGoalRetryReason::NavigationNotReady(_))
        ));
    }

    #[tokio::test]
    async fn cancel_clears_active_goal() {
        use futures::StreamExt;

        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let cancel_channel = channel_registry::resolve_channel(
                "NUWAX_LCM_CHANNEL_NAV_CANCEL",
            "/nav/cancel#nuwax_msgs.CancelNavigation",
        );
        let (cancel_topic, _) = parse_channel_with_suffix(&cancel_channel).expect("cancel topic");
        let mut cancellations = handle
            .transport
            .subscribe(
                cancel_topic,
                Some(
                    LcmMessageType::new("nuwax_msgs/CancelNavigation")
                        .expect("CancelNavigation type"),
                ),
            )
            .await;
        sleep(Duration::from_millis(30)).await;
        observe_ready_execution_chain(&state).await;
        let mut input = test_goal_input("g3");
        input.target.x = 1.0;
        let accepted = set_goal(&handle, &state, input).await;
        let lease = accepted.lease.expect("accepted lease");
        let outcome = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(outcome.state.status, "CANCELLED");
        assert_eq!(outcome.state.goal_id, "g3");
        let cancellation = tokio::time::timeout(Duration::from_secs(1), cancellations.next())
            .await
            .expect("cancellation was not published")
            .expect("cancellation stream closed")
            .expect("cancellation transport error");
        let cancellation =
            nuwax_codec::lcm::schema::decode_cancel_navigation_body(&cancellation.payload)
                .expect("decode cancellation");
        let pending = state.cancel_generation();
        assert_eq!(cancellation.session_id, "g3");
        assert_eq!(cancellation.owner_kind, lease.owner_kind);
        assert_eq!(cancellation.parent_request_id, lease.parent_request_id);
        assert_eq!(cancellation.goal_generation, lease.goal_generation as i64);
        assert_eq!(
            cancellation.producer_runtime_generation,
            lease.producer_runtime_generation as i64
        );
        assert_eq!(cancellation.cancel_generation, pending as i64);
        assert!(cancellation.cancel_generation > 0);
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::Cancelled);
        assert!(slot.goal.is_none());
    }

    #[tokio::test]
    async fn older_motion_generation_is_rejected_and_equal_or_newer_replaces_current_goal() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let mut first = test_goal_input("latest-10"); first.motion_admission.motion_stop_generation = 10;
        let admitted = set_goal(&handle, &state, first).await;
        assert!(admitted.accepted(), "{}", admitted.state.message);
        let original = state.active_goal_lease().await;
        let mut stale = test_goal_input("stale-9"); stale.motion_admission.motion_stop_generation = 9;
        let rejected = set_goal(&handle, &state, stale).await;
        assert_eq!(rejected.reject_reason, Some(SetGoalRejectReason::GoalGenerationStale));
        assert_eq!(state.active_goal_lease().await, original);
        assert_eq!(state.inner.read().await.goal.as_ref().unwrap().motion_admission.motion_stop_generation, 10);
        for generation in [10, 11] {
            let mut next = test_goal_input(&format!("replacement-{generation}"));
            next.motion_admission.motion_stop_generation = generation;
            let result = set_goal(&handle, &state, next).await;
            assert!(result.accepted(), "{}", result.state.message);
            assert_eq!(state.active_goal_lease().await.unwrap().session_id, format!("replacement-{generation}"));
        }
    }

    #[tokio::test]
    async fn cancellation_is_terminal_after_one_publish_without_retry_or_zero_receipt() {
        let (handle, backend) = make_blocking_handle([]).await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("one-cancel", 1);
        state.replace_active_goal_for_test(test_active_goal_with_lease(lease.clone())).await;
        backend.release_one();
        let result = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(result.state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
        assert_eq!(backend.publish_count(), 1);
        let repeated = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(repeated.state.status, "FAILED");
        assert_eq!(backend.publish_count(), 1);
        assert_eq!(state.cancel_generation(), 1);
    }

    #[tokio::test]
    async fn cancel_with_no_active_goal_is_safe() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let outcome = cancel(
            &handle,
            &state,
            CancelGoalInput {
                goal_id: String::new(),
                owner_kind: 0,
                parent_request_id: String::new(),
                goal_generation: 0,
                producer_runtime_generation: 0,
            },
        )
        .await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("GOAL_LEASE_MISSING"));
    }

    #[tokio::test]
    async fn readiness_unready_lists_all_false() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.ready);
        assert!(!d.tf);
        assert!(!d.costmap);
        assert_eq!(d.profile, PROFILE_MLS3D);
        // 仅检查列表保留尚未接通运动守卫的原生配置。
        assert!(d
            .p1_native_profiles
            .contains(&PROFILE_CMU_NATIVE.to_string()));
        assert!(d
            .p1_native_profiles
            .contains(&PROFILE_EXPLORATION_NATIVE.to_string()));
        assert!(!d.p1_native_profiles.contains(&PROFILE_MLS3D.to_string()));
    }

    #[test]
    fn build_nav_stack_handle_renders_for_all_profiles() {
        for p in [
            PROFILE_SIMPLE_2D,
            PROFILE_REPLANNING_ASTAR,
            PROFILE_CMU_NATIVE,
            PROFILE_EXPLORATION_NATIVE,
            PROFILE_MLS3D,
        ] {
            let h = build_nav_stack_handle(p);
            if let Some(h) = h {
                let text = render_dry_run(&h);
                assert!(text.contains(p), "dry-run 文本应包含 profile={}", p);
            }
        }
    }

    #[test]
    fn shared_state_default_serializes() {
        // 保证 SharedState 字段都派生 Default,给未来 nav_runtime 直接持
        // Arc<SharedState> 时用(目前 NavRuntimeSlot 内部不持 SharedState,
        // 但留 regression test)。
        let s = SharedState::default();
        assert_eq!(s.nav_state.status, "IDLE");
    }

    async fn observe_healthy_stack(state: &NavRuntimeState) {
        state
            .observe_stack_state(NavState {
                status: "IDLE".into(),
                message: "nav-stack healthy".into(),
                goal_id: String::new(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            })
            .await;
    }

    fn observe_ready_inputs(state: &NavRuntimeState) {
        state
            .observe_tf_chain(TfChainObservation {
                generation: 1,
                map_frame: "map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("ready TF observation");
        state
            .observe_planner_input(PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".into(),
                pose_frame: "map".into(),
                ready: true,
            })
            .expect("ready planner observation");
        state
            .observe_motion_sink(MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("ready motion sink observation");
    }

    async fn observe_ready_execution_chain(state: &NavRuntimeState) {
        observe_healthy_stack(state).await;
        observe_ready_inputs(state);
    }

    #[tokio::test]
    async fn readiness_requires_fresh_stack_state_observation() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();

        let missing = compute_readiness(&handle, &state).await;
        assert!(!missing.ready);
        assert!(!missing.planner);
        assert!(!missing.stack_state_observed);

        observe_ready_execution_chain(&state).await;
        let live = compute_readiness(&handle, &state).await;
        assert!(live.ready, "{}", live.reason);
        assert!(live.planner);
        assert!(live.stack_state_observed);
        assert!(live.stack_state_fresh);

        state.expire_stack_observation_for_test();
        let stale = compute_readiness(&handle, &state).await;
        assert!(!stale.ready);
        assert!(!stale.planner);
        assert!(stale.stack_state_observed);
        assert!(!stale.stack_state_fresh);
    }

    #[tokio::test]
    async fn missing_committed_map_authority_rejects_goal_without_side_effects() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("build test handle");
        let backend = BlockingPublishBackend::new([None]);
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        assert!(handle
            .committed_map_authority
            .lock()
            .expect("committed map authority")
            .is_none());
        let state = NavRuntimeState::from_env();
        let cancel_before = state.cancel_generation();
        let next_cancel_before = state.next_cancel_generation.load(AtomicOrdering::Acquire);

        let outcome = set_goal(
            &handle,
            &state,
            test_goal_input("missing-committed-map-authority"),
        )
        .await;

        assert_eq!(outcome.state.status, "FAILED");
        assert!(
            outcome
                .state
                .message
                .contains("authoritative committed map is unavailable"),
            "{}",
            outcome.state.message
        );
        assert!(outcome.lease.is_none());
        let slot = state.inner.read().await;
        assert!(slot.goal.is_none());
        assert_eq!(slot.phase, NavPhase::Idle);
        drop(slot);
        assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
        assert_eq!(state.cancel_generation(), cancel_before);
        assert_eq!(
            state.next_cancel_generation.load(AtomicOrdering::Acquire),
            next_cancel_before
        );
        assert_eq!(backend.publish_count(), 0);
    }

    #[tokio::test]
    async fn changed_committed_map_provenance_rejects_intent_without_side_effects() {
        for changed_field in ["revision", "boot_nonce", "commit_sequence", "source_timestamp"] {
            let (handle, backend) = make_blocking_handle([None]).await;
            let state = NavRuntimeState::from_env();
            observe_ready_execution_chain(&state).await;
            let admission = NavAdmissionContext::new(
                handle.mapping_lifecycle_lock.clone(),
                handle.committed_map_authority.clone(),
                handle.map_revision_latest_tx.subscribe(),
            );
            let intent = goal_intent_from_set_goal(
                &admission,
                &state,
                test_goal_input(changed_field),
                None,
            )
            .await
            .expect("frozen committed-map intent");
            {
                let mut authority = handle
                    .committed_map_authority
                    .lock()
                    .expect("committed map authority");
                let current = authority.as_mut().expect("installed committed map");
                match changed_field {
                    "boot_nonce" => current.commit_boot_nonce[0] ^= 0xff,
                    "commit_sequence" => current.commit_sequence += 1,
                    "source_timestamp" => current.source_timestamp_ns += 1,
                    _ => current.revision += 1,
                }
            }

            let outcome = admit_goal_intent(
                &admission,
                &NavAdmissionIo::new(
                    handle.state.clone(),
                    handle.transport.clone(),
                    handle.transport_backend_name,
                    handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
),
                &state,
                intent,
                GoalProducerEpochAuthority::PublicNavigation,
            )
            .await;

            assert_eq!(outcome.state.status, "FAILED");
            assert!(outcome
                .state
                .message
                .contains(match changed_field {
                    "revision" => "committed map revision advanced",
                    "boot_nonce" => "map session or coordinate generation changed",
                    _ => "committed map revision or receipt conflicts",
                }));
            assert!(outcome.lease.is_none());
            assert!(state.inner.read().await.goal.is_none());
            assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
            assert_eq!(backend.publish_count(), 0);
        }
    }

    #[tokio::test]
    async fn stale_goal_producer_epoch_rejects_intent_without_side_effects() {
        let (handle, backend) = make_blocking_handle([None]).await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let admission = NavAdmissionContext::new(
            handle.mapping_lifecycle_lock.clone(),
            handle.committed_map_authority.clone(),
            handle.map_revision_latest_tx.subscribe(),
        );
        let mut intent = goal_intent_from_set_goal(
            &admission,
            &state,
            test_goal_input("stale-producer-epoch"),
            None,
        )
        .await
        .expect("frozen producer intent");
        intent.producer_epoch += 1;

        let outcome = admit_goal_intent(
            &admission,
            &NavAdmissionIo::new(
                handle.state.clone(),
                handle.transport.clone(),
                handle.transport_backend_name,
                handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
),
            &state,
            intent,
            GoalProducerEpochAuthority::PublicNavigation,
        )
        .await;

        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome
            .state
            .message
            .contains("invalid producer, owner, priority, or generation identity"));
        assert!(outcome.lease.is_none());
        assert!(state.inner.read().await.goal.is_none());
        assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
        assert_eq!(backend.publish_count(), 0);
    }

    #[tokio::test]
    async fn set_goal_only_publishes_goal_and_returns_accepted() {
        use futures::StreamExt;

        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let cmd_channel = channel_registry::resolve_channel(
            "NUWAX_LCM_CHANNEL_CMD_VEL",
            "/cmd_vel#geometry_msgs.Twist",
        );
        let (cmd_topic, _) = parse_channel_with_suffix(&cmd_channel).expect("cmd topic");
        let mut commands = handle
            .transport
            .subscribe(
                cmd_topic,
                Some(LcmMessageType::new("geometry_msgs/Twist").expect("Twist type")),
            )
            .await;
        let goal_channel = channel_registry::resolve_channel(
                GOAL_CHANNEL_PLATFORM,
            GOAL_CHANNEL_DEFAULT,
        );
        let (goal_topic, _) = parse_channel_with_suffix(&goal_channel).expect("goal topic");
        let mut goals = handle
            .transport
            .subscribe(goal_topic, Some(goal_msg_type()))
            .await;
        sleep(Duration::from_millis(30)).await;

        let outcome = set_goal(
            &handle,
            &state,
            SetGoalInput {
            options: nuwax_nav_types_messages::GoalOptions { allow_reverse: true, ..Default::default() },
                target: PoseState {
                    x: 1.0,
                    qw: 1.0,
                    timestamp_ns: now_ns(),
                    ..PoseState::default()
                },
                ..test_goal_input("stack-owned-goal")
            },
        )
        .await;

        assert_eq!(
            outcome.state.status, "ACCEPTED",
            "{}",
            outcome.state.message
        );
        assert_eq!(outcome.state.goal_id, "stack-owned-goal");
        let goal = tokio::time::timeout(Duration::from_secs(1), goals.next())
            .await
            .expect("goal was not published")
            .expect("goal stream closed")
            .expect("goal transport error");
        let goal = nuwax_codec::lcm::schema::decode_goal_envelope_body(&goal.payload)
            .expect("decode submitted GoalEnvelope");
        assert_eq!(goal.goal.header.frame_id, "map");
        assert_eq!(goal.session_id, "stack-owned-goal");
        assert_eq!(goal.owner_kind, schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV);
        assert_eq!(goal.parent_request_id, "request-stack-owned-goal");
        assert_eq!(goal.motion_guard_boot_nonce, [7; 16]);
        assert_eq!(goal.motion_stop_generation, 1);
        assert!(goal.goal_generation > 0);
        assert_ne!(goal.producer_runtime_generation, 0);
        assert!(
            goal.goal.header.stamp_sec > 0 || goal.goal.header.stamp_nsec > 0,
            "published goal must carry a real timestamp"
        );
        assert_eq!(goal.goal.pose.position[0], 1.0);
        assert!(
            tokio::time::timeout(Duration::from_millis(150), commands.next())
                .await
                .is_err(),
            "geometry facade must not publish an immediate /cmd_vel"
        );
    }

    #[tokio::test]
    async fn exploration_goal_envelope_carries_complete_map_boundary() {
        use futures::StreamExt;

        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let (goal_topic, _) = parse_channel_with_suffix(&channel_registry::resolve_channel(
                GOAL_CHANNEL_PLATFORM,
            GOAL_CHANNEL_DEFAULT,
        ))
        .expect("goal topic");
        let mut goals = handle
            .transport
            .subscribe(goal_topic, Some(goal_msg_type()))
            .await;
        sleep(Duration::from_millis(30)).await;
        let mut input = test_goal_input("exploration-boundary-goal");
        input.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        input.parent_request_id = "explore-parent".to_string();
        let points = vec![
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [4.0, 3.0, 0.0],
            [0.0, 3.0, 0.0],
        ];

        let slash_frame = set_exploration_goal_for_test(
            &handle,
            &state,
            input.clone(),
            Some(GoalBoundaryConstraint {
                frame_id: "/map".to_string(),
                points: points.clone(),
            }),
        )
        .await;
        assert_eq!(slash_frame.state.status, "FAILED");
        assert!(slash_frame.state.message.contains("GOAL_BOUNDARY_INVALID"));
        assert!(state.active_goal_lease().await.is_none());

        let outcome = set_exploration_goal_for_test(
            &handle,
            &state,
            input,
            Some(GoalBoundaryConstraint {
                frame_id: "map".to_string(),
                points: points.clone(),
            }),
        )
        .await;

        assert_eq!(
            outcome.state.status, "ACCEPTED",
            "{}",
            outcome.state.message
        );
        let frame = tokio::time::timeout(Duration::from_secs(1), goals.next())
            .await
            .expect("goal was not published")
            .expect("goal stream closed")
            .expect("goal transport error");
        let envelope = schema::decode_goal_envelope_body(&frame.payload)
            .expect("decode exploration GoalEnvelope");
        assert_eq!(
            envelope.owner_kind,
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        );
        assert_eq!(envelope.parent_request_id, "explore-parent");
        assert_eq!(envelope.boundary_frame_id, "map");
        assert_eq!(envelope.boundary_points, points);
    }

    #[tokio::test]
    async fn exploration_goal_envelope_preserves_configured_map_boundary() {
        use futures::StreamExt;

        let handle = make_handle().await;
        install_committed_map_fixture(&handle, "robot/map", true).await;
        let state = NavRuntimeState::from_env();
        observe_healthy_stack(&state).await;
        state
            .observe_tf_chain(TfChainObservation {
                generation: 1,
                map_frame: "robot/map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("configured-frame TF observation");
        state
            .observe_planner_input(PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "robot/map".into(),
                pose_frame: "robot/map".into(),
                ready: true,
            })
            .expect("configured-frame planner observation");
        state
            .observe_motion_sink(MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("ready motion sink observation");
        let (goal_topic, _) = parse_channel_with_suffix(&channel_registry::resolve_channel(
                GOAL_CHANNEL_PLATFORM,
            GOAL_CHANNEL_DEFAULT,
        ))
        .expect("goal topic");
        let mut goals = handle
            .transport
            .subscribe(goal_topic, Some(goal_msg_type()))
            .await;
        sleep(Duration::from_millis(30)).await;
        let mut input = test_goal_input("configured-exploration-boundary");
        input.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        input.parent_request_id = "configured-explore-parent".to_string();
        input.frame_id = "robot/map".to_string();
        input.target.frame_id = "robot/map".to_string();
        let points = vec![[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 3.0, 0.0]];

        let slash_frame = set_exploration_goal_for_test(
            &handle,
            &state,
            input.clone(),
            Some(GoalBoundaryConstraint {
                frame_id: "/robot/map".to_string(),
                points: points.clone(),
            }),
        )
        .await;
        assert_eq!(slash_frame.state.status, "FAILED");
        assert!(slash_frame.state.message.contains("GOAL_BOUNDARY_INVALID"));
        assert!(state.active_goal_lease().await.is_none());

        let outcome = set_exploration_goal_for_test(
            &handle,
            &state,
            input,
            Some(GoalBoundaryConstraint {
                frame_id: "robot/map".to_string(),
                points: points.clone(),
            }),
        )
        .await;

        assert_eq!(
            outcome.state.status, "ACCEPTED",
            "{}",
            outcome.state.message
        );
        let frame = tokio::time::timeout(Duration::from_secs(1), goals.next())
            .await
            .expect("configured-frame goal was not published")
            .expect("goal stream closed")
            .expect("goal transport error");
        let envelope = schema::decode_goal_envelope_body(&frame.payload)
            .expect("decode configured-frame exploration GoalEnvelope");
        assert_eq!(envelope.goal.header.frame_id, "robot/map");
        assert_eq!(envelope.boundary_frame_id, "robot/map");
        assert_eq!(envelope.boundary_points, points);
    }

    #[tokio::test]
    async fn configured_exploration_boundary_rejects_literal_map_without_publish() {
        let (handle, backend) = make_blocking_handle([None]).await;
        install_committed_map_fixture(&handle, "robot/map", true).await;
        let state = NavRuntimeState::from_env();
        let mut input = test_goal_input("configured-boundary-mismatch");
        input.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        input.parent_request_id = "configured-explore-parent".to_string();
        input.frame_id = "robot/map".to_string();
        input.target.frame_id = "robot/map".to_string();

        let outcome = set_exploration_goal_for_test(
            &handle,
            &state,
            input,
            Some(GoalBoundaryConstraint {
                frame_id: "map".to_string(),
                points: vec![[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 3.0, 0.0]],
            }),
        )
        .await;

        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("GOAL_BOUNDARY_INVALID"));
        assert!(outcome.lease.is_none());
        assert!(state.inner.read().await.goal.is_none());
        assert_eq!(backend.publish_count(), 0);
        assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
    }

    #[tokio::test]
    async fn same_map_name_allows_revision_and_receipt_change_at_admission() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let admission = NavAdmissionContext::new(
            handle.mapping_lifecycle_lock.clone(),
            handle.committed_map_authority.clone(),
            handle.map_revision_latest_tx.subscribe(),
        );
        let intent = goal_intent_from_set_goal(
            &admission,
            &state,
            test_goal_input("same-name-increment"),
            None,
        )
        .await
        .expect("same-name intent");
        let mut increment = handle
            .map_revision_latest_tx
            .borrow()
            .clone()
            .expect("initial map revision");
        increment.revision += 1;
        increment.receipt.occupancy_revision = increment.revision;
        increment.receipt.commit_id.sequence += 1;
        handle.map_revision_latest_tx.send_replace(Some(increment));

        let outcome = admit_goal_intent(
            &admission,
            &NavAdmissionIo::new(
                handle.state.clone(),
                handle.transport.clone(),
                handle.transport_backend_name,
                handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
),
            &state,
            intent,
            GoalProducerEpochAuthority::PublicNavigation,
        )
        .await;

        assert_eq!(
            outcome.state.status, "ACCEPTED",
            "{}",
            outcome.state.message
        );
        assert!(outcome.lease.is_some());
        assert!(state.inner.read().await.goal.is_some());
    }

    #[tokio::test]
    async fn changed_map_name_rejects_before_goal_lease_and_slot_cas() {
        let (handle, backend) = make_blocking_handle([None]).await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let admission = NavAdmissionContext::new(
            handle.mapping_lifecycle_lock.clone(),
            handle.committed_map_authority.clone(),
            handle.map_revision_latest_tx.subscribe(),
        );
        let intent =
            goal_intent_from_set_goal(&admission, &state, test_goal_input("changed-name"), None)
                .await
                .expect("changed-name intent");
        let mut replacement = handle
            .map_revision_latest_tx
            .borrow()
            .clone()
            .expect("initial map revision");
        replacement.map_name = "hallway".to_string();
        handle
            .map_revision_latest_tx
            .send_replace(Some(replacement));

        let cancel_before = state.cancel_generation();
        let next_cancel_before = state.next_cancel_generation.load(AtomicOrdering::Acquire);
        let outcome = admit_goal_intent(
            &admission,
            &NavAdmissionIo::new(
                handle.state.clone(),
                handle.transport.clone(),
                handle.transport_backend_name,
                handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
),
            &state,
            intent,
            GoalProducerEpochAuthority::PublicNavigation,
        )
        .await;

        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("active map name changed"));
        assert_eq!(
            outcome.reject_reason,
            Some(SetGoalRejectReason::ActiveMapNameChanged)
        );
        assert!(outcome.lease.is_none());
        assert!(state.inner.read().await.goal.is_none());
        assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
        assert_eq!(state.cancel_generation(), cancel_before);
        assert_eq!(
            state.next_cancel_generation.load(AtomicOrdering::Acquire),
            next_cancel_before
        );
        assert_eq!(backend.publish_count(), 0);
    }

    #[test]
    fn mapping_cloud_frozen_name_survives_provider_wait_map_switch() {
        let (map_tx, map_rx) =
            tokio::sync::watch::channel(Some(map_revision_fixture(
                &MapState {
                    width: 1,
                    height: 1,
                    resolution: 0.1,
                    origin_x: 0.0,
                    origin_y: 0.0,
                    frame_id: "map".to_string(),
                    revision: 1,
                    timestamp_ns: 1,
                    data: vec![0],
                },
                "mapping-cloud-session",
                "office",
            )));
        let admission = NavAdmissionContext::with_expected_map_name(
            Arc::new(tokio::sync::Mutex::new(())),
            Arc::new(StdMutex::new(None)),
            map_rx,
            Some("office".to_string()),
        );
        let mut switched = map_tx.borrow().clone().expect("initial map");
        switched.map_name = "hallway".to_string();
        map_tx.send_replace(Some(switched));

        assert!(!admission.map_name_unchanged());
    }

    #[tokio::test]
    async fn goal_lease_is_idempotent_and_new_owners_replace_directly() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let mut first = test_goal_input("public-1"); first.goal_generation = 10;
        let initial = set_goal(&handle, &state, first.clone()).await;
        assert!(initial.accepted());
        let replay = set_goal(&handle, &state, first).await;
        assert!(replay.accepted()); assert_eq!(replay.lease, initial.lease);
        let mut child = test_goal_input("child-2"); child.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        child.parent_request_id = "exploration-parent".into(); child.goal_generation = 11;
        let replacement = set_exploration_goal_for_test(&handle, &state, child, None).await;
        assert!(replacement.accepted(), "{}", replacement.state.message);
        assert_eq!(state.active_goal_lease().await, replacement.lease);
        let old_cancel = cancel(&handle, &state, cancel_input(initial.lease.as_ref().unwrap())).await;
        assert_eq!(old_cancel.state.status, "FAILED");
        assert_eq!(state.active_goal_lease().await, replacement.lease);
        let mut next = test_goal_input("public-3"); next.goal_generation = 12;
        let result = set_goal(&handle, &state, next).await;
        assert!(result.accepted()); assert_eq!(state.active_goal_lease().await, result.lease);
    }

    #[tokio::test]
    async fn cancel_during_goal_publish_does_not_wait_or_restore_cancelled_goal() {
        let (handle, backend, owner) = make_unit_blocking_handle([]).await;
        backend.outcomes.lock().unwrap().push_back(Some(TransportError::NotDispatched(
            Box::new(TransportError::LcmBackend("goal not dispatched".into())))));
        let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
        observe_ready_execution_chain(&state).await;
        let goal = tokio::spawn({ let handle=handle.clone();let state=state.clone();
            async move { set_goal(&handle, &state, test_goal_input("cancel-during-publish")).await } });
        backend.wait_for_publish_count(1).await;
        let lease = state.active_goal_lease().await.unwrap();
        let cancellation = tokio::spawn({ let handle=handle.clone();let state=state.clone();
            async move { cancel(&handle, &state, cancel_input(&lease)).await } });
        backend.wait_for_publish_count(2).await;
        assert!(state.active_goal_lease().await.is_none());
        backend.release_one();
        let _ = goal.await.unwrap();
        backend.release_one();
        assert_eq!(cancellation.await.unwrap().state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
        drop(handle); owner.finish().await.unwrap();
    }

    #[tokio::test]
    async fn path_envelope_requires_exact_lease_hash_and_monotonic_path_revision() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("path-goal", 5);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Planning;
            slot.goal = Some(test_active_goal("path-goal", 5));
        }
        let first = sealed_path(&lease, 1, 1.0);
        let accepted = state
            .observe_path_envelope(&first)
            .await
            .expect("first path");
        assert_eq!(state.display_path_for(&lease,true).await,first.path.poses.iter().map(|p|p.pose.position).collect::<Vec<_>>());
        assert!(state.display_path_for(&test_lease("other-goal",6),true).await.is_empty());
        assert!(state.display_path_for(&accepted,false).await.is_empty(),"等待重规划时不能继续显示已失效的旧路径");
        assert_eq!(accepted.path_revision, 1);
        assert_eq!(accepted.planner_runtime_generation, first.planner_runtime_generation as u64);
        assert_eq!(
            state.observe_path_envelope(&first).await.unwrap(),
            accepted,
            "same assigned PathKey is idempotent"
        );

        let conflicting = sealed_path(&lease, 1, 2.0);
        assert_eq!(state.observe_path_envelope(&conflicting).await.unwrap(), accepted);
        assert_eq!(state.display_path_for(&lease, true).await, first.path.poses.iter()
            .map(|pose| pose.pose.position).collect::<Vec<_>>(), "display retains first same-key candidate; scheduler detects real mutation");
        let advanced = sealed_path(&lease, 2, 3.0);
        assert_eq!(
            state
                .observe_path_envelope(&advanced)
                .await
                .expect("advanced path")
                .path_revision,
            2
        );
        assert!(state
            .observe_path_envelope(&first)
            .await
            .unwrap_err()
            .contains("stale path revision"));
        let mut wrong_goal = sealed_path(&lease, 3, 4.0);
        wrong_goal.producer_runtime_generation = 9;
        schema::validate_path_envelope(&wrong_goal).expect("reseal wrong goal identity");
        assert!(state
            .observe_path_envelope(&wrong_goal)
            .await
            .unwrap_err()
            .contains("assigned goal"));
    }

    #[tokio::test]
    async fn initial_path_remains_candidate_until_exact_navigation_state_promotes_it() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("confirmed-path", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        let first_path = sealed_path(&lease, 1, 1.0);
        let candidate = state
            .observe_path_envelope(&first_path)
            .await
            .expect("first path is retained as a candidate");
        assert_eq!(
            state.active_goal_lease().await.as_ref(),
            Some(&lease),
            "revision 1 publication alone must not replace the pre-path confirmed lease"
        );

        let pre_path = state
            .correlate_navigation_state(
                &lease.session_id,
                lease.owner_kind,
                &lease.parent_request_id,
                lease.goal_generation,
                lease.path_revision,
                lease.producer_runtime_generation,
                lease.planner_runtime_generation,
            )
            .await
            .expect("path-first publication must preserve zero/zero state correlation")
            .expect("active lease");
        assert_eq!(pre_path, lease);

        let promoted = state
            .correlate_navigation_state(
                &candidate.session_id,
                candidate.owner_kind,
                &candidate.parent_request_id,
                candidate.goal_generation,
                candidate.path_revision,
                candidate.producer_runtime_generation,
                candidate.planner_runtime_generation,
            )
            .await
            .expect("exact scheduler state promotes revision 1")
            .expect("active lease");
        assert_eq!(promoted, candidate);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&candidate));
    }

    #[tokio::test]
    async fn pre_path_state_then_initial_path_preserves_confirmed_lease() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("state-first", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Planning;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        assert_eq!(
            state
                .correlate_navigation_state(
                    &lease.session_id,
                    lease.owner_kind,
                    &lease.parent_request_id,
                    lease.goal_generation,
                    0,
                    lease.producer_runtime_generation,
                    0,
                )
                .await
                .expect("state-first zero/zero correlation")
                .as_ref(),
            Some(&lease)
        );
        let candidate = state
            .observe_path_envelope(&sealed_path(&lease, 1, 1.0))
            .await
            .expect("initial path candidate");
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));
        assert_eq!(candidate.path_revision, 1);
    }

    #[tokio::test]
    async fn navigation_state_promotes_only_an_observed_path_candidate() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("candidate-path", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        let first_path = sealed_path(&lease, 1, 1.0);
        let first_candidate = state.observe_path_envelope(&first_path).await.unwrap();
        let second_path = sealed_path(&lease, 2, 2.0);
        assert!(state
            .correlate_navigation_state(
                &lease.session_id,
                lease.owner_kind,
                &lease.parent_request_id,
                lease.goal_generation,
                2,
                lease.producer_runtime_generation,
                second_path.planner_runtime_generation as u64,
            )
            .await
            .expect_err("NavState cannot authorize a path Geometry has not observed yet")
            .contains("path_revision"));

        let candidate = state.observe_path_envelope(&second_path).await.unwrap();
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));
        let first_promoted = state
            .correlate_navigation_state(
                &first_candidate.session_id,
                first_candidate.owner_kind,
                &first_candidate.parent_request_id,
                first_candidate.goal_generation,
                first_candidate.path_revision,
                first_candidate.producer_runtime_generation,
                first_candidate.planner_runtime_generation,
            )
            .await
            .expect("revision 1 remains independently promotable")
            .expect("active lease");
        assert_eq!(first_promoted, first_candidate);
        let promoted = state
            .correlate_navigation_state(
                &candidate.session_id,
                candidate.owner_kind,
                &candidate.parent_request_id,
                candidate.goal_generation,
                candidate.path_revision,
                candidate.producer_runtime_generation,
                candidate.planner_runtime_generation,
            )
            .await
            .expect("scheduler acknowledgement promotes an observed candidate")
            .expect("active lease");
        assert_eq!(promoted, candidate);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&candidate));
    }

    #[tokio::test]
    async fn cancel_requires_complete_matching_goal_lease() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("owned-goal", 7);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("owned-goal", 7));
        }
        let mut wrong = cancel_input(&lease);
        wrong.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        let rejected = cancel(&handle, &state, wrong).await;
        assert_eq!(rejected.state.status, "FAILED");
        assert!(rejected.state.message.contains("GOAL_LEASE_MISMATCH"));
        assert_eq!(state.cancel_generation(), 0);
        assert!(state.active_goal_lease().await.is_some());
    }

    #[tokio::test]
    async fn cancel_rejects_a_stale_producer_epoch_before_publish() {
        let (handle, backend) = make_blocking_handle([None]).await;
        let state = NavRuntimeState::from_env();
        let mut lease_a = test_lease("epoch-goal", 1);
        lease_a.producer_runtime_generation = 41;
        let mut lease_b = lease_a.clone();
        lease_b.producer_runtime_generation = 42;
        lease_b.producer_runtime_generation = 42;
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease_b.clone()));
        }

        let rejected = tokio::time::timeout(
            Duration::from_millis(100),
            cancel(&handle, &state, cancel_input(&lease_a)),
        )
        .await
        .expect("a stale producer epoch must be rejected before publish");
        assert_eq!(rejected.state.status, "FAILED");
        assert!(rejected.state.message.contains("GOAL_LEASE_MISMATCH"));
        assert_eq!(backend.publish_count(), 0);
        assert_eq!(state.cancel_generation(), 0);

        let exact_handle = handle.clone();
        let exact_state = state.clone();
        let exact_lease = lease_b.clone();
        let exact = tokio::spawn(async move {
            cancel(&exact_handle, &exact_state, cancel_input(&exact_lease)).await
        });
        backend.wait_for_publish_count(1).await;
        assert!(state.cancel_generation() > 0);
        assert!(state.active_goal_lease().await.is_none());
        backend.release_one();
        assert_eq!(
            exact.await.expect("exact epoch cancel").state.status,
            "CANCELLED"
        );
        assert_eq!(backend.publish_count(), 1);
    }

    #[tokio::test]
    async fn navigation_state_names_real_lease_mismatch() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("diagnostic-goal", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        let correlated = state
            .correlate_navigation_state(
                &lease.session_id,
                lease.owner_kind,
                &lease.parent_request_id,
                lease.goal_generation,
                lease.path_revision,
                lease.producer_runtime_generation,
                lease.planner_runtime_generation,
            )
            .await
            .expect("matching lease identity must correlate")
            .expect("active lease");
        assert_eq!(correlated, lease);

        let error = state
            .correlate_navigation_state(
                &lease.session_id,
                schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                &lease.parent_request_id,
                lease.goal_generation,
                lease.path_revision,
                lease.producer_runtime_generation,
                lease.planner_runtime_generation,
            )
            .await
            .expect_err("wrong owner must not match an owned navigation state");
        assert_eq!(
            error,
            "NAV_STATE_LEASE_REJECTED: mismatched fields: owner_kind"
        );
    }

    #[tokio::test]
    async fn candidate_from_replaced_goal_cannot_promote_replacement_lease() {
        let state = NavRuntimeState::from_env();
        let lease_a = test_lease("candidate-a", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Planning;
            slot.goal = Some(test_active_goal_with_lease(lease_a.clone()));
        }
        let candidate_a = state
            .observe_path_envelope(&sealed_path(&lease_a, 1, 1.0))
            .await
            .expect("candidate A");

        let lease_b = test_lease("replacement-b", 2);
        {
            let mut slot = state.inner.write().await;
            slot.goal = Some(test_active_goal_with_lease(lease_b.clone()));
        }
        let error = state
            .correlate_navigation_state(
                &candidate_a.session_id,
                candidate_a.owner_kind,
                &candidate_a.parent_request_id,
                candidate_a.goal_generation,
                candidate_a.path_revision,
                candidate_a.producer_runtime_generation,
                candidate_a.planner_runtime_generation,
            )
            .await
            .expect_err("stale candidate A must not promote replacement B");
        assert!(error.contains("session_id"));
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease_b));
    }

    #[tokio::test]
    async fn empty_cancel_without_active_goal_and_partial_identity_remain_rejected() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("exact-cancel", 1);
        let empty = CancelGoalInput { goal_id: String::new(), owner_kind: 0, parent_request_id: String::new(), goal_generation: 0, producer_runtime_generation: 0 };
        assert_eq!(cancel(&handle, &state, empty.clone()).await.state.status, "FAILED");
        state.replace_active_goal_for_test(test_active_goal_with_lease(lease.clone())).await;
        let mut partial = empty.clone(); partial.goal_id = lease.session_id.clone();
        assert_eq!(cancel(&handle, &state, partial).await.state.status, "FAILED");
        assert_eq!(state.active_goal_lease().await, Some(lease));
        assert_eq!(cancel(&handle, &state, empty).await.state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
    }

    #[tokio::test]
    async fn cancel_generation_rejects_int64_exhaustion_without_reuse() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("exhausted-cancel", 1);
        state.replace_active_goal_for_test(test_active_goal_with_lease(lease.clone())).await;
        state.next_cancel_generation.store(i64::MAX as u64, AtomicOrdering::Release);
        let outcome = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("CANCEL_GENERATION_EXHAUSTED"));
        assert_eq!(state.cancel_generation(), i64::MAX as u64);
        assert_eq!(state.active_goal_lease().await, Some(lease));
    }

    #[tokio::test]
    async fn cancel_generation_rejects_u64_max_without_wrapping_to_zero() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("exhausted-cancel", 1);
        state.replace_active_goal_for_test(test_active_goal_with_lease(lease.clone())).await;
        state.next_cancel_generation.store(u64::MAX, AtomicOrdering::Release);
        let outcome = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("CANCEL_GENERATION_EXHAUSTED"));
        assert_eq!(state.cancel_generation(), u64::MAX);
        assert_eq!(state.active_goal_lease().await, Some(lease));
    }

    #[tokio::test]
    async fn late_cancel_success_or_error_cannot_mutate_a_replacement_slot() {
        for fail in [false, true] {
            let (handle, backend) = make_blocking_handle([fail.then(|| "late cancel publish failed".into())]).await;
            let state = NavRuntimeState::from_env();
            let old = test_lease("old", 1); let new = test_lease("new", 2);
            state.replace_active_goal_for_test(test_active_goal_with_lease(old.clone())).await;
            let task = tokio::spawn({ let handle=handle.clone(); let state=state.clone();
                async move { cancel(&handle, &state, cancel_input(&old)).await } });
            backend.wait_for_publish_count(1).await;
            assert!(state.active_goal_lease().await.is_none());
            state.replace_active_goal_for_test(test_active_goal_with_lease(new.clone())).await;
            backend.release_one();
            let result = task.await.unwrap();
            assert_eq!(result.state.status, if fail { "FAILED" } else { "CANCELLED" });
            assert_eq!(state.active_goal_lease().await, Some(new));
            assert_eq!(backend.publish_count(), 1);
        }
    }

    #[tokio::test]
    async fn set_goal_rejects_frame_that_differs_from_planner_map_frame() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;

        let outcome = set_goal(
            &handle,
            &state,
            SetGoalInput {
            options: Default::default(),
                frame_id: "odom".into(),
                target: PoseState {
                    frame_id: "odom".into(),
                    timestamp_ns: now_ns(),
                    ..PoseState::default()
                },
                ..test_goal_input("wrong-frame")
            },
        )
        .await;

        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("GOAL_FRAME_MISMATCH"));
        assert_eq!(outcome.retry_reason, None);
        assert!(state.inner.read().await.goal.is_none());
    }

    #[tokio::test]
    async fn cancel_publish_failure_is_explicit_failed_state() {
        let (handle, backend) = make_blocking_handle([Some("cancel publish failed".into())]).await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("cancel-me", 1);
        state.replace_active_goal_for_test(test_active_goal_with_lease(lease.clone())).await;
        backend.release_one();
        let outcome = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("cancel publish failed"));
        assert!(state.active_goal_lease().await.is_none());
        assert_eq!(backend.publish_count(), 1);
    }

    #[tokio::test]
    async fn readiness_uses_real_consumer_observations_not_shared_state() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_healthy_stack(&state).await;

        let before = compute_readiness(&handle, &state).await;
        assert!(!before.ready);
        assert!(
            !before.tf,
            "SharedState.pose must not impersonate a TF chain"
        );
        assert!(
            !before.costmap,
            "SharedState.map must not impersonate planner-observed costmap input"
        );
        assert!(
            !before.cmd_vel_sink_ready,
            "the legacy handle bool must not impersonate a live motion sink"
        );

        state
            .observe_tf_chain(TfChainObservation {
                generation: 1,
                map_frame: "map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("valid TF observation");
        state
            .observe_planner_input(PlannerInputObservation {
                generation: 7,
                map_revision: 42,
                map_frame: "map".into(),
                pose_frame: "map".into(),
                ready: true,
            })
            .expect("valid planner observation");
        state
            .observe_motion_sink(MotionSinkObservation {
                generation: 11,
                ready: true,
            })
            .expect("valid sink observation");

        let after = compute_readiness(&handle, &state).await;
        assert!(after.ready, "{}", after.reason);
        assert_eq!(after.reason_code, NavigationReadinessCode::Ready);
        assert_eq!(after.tf_count, 1);
        assert_eq!(after.costmap_revision, 42);
        assert_eq!(after.tf_generation, 1);
        assert_eq!(after.planner_generation, 7);
        assert_eq!(after.motion_sink_generation, 11);
        {
            let mut observations = state.runtime_observations.lock().unwrap();
            let earlier = Instant::now() - Duration::from_secs(10);
            observations.tf.observed_at = Some(earlier);
            observations.planner.observed_at = Some(earlier);
        }
        assert!(compute_readiness(&handle, &state).await.ready,
            "有效输入不能因旧两秒门槛阻断导航");
        state.observe_planner_input(PlannerInputObservation {
            generation: 7, map_revision: 42, map_frame: "map".into(),
            pose_frame: "map".into(), ready: false,
        }).unwrap();
        assert!(!compute_readiness(&handle, &state).await.ready,
            "明确失效的规划输入仍须撤销准入");
    }

    #[tokio::test]
    async fn absent_planner_frames_report_unready_instead_of_frame_mismatch() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_healthy_stack(&state).await;
        let before = compute_readiness(&handle, &state).await;
        assert!(before.blockers.contains(&NavigationReadinessCode::PlannerInputUnobserved));
        for (map, pose) in [("", ""), ("map", ""), ("", "map")] {
            state.observe_planner_input(PlannerInputObservation {
                generation: 1, map_revision: 1, map_frame: map.into(), pose_frame: pose.into(), ready: false,
            }).unwrap();
            let detail = compute_readiness(&handle, &state).await;
            assert!(!detail.ready);
            assert!(detail.blockers.contains(&NavigationReadinessCode::PlannerNotReady));
            assert!(!detail.blockers.contains(&NavigationReadinessCode::PlannerFrameMismatch));
            let io = NavAdmissionIo::new(handle.state.clone(), handle.transport.clone(), handle.transport_backend_name, handle.lcm_backend_available, handle.body_reference_frame_id.clone(),
);
            let admission = compute_admission_readiness(&io, &state).await;
            assert!(admission.blockers.contains(&NavigationReadinessCode::PlannerNotReady));
            assert!(!admission.blockers.contains(&NavigationReadinessCode::PlannerFrameMismatch));
        }
    }

    #[tokio::test]
    async fn frame_mismatch_and_stale_generation_fail_closed() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_healthy_stack(&state).await;
        state
            .observe_tf_chain(TfChainObservation {
                generation: 4,
                map_frame: "map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("valid TF observation");
        state
            .observe_planner_input(PlannerInputObservation {
                generation: 9,
                map_revision: 3,
                map_frame: "map".into(),
                pose_frame: "odom".into(),
                ready: true,
            })
            .expect("observation is accepted but readiness rejects the frame contract");
        state
            .observe_motion_sink(MotionSinkObservation {
                generation: 5,
                ready: true,
            })
            .expect("valid sink observation");

        let detail = compute_readiness(&handle, &state).await;
        assert!(!detail.ready);
        assert_eq!(
            detail.reason_code,
            NavigationReadinessCode::PlannerFrameMismatch
        );
        assert_eq!(detail.map_frame, "map");
        assert_eq!(detail.pose_frame, "odom");

        let stale = state.observe_planner_input(PlannerInputObservation {
            generation: 8,
            map_revision: 4,
            map_frame: "map".into(),
            pose_frame: "map".into(),
            ready: true,
        });
        assert_eq!(
            stale,
            Err(ObservationError::StaleGeneration {
                current: 9,
                received: 8,
            })
        );
    }

    #[tokio::test]
    async fn motion_sink_heartbeat_refreshes_freshness_with_same_generation() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        assert!(compute_readiness(&handle, &state).await.ready);

        state.expire_motion_sink_for_test();
        let stale = compute_readiness(&handle, &state).await;
        assert!(stale.ready);
        assert!(!stale.cmd_vel_sink_ready);
        assert!(!stale.motion_sink_fresh);
        assert_eq!(stale.reason_code, NavigationReadinessCode::Ready);

        state
            .observe_motion_sink(MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("same-generation heartbeat refreshes freshness");
        assert!(compute_readiness(&handle, &state).await.ready);
    }

    #[tokio::test]
    async fn cancel_never_publishes_actuator_cmd_vel() {
        use futures::StreamExt;

        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("single-owner", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("single-owner", 1));
        }
        let cmd_channel = channel_registry::resolve_channel(
            "NUWAX_LCM_CHANNEL_CMD_VEL",
            "/cmd_vel#geometry_msgs.Twist",
        );
        let (cmd_topic, _) = parse_channel_with_suffix(&cmd_channel).expect("cmd topic");
        let mut commands = handle
            .transport
            .subscribe(
                cmd_topic,
                Some(LcmMessageType::new("geometry_msgs/Twist").expect("Twist type")),
            )
            .await;
        sleep(Duration::from_millis(30)).await;

        let outcome = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(outcome.state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
        assert!(
            tokio::time::timeout(Duration::from_millis(150), commands.next())
                .await
                .is_err(),
            "facade cancellation must not publish /cmd_vel; the guard UDS is the sole actuator ingress"
        );
    }

    // These tests exercise the actual internal producer/admission functions only.
    // They stop at existing rejection fences before configuration/wire SHA paths.
    fn self_hash_stage_context() -> (NavAdmissionContext, NavRuntimeState) {
        let committed = schema::CommittedMapIdentity {
            session_id: "stage-a-map".into(), session_generation: 1, revision: 2,
            commit_boot_nonce: [3; 16], commit_sequence: 4, source_timestamp_ns: 5,
            frame_id: "map".into(),
        };
        let (_, rx) = tokio::sync::watch::channel(None);
        let admission = NavAdmissionContext::new(
            Arc::new(tokio::sync::Mutex::new(())),
            Arc::new(StdMutex::new(Some(committed))), rx,
        );
        let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 7);
        (admission, state)
    }

    #[tokio::test]
    async fn goal_intent_self_hash_public_construction_preserves_options() {
        let (admission, state) = self_hash_stage_context();
        let mut input = test_goal_input("stage-a-public");
        input.options.allow_reverse = true;
        input.options.position_tolerance_m = 0.12;
        input.options.yaw_tolerance_rad = std::f64::consts::PI;
        let expected = input.options;
        let intent = goal_intent_from_set_goal(&admission, &state, input, None).await.unwrap();
        assert_eq!(intent.options, expected);
        assert_eq!(intent.producer, "public-navigation");
        assert_eq!(intent.producer_epoch, 7);
        assert_eq!(intent.goal.header.frame_id, "map");
        assert!(intent.boundary_points.is_empty());
        for invalid in [-1.0, f64::NAN, f64::INFINITY] {
            let mut input = test_goal_input("stage-a-invalid-options");
            input.options.position_tolerance_m = invalid;
            assert!(goal_intent_from_set_goal(&admission, &state, input, None)
                .await.unwrap_err().contains("导航目标容差无效"));
        }
        assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
    }

    #[tokio::test]
    async fn goal_intent_self_hash_wavefront_construction_keeps_original_identity() {
        use nuwax_nav_types_messages::geometry::{Pose3, PoseStamped, Quaternion, Timestamp};
        use nuwax_nav_types_messages::frames::NavFrameId;
        use nuwax_nav_types_messages::goal::NavigationGoal;
        let (admission, _) = self_hash_stage_context();
        let committed = admission.committed_map_authority.lock().unwrap().clone().unwrap();
        let goal = NavigationGoal::new("original-frontier", PoseStamped {
            stamp: Timestamp::from_unix(1, 2), frame_id: NavFrameId::Map,
            pose: Pose3 { orientation: Quaternion::IDENTITY, ..Pose3::default() },
        });
        let mut intent = crate::explore_runtime::build_wavefront_goal_intent(
            9, 4, "parent-exploration", "original-child", &test_motion_admission(),
            &goal, None, committed,
        ).unwrap();
        assert_eq!(intent.producer, "wavefront");
        assert_eq!(intent.producer_epoch, 9);
        assert_eq!(intent.cancel_generation, 4);
        assert_eq!(intent.parent_request_id, "parent-exploration");
        assert_eq!(intent.session_id, "original-child");
        intent.options.allow_reverse = true;
        intent.options.yaw_tolerance_rad = std::f64::consts::PI;
        assert!(intent.options.validate().is_ok());
    }

    #[tokio::test]
    async fn goal_intent_self_hash_admission_rejects_before_legacy_wire_hashes() {
        let (admission, state) = self_hash_stage_context();
        let backend = BlockingPublishBackend::new([None]);
        let io = NavAdmissionIo::new(Arc::new(RwLock::new(SharedState::default())),
            LcmTransport::new(Arc::new(backend.clone())), "blocking-test", true, Arc::from("base_link"),
);
        let original = goal_intent_from_set_goal(&admission, &state,
            test_goal_input("stage-a-rejections"), None).await.unwrap();
        for source in ["public-navigation", "mapping-cloud", "wavefront"] {
            for mutation in ["options-position", "options-yaw", "pose", "frame", "seconds",
                "nanoseconds", "sequence", "producer-epoch", "map-generation"] {
                let mut intent = original.clone();
                intent.producer = source.into();
                if source == "wavefront" {
                    intent.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
                    intent.priority = 100;
                }
                // 有效目标选项进入准入，其它具名输入边界仍逐项验证。
                intent.options.allow_reverse = true;
                intent.options.yaw_tolerance_rad = std::f64::consts::PI;
                let expected = match mutation {
                    "options-position" => { intent.options.position_tolerance_m = f64::NAN; "导航目标容差无效" }
                    "options-yaw" => { intent.options.yaw_tolerance_rad = -1.0; "导航目标容差无效" }
                    "pose" => { intent.goal.pose.position[0] = f64::INFINITY; "goal header, frame, timestamp, or pose is invalid" }
                    "frame" => { intent.goal.header.frame_id = "odom".into(); "goal header, frame, timestamp, or pose is invalid" }
                    "seconds" => { intent.goal.header.stamp_sec = -1; "goal header, frame, timestamp, or pose is invalid" }
                    "nanoseconds" => { intent.goal.header.stamp_nsec = 1_000_000_000; "goal header, frame, timestamp, or pose is invalid" }
                    "sequence" => { intent.goal.header.seq = 1; "goal header, frame, timestamp, or pose is invalid" }
                    "producer-epoch" => { intent.producer_epoch += 1; "invalid producer, owner, priority, or generation identity" }
                    "map-generation" => { intent.map_frame_generation += 1; "map session or coordinate generation changed" }
                    _ => unreachable!("具名输入变体已列完")
                };
                let authority = match source {
                    "wavefront" => GoalProducerEpochAuthority::Wavefront {
                boundary_owner: Arc::new(crate::explore_runtime::ExploreStartConfig::default()),
                        current_session_generation: Arc::new(AtomicU64::new(7)),
                    },
                    "mapping-cloud" => GoalProducerEpochAuthority::MappingCloud,
                    _ => GoalProducerEpochAuthority::PublicNavigation,
                };
                let outcome = admit_goal_intent(&admission, &io, &state, intent, authority).await;
                assert_eq!(outcome.state.status, "FAILED", "{source}/{mutation}");
                assert!(outcome.state.message.contains(expected), "{source}/{mutation}: {}", outcome.state.message);
                assert!(outcome.lease.is_none());
            }
        }
        assert_eq!(backend.publish_count(), 0);
        assert_eq!(state.next_goal_generation.load(AtomicOrdering::Acquire), 0);
        assert!(state.inner.read().await.goal.is_none());
        assert_eq!(state.cancel_generation(), 0);
    }
    #[tokio::test]
    async fn assigned_goal_replay_uses_action_values_and_original_body_binding() {
        let (mut handle, backend, owner) = make_unit_blocking_handle([None]).await;
        handle.body_reference_frame_id = Arc::from("robot/body_a");
        backend.publish_gate.add_permits(1);
        let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 1);
        observe_ready_execution_chain(&state).await;
        let mut input = test_goal_input("assigned-action-values");
        input.goal_generation = 61;
        let first = set_goal(&handle, &state, input.clone()).await;
        assert_eq!(first.state.status, "ACCEPTED", "{}", first.state.message);
        let mut replay = input.clone();
        replay.target.x = -0.0;
        replay.target.timestamp_ns += 1;
        let same = set_goal(&handle, &state, replay).await;
        assert_eq!(same.lease, first.lease);
        assert!(same.state.message.contains("GOAL_LEASE_IDEMPOTENT"));
        for changed in [0, 1, 2] {
            let mut different = input.clone();
            match changed {
                0 => different.target.x = 1.0,
                1 => different.options.allow_reverse = !different.options.allow_reverse,
                _ => different.options.position_tolerance_m += 0.1,
            }
            let rejected = set_goal(&handle, &state, different).await;
            assert_eq!(rejected.state.status, "FAILED");
            assert!(rejected.state.message.contains("GOAL_GENERATION_STALE"));
        }
        assert_eq!(backend.publish_count(), 1);
        let frames = backend.frames.lock().unwrap();
        let envelope = schema::decode_goal_envelope_body(&frames[0].payload).unwrap();
        assert_eq!(envelope.body_reference_frame_id, "robot/body_a");
        drop(frames);
        drop(handle);
        owner.finish().await.unwrap();
    }

    #[tokio::test]
    async fn path_key_keeps_original_revision_watermark_across_planner_domains() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("planner-domain-watermark", 3);
        { let mut slot = state.inner.write().await; slot.phase = NavPhase::Planning;
            slot.goal = Some(test_active_goal_with_lease(lease.clone())); }
        let old = sealed_path(&lease, 1, 1.0);
        state.observe_path_envelope(&old).await.unwrap();
        let mut new = sealed_path(&lease, 2, 2.0);
        new.planner_runtime_generation = 9;
        state.observe_path_envelope(&new).await.unwrap();
        let promoted = state.correlate_navigation_state(&lease.session_id, lease.owner_kind,
            &lease.parent_request_id, lease.goal_generation, 2, lease.producer_runtime_generation, 9)
            .await.unwrap().unwrap();
        assert_eq!(promoted.path_revision, 2);
        assert!(state.correlate_navigation_state(&lease.session_id, lease.owner_kind,
            &lease.parent_request_id, lease.goal_generation, 1, lease.producer_runtime_generation, 1).await.is_err());
        assert!(state.observe_path_envelope(&old).await.is_err());
        let mut same_revision_other_planner = new;
        same_revision_other_planner.planner_runtime_generation = 10;
        assert!(state.observe_path_envelope(&same_revision_other_planner).await.is_err());
        assert_eq!(state.active_goal_lease().await.unwrap(), promoted);
    }

    #[tokio::test]
    async fn all_goal_producers_emit_v9_from_original_body_and_boundary_owners_without_hash() {
        let (mut handle, backend, owner) = make_unit_blocking_handle([None, None, None]).await;
        handle.body_reference_frame_id = Arc::from("robot/body_a");
        backend.publish_gate.add_permits(3);
        let body = Arc::clone(&handle.body_reference_frame_id);
        struct NoHash;
        impl Drop for NoHash { fn drop(&mut self) { crate::map_checksum::FORBID_HASH.with(|flag| flag.set(false)); } }
        crate::map_checksum::FORBID_HASH.with(|flag| flag.set(true));
        let _no_hash = NoHash;
        for (index, source) in ["public-navigation", "mapping-cloud", "wavefront"].into_iter().enumerate() {
            let state = NavRuntimeState::with_profile_and_producer_runtime_generation(PROFILE_SIMPLE_2D, 70 + index as u64);
            observe_ready_execution_chain(&state).await;
            let mut input = test_goal_input(source);
            input.goal_generation = 5;
            if source == "public-navigation" {
                let result = set_goal(&handle, &state, input).await;
                assert_eq!(result.state.status, "ACCEPTED", "{}", result.state.message);
                continue;
            }
            let admission = NavAdmissionContext::new(handle.mapping_lifecycle_lock.clone(),
                handle.committed_map_authority.clone(), handle.map_revision_latest_tx.subscribe())
                .with_mapping_adapter(handle.mapping_adapter.clone());
            let io = NavAdmissionIo::new(handle.state.clone(), handle.transport.clone(),
                handle.transport_backend_name, handle.lcm_backend_available, Arc::clone(&body));
            let (intent, authority) = if source == "mapping-cloud" {
                let mut intent = goal_intent_from_set_goal(&admission, &state, input, None).await.unwrap();
                intent.producer = source.into();
                (intent, GoalProducerEpochAuthority::MappingCloud)
            } else {
                use nuwax_nav_types_messages::geometry::{Point2, Point3, Pose3, PoseStamped, Quaternion, Timestamp};
                let owner = Arc::new(crate::explore_runtime::ExploreStartConfig {
                    boundary: Some(nuwax_exploration::ExplorationBoundary::from_vertices(vec![
                        Point2 { x: -1.0, y: -1.0 }, Point2 { x: 2.0, y: -1.0 }, Point2 { x: 2.0, y: 2.0 },
                    ]).unwrap()),
                    ..Default::default()
                });
                let goal = nuwax_nav_types_messages::goal::NavigationGoal::new("wavefront", PoseStamped {
                    frame_id: nuwax_nav_types_messages::frames::NavFrameId::Map,
                    stamp: Timestamp::from_unix(1_700_000_000, 0),
                    pose: Pose3 { position: Point3::new(0.0, 0.0, 0.0), orientation: Quaternion::IDENTITY },
                });
                let committed = handle.committed_map_authority.lock().unwrap().clone().unwrap();
                let mut intent = crate::explore_runtime::build_wavefront_goal_intent(
                    state.producer_runtime_generation(), 0, "wavefront-parent", "wavefront", &test_motion_admission(),
                    &goal, owner.boundary.as_ref(), committed).unwrap();
                intent.map_frame_generation = admission.map_frame_generation();
                intent.goal_generation = 5;
                (intent, GoalProducerEpochAuthority::Wavefront {
                    current_session_generation: Arc::new(AtomicU64::new(state.producer_runtime_generation())), boundary_owner: owner,
                })
            };
            let result = admit_goal_intent(&admission, &io, &state, intent, authority).await;
            assert_eq!(result.state.status, "ACCEPTED", "{source}: {}", result.state.message);
        }
        let frames = backend.frames.lock().unwrap();
        assert_eq!(frames.len(), 3);
        for (index, frame) in frames.iter().enumerate() {
            let goal = schema::decode_goal_envelope_body(&frame.payload).unwrap();
            assert_eq!(goal.schema_version, 9);
            assert_eq!(goal.body_reference_frame_id, "robot/body_a");
            assert_eq!(goal.producer_runtime_generation, 70 + index as i64);
            if goal.producer == "wavefront" {
                assert_eq!(goal.boundary_points, vec![[-1.0,-1.0,0.0],[2.0,-1.0,0.0],[2.0,2.0,0.0]]);
            } else { assert!(goal.boundary_points.is_empty()); }
        }
        drop(frames);
        drop(handle);
        owner.finish().await.unwrap();
    }

}
