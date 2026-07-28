// Phase 6 (codex-round3 R3-4):frame_id / target / accepted_at_ns / inner /
// build_nav_stack_handle / render_dry_run / readiness_from_reason 是 Phase 4
// nav RuntimeDiagnostics platform API 预留(共享 nuwax-nav-stack handle 与
// 探测日志 trace 入口)。uninlined_format_args 是 debug log 模板代码风格,
// 不动语义。
#![allow(dead_code, clippy::uninlined_format_args)]

//! Phase 4:nav runtime adapter —— 把 `SetNavigationGoal` / `CancelNavigation` /
//! `SubscribeNavigationState` / `GetNavigationReadiness` 从 `UNIMPLEMENTED` 变成
//! 真实导航执行链。
//!
//! ## 核心职责
//!
//! 1. 读 `NUWAX_NAV_PROFILE` env,默认 `simple-2d`,可选 `replanning-astar` /
//!    `cmu-native` / `exploration-native` / `mls3d`(本阶段只实装 `simple-2d`
//!    + `replanning-astar` 的纯 Rust 路径,其它 profile 标记为 P1 native)。
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
//!   state 推进，`CANCELLED` 必须由 motion guard `ZEROED` ack 推进。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use nuwax_codec::lcm::schema;
use nuwax_nav_stack::{
    graph::ModuleNode, NavStackBuilder, NavStackHandle, NavStackPreset, NavStackProfile,
};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
use serde::{Deserialize, Serialize};
use tokio::sync::RwLock;
use tracing::{info, warn};
use uuid::Uuid;

use crate::channel_registry;
use crate::lcm_runtime::{GeometryRuntimeHandle, NavState, PoseState};
use crate::sequence_state::{SequenceStateError, SequenceStateStore};
use crate::typed_codec;

/// 导航 profile 字符串常量。env `NUWAX_NAV_PROFILE` 直接吃这些字符串,
/// 解析失败时回退到 `simple-2d`。
pub const PROFILE_SIMPLE_2D: &str = "simple-2d";
/// `replanning-astar` profile 字符串。
pub const PROFILE_REPLANNING_ASTAR: &str = "replanning-astar";
/// `cmu-native` profile 字符串(P1:依赖 native bridge 进程,本期未实装)。
pub const PROFILE_CMU_NATIVE: &str = "cmu-native";
/// `exploration-native` profile 字符串(P1:依赖 native bridge 进程,本期未实装)。
pub const PROFILE_EXPLORATION_NATIVE: &str = "exploration-native";
/// `mls-3d` profile 字符串(P1:`nuwax-nav-mls-planner` feature 未开)。
/// 注意:与 `NavStackProfile::Mls3D.as_str()` 一致,使用 kebab-case "mls-3d"
/// 而非 "mls3d",否则 `build_nav_stack_handle_renders_for_all_profiles`
/// 测试中 `text.contains(p)` 会失败。
pub const PROFILE_MLS3D: &str = "mls-3d";

/// 默认 profile(env 未设置或非法值时)。
pub const DEFAULT_PROFILE: &str = PROFILE_SIMPLE_2D;

static NEXT_NAV_RUNTIME_GENERATION: AtomicU64 = AtomicU64::new(0);

/// `/nav/goal` typed channel 平台级 channel name。
pub const GOAL_CHANNEL_PLATFORM: &str = "NUWAX_LCM_CHANNEL_GOAL_POSE";
/// `/nav/goal` 本进程内 alias。
pub const GOAL_CHANNEL_RUNTIME: &str = "NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL";
/// `/nav/goal` 内置 default。
pub const GOAL_CHANNEL_DEFAULT: &str = "/nav/goal#nuwax_msgs.GoalEnvelope";
/// `/nav/path` versioned planner output platform key.
pub const PATH_CHANNEL_PLATFORM: &str = "NUWAX_LCM_CHANNEL_NAV_PATH";
/// `/nav/path` versioned planner output runtime alias.
pub const PATH_CHANNEL_RUNTIME: &str = "NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL";
/// `/nav/path` immutable envelope default.
pub const PATH_CHANNEL_DEFAULT: &str = "/nav/path#nuwax_msgs.PathEnvelope";

/// GoalEnvelope message type validated once at startup.
/// 失败应直接 panic(因为这是 typed protocol 层,不可能 fail)。
fn goal_msg_type() -> LcmMessageType {
    LcmMessageType::new("nuwax_msgs.GoalEnvelope").expect("typed nuwax_msgs.GoalEnvelope")
}

/// PathEnvelope message type validated once at startup.
fn path_msg_type() -> LcmMessageType {
    LcmMessageType::new("nuwax_msgs.PathEnvelope").expect("typed nuwax_msgs.PathEnvelope")
}

/// 单一目标状态机阶段。`status` 字段直接对应 gRPC `NavigationState.status`,
/// 这样 Python 侧 `get_navigation_state` 不用再翻译。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavPhase {
    /// 空闲(初始 / 已被 cancel / 已被 succeed 后复位)。
    Idle,
    /// 收到 set_goal,已写 active goal + 已发布 /nav/goal,等待进入规划。
    Accepted,
    /// 正在规划路径。
    Planning,
    /// 规划成功,正在跟随路径生成 cmd_vel。
    Running,
    /// 到达目标点(距离 < arrival_tolerance_m)。
    Succeeded,
    /// 用户取消，且唯一 motion guard actuator path 已确认 ZEROED。
    Cancelled,
    /// cancel 已提交，等待 motion guard ZEROED acknowledgement。
    CancelPending,
    /// 失败(readiness 不通过 / 路径无法生成 / 等)。
    Failed,
}

impl NavPhase {
    /// 序列化成 gRPC `NavigationState.status` 字符串。
    pub fn as_str(self) -> &'static str {
        match self {
            NavPhase::Idle => "IDLE",
            NavPhase::Accepted => "ACCEPTED",
            NavPhase::Planning => "PLANNING",
            NavPhase::Running => "RUNNING",
            NavPhase::Succeeded => "SUCCEEDED",
            NavPhase::Cancelled => "CANCELLED",
            NavPhase::CancelPending => "PENDING",
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
    /// Canonical sealed goal digest.
    pub goal_payload_sha256: [u8; 32],
    /// Last scheduler-confirmed planner path revision.
    pub path_revision: u64,
    /// Canonical scheduler-confirmed path digest, zero before confirmation.
    pub path_payload_sha256: [u8; 32],
}

impl GoalLease {
    fn identity_matches(&self, other: &Self) -> bool {
        self.session_id == other.session_id
            && self.owner_kind == other.owner_kind
            && self.parent_request_id == other.parent_request_id
            && self.goal_generation == other.goal_generation
    }
}

/// 当前 active goal 的内存表示。`id` 是唯一 public session id。
#[derive(Clone)]
pub struct ActiveGoal {
    /// 目标点 ID(用户传或自动生成 UUID)。
    pub id: String,
    /// frame_id(`map` 默认)。
    pub frame_id: String,
    /// 目标位姿(在 `frame_id` 坐标系下)。
    pub target: PoseState,
    /// 收到 `set_goal` 的本地墙钟(ns),给 `timestamp_ns` 字段用。
    pub accepted_at_ns: u64,
    /// Immutable owner/hash identity used by goal/path/cancel CAS.
    pub lease: GoalLease,
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
    path_candidates: VecDeque<(u64, [u8; 32])>,
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
    inner: Arc<RwLock<NavRuntimeSlot>>,
    stack_observation: Arc<StdMutex<StackObservation>>,
    runtime_observations: Arc<StdMutex<RuntimeObservations>>,
    cancel: Arc<StdMutex<CancelTracker>>,
    goal_command_gate: Arc<RwLock<()>>,
    producer_runtime_generation: u64,
    next_goal_generation: Arc<AtomicU64>,
    next_cancel_generation: Arc<AtomicU64>,
}

#[derive(Clone, Default)]
struct StackObservation {
    observed_at: Option<Instant>,
    state: Option<NavState>,
}

const DEFAULT_STACK_STATE_TTL: Duration = Duration::from_secs(2);
const DEFAULT_TF_TTL: Duration = Duration::from_secs(2);
const DEFAULT_PLANNER_INPUT_TTL: Duration = Duration::from_secs(2);
const DEFAULT_MOTION_SINK_TTL: Duration = Duration::from_secs(1);
const DEFAULT_CANCEL_TIMEOUT: Duration = Duration::from_secs(5);
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

/// Cancellation lifecycle independent from the navigation planner status.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CancelPhase {
    /// No cancellation is active.
    #[default]
    Idle,
    /// Cancellation is submitted and awaits the actuator ZEROED acknowledgement.
    Pending,
    /// Motion guard acknowledged that actuator output is zero.
    Zeroed,
    /// Motion guard or the cancellation dispatch reported a failure.
    Failed,
    /// No terminal guard acknowledgement arrived before the deadline.
    TimedOut,
}

/// Guard acknowledgement kind for a cancellation command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MotionSinkCancelAckStatus {
    /// The guard applied zero velocity to the actuator.
    Zeroed,
    /// The guard rejected or failed the zero command.
    Failed,
}

/// Acknowledgement returned by the unique guard-backed actuator path.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MotionSinkCancelAck {
    /// Generation returned by [`NavRuntimeState::cancel_snapshot`].
    pub cancel_generation: u64,
    /// Goal associated with the cancellation.
    pub goal_id: String,
    /// Lease owner kind returned by NavigationState.
    pub owner_kind: i32,
    /// Lease parent request returned by NavigationState.
    pub parent_request_id: String,
    /// Lease generation returned by NavigationState.
    pub goal_generation: u64,
    /// Guard result.
    pub status: MotionSinkCancelAckStatus,
    /// Stable diagnostic detail supplied by the scheduler/guard client.
    pub message: String,
}

/// Public typed snapshot used to correlate scheduler/guard cancellation work.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CancelSnapshot {
    /// Monotonic cancellation command generation.
    pub generation: u64,
    /// Goal associated with the cancellation.
    pub goal_id: String,
    /// Full goal lease being cancelled, retained through terminal projection.
    pub lease: Option<GoalLease>,
    /// Current typed cancellation phase.
    pub phase: CancelPhase,
    /// Whether nav-stack has already reported its cancellation terminal.
    pub stack_terminal_observed: bool,
    /// Stable machine-oriented diagnostic detail.
    pub message: String,
}

#[derive(Clone)]
struct CancelTracker {
    generation: u64,
    goal_id: String,
    lease: Option<GoalLease>,
    phase: CancelPhase,
    requested_at: Option<Instant>,
    stack_terminal_observed: bool,
    message: String,
}

enum CancelStart {
    Publish {
        lease: GoalLease,
        generation: u64,
        goal_id: String,
    },
    ExistingPending {
        lease: GoalLease,
        generation: u64,
        goal_id: String,
        message: String,
    },
    Rejected {
        goal_id: String,
        message: String,
    },
}

impl Default for CancelTracker {
    fn default() -> Self {
        Self {
            generation: 0,
            goal_id: String::new(),
            lease: None,
            phase: CancelPhase::Idle,
            requested_at: None,
            stack_terminal_observed: false,
            message: String::new(),
        }
    }
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
    /// 新建 nav runtime:读 env profile、build module graph、把节点 ID
    /// 写入 slot。
    pub fn from_env() -> Self {
        let producer_runtime_generation = NEXT_NAV_RUNTIME_GENERATION
            .fetch_update(AtomicOrdering::AcqRel, AtomicOrdering::Acquire, |current| {
                current
                    .checked_add(1)
                    .filter(|next| *next <= i64::MAX as u64)
            })
            .map(|previous| previous + 1)
            .expect("process-local test producer generation exhausted");
        Self::from_env_with_producer_runtime_generation(producer_runtime_generation)
    }

    /// Construct the production runtime only after durably reserving a producer
    /// epoch that remains newer after a geometry process restart.
    pub(crate) fn try_from_sequence_store(
        sequence_store: &SequenceStateStore,
    ) -> Result<Self, SequenceStateError> {
        let producer_runtime_generation = sequence_store.reserve_producer_runtime_generation()?;
        Ok(Self::from_env_with_producer_runtime_generation(
            producer_runtime_generation,
        ))
    }

    fn from_env_with_producer_runtime_generation(producer_runtime_generation: u64) -> Self {
        let profile = resolve_profile_from_env();
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
            inner: Arc::new(RwLock::new(slot)),
            stack_observation: Arc::new(StdMutex::new(StackObservation::default())),
            runtime_observations: Arc::new(StdMutex::new(RuntimeObservations::default())),
            cancel: Arc::new(StdMutex::new(CancelTracker::default())),
            goal_command_gate: Arc::new(RwLock::new(())),
            producer_runtime_generation,
            next_goal_generation: Arc::new(AtomicU64::new(0)),
            next_cancel_generation: Arc::new(AtomicU64::new(0)),
        }
    }

    /// 读内部 slot 句柄(给 server.rs 写 diagnostics 用)。
    pub fn inner(&self) -> Arc<RwLock<NavRuntimeSlot>> {
        self.inner.clone()
    }

    /// Stable producer generation for the lifetime of this runtime instance.
    pub fn producer_runtime_generation(&self) -> u64 {
        self.producer_runtime_generation
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

    /// Validate and retain planner output without replacing a path already
    /// confirmed by the scheduler.
    pub async fn observe_path_envelope(
        &self,
        path: &schema::PathEnvelope,
    ) -> Result<GoalLease, String> {
        schema::validate_path_envelope(path).map_err(|error| error.to_string())?;
        let path_goal_generation = u64::try_from(path.goal_generation)
            .map_err(|_| "PathEnvelope goal_generation is invalid".to_string())?;
        let path_revision = u64::try_from(path.path_revision)
            .map_err(|_| "PathEnvelope path_revision is invalid".to_string())?;
        let mut slot = self.inner.write().await;
        let confirmed = slot
            .goal
            .as_ref()
            .map(|active| active.lease.clone())
            .ok_or_else(|| "PATH_LEASE_REJECTED: no active goal lease".to_string())?;
        if path.session_id != confirmed.session_id
            || path.owner_kind != confirmed.owner_kind
            || path.parent_request_id != confirmed.parent_request_id
            || path_goal_generation != confirmed.goal_generation
            || path.goal_payload_sha256 != confirmed.goal_payload_sha256
        {
            return Err(
                "PATH_LEASE_REJECTED: identity/goal hash does not match active lease".to_string(),
            );
        }
        let cancel = self
            .cancel
            .lock()
            .map_err(|_| "PATH_LEASE_REJECTED: cancel state unavailable".to_string())?;
        let cancel_freezes_lease =
            cancel.phase != CancelPhase::Idle && cancel.lease.as_ref() == Some(&confirmed);
        drop(cancel);
        if cancel_freezes_lease {
            return Err("PATH_LEASE_REJECTED[CANCEL_PENDING]: active lease is frozen".to_string());
        }

        let candidate_goal_matches =
            slot.path_candidate_goal
                .as_ref()
                .is_some_and(|candidate_goal| {
                    candidate_goal.identity_matches(&confirmed)
                        && candidate_goal.producer_runtime_generation
                            == confirmed.producer_runtime_generation
                        && candidate_goal.goal_payload_sha256 == confirmed.goal_payload_sha256
                });
        if !candidate_goal_matches {
            slot.path_candidate_goal = Some(confirmed.clone());
            slot.path_candidates.clear();
        }
        let (latest_revision, latest_hash) = slot
            .path_candidates
            .back()
            .copied()
            .unwrap_or((confirmed.path_revision, confirmed.path_payload_sha256));
        if path_revision < latest_revision {
            return Err("PATH_LEASE_REJECTED: stale path revision".to_string());
        }
        if path_revision == latest_revision {
            if path.path_payload_sha256 == latest_hash {
                let mut observed = confirmed;
                observed.path_revision = latest_revision;
                observed.path_payload_sha256 = latest_hash;
                return Ok(observed);
            }
            return Err("PATH_LEASE_REJECTED: conflicting path hash at same revision".to_string());
        }

        let mut observed = confirmed.clone();
        observed.path_revision = path_revision;
        observed.path_payload_sha256 = path.path_payload_sha256;
        if slot.path_candidates.len() == MAX_PENDING_PATH_CANDIDATES {
            slot.path_candidates.pop_front();
        }
        slot.path_candidates
            .push_back((path_revision, path.path_payload_sha256));
        Ok(observed)
    }

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
        goal_payload_sha256: [u8; 32],
        path_payload_sha256: [u8; 32],
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
            if goal_payload_sha256 != lease.goal_payload_sha256 {
                fields.push("goal_payload_sha256");
            }
            if path_revision != lease.path_revision {
                fields.push("path_revision");
            }
            if path_payload_sha256 != lease.path_payload_sha256 {
                fields.push("path_payload_sha256");
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
            let retained = self
                .cancel
                .lock()
                .ok()
                .and_then(|cancel| cancel.lease.clone())
                .ok_or_else(|| "NAV_STATE_LEASE_REJECTED: no matching lease".to_string())?;
            let fields = mismatched_fields(&retained);
            return if fields.is_empty() {
                Ok(Some(retained))
            } else {
                Err(format!(
                    "NAV_STATE_LEASE_REJECTED: mismatched fields: {}",
                    fields.join(",")
                ))
            };
        };
        let goal_identity_matches = session_id == confirmed.session_id
            && owner_kind == confirmed.owner_kind
            && parent_request_id == confirmed.parent_request_id
            && goal_generation == confirmed.goal_generation
            && goal_payload_sha256 == confirmed.goal_payload_sha256;
        let confirmed_path_matches = path_revision == confirmed.path_revision
            && path_payload_sha256 == confirmed.path_payload_sha256;
        if goal_identity_matches && confirmed_path_matches {
            return Ok(Some(confirmed));
        }
        if !goal_identity_matches {
            return Err(rejection(&confirmed));
        }

        let cancel = self
            .cancel
            .lock()
            .map_err(|_| "NAV_STATE_LEASE_REJECTED: cancel state unavailable".to_string())?;
        let cancel_freezes_lease =
            cancel.phase != CancelPhase::Idle && cancel.lease.as_ref() == Some(&confirmed);
        drop(cancel);
        if !cancel_freezes_lease {
            let candidate_goal_matches =
                slot.path_candidate_goal
                    .as_ref()
                    .is_some_and(|candidate_goal| {
                        candidate_goal.identity_matches(&confirmed)
                            && candidate_goal.producer_runtime_generation
                                == confirmed.producer_runtime_generation
                            && candidate_goal.goal_payload_sha256 == confirmed.goal_payload_sha256
                    });
            let candidate_matches = candidate_goal_matches
                && slot.path_candidates.iter().any(|(revision, hash)| {
                    *revision == path_revision && *hash == path_payload_sha256
                });
            if candidate_matches {
                let mut promoted = confirmed;
                promoted.path_revision = path_revision;
                promoted.path_payload_sha256 = path_payload_sha256;
                slot.goal.as_mut().expect("active goal checked above").lease = promoted.clone();
                while slot
                    .path_candidates
                    .front()
                    .is_some_and(|(revision, _)| *revision <= path_revision)
                {
                    slot.path_candidates.pop_front();
                }
                return Ok(Some(promoted));
            }
        }
        Err(rejection(&confirmed))
    }

    /// 当前 profile(用于 diagnostics 暴露)。
    pub async fn profile(&self) -> &'static str {
        self.inner.read().await.profile
    }

    /// 当前 module graph 节点列表(diagnostics 用)。
    pub async fn module_nodes(&self) -> Vec<String> {
        self.inner.read().await.module_nodes.clone()
    }

    /// 当前阶段 + message(转成 NavState 返回值)。
    pub async fn snapshot(&self, goal_id: String, distance_remaining: f32) -> NavState {
        let slot = self.inner.read().await;
        NavState {
            status: slot.phase.as_str().to_string(),
            message: slot.message.clone(),
            goal_id,
            distance_remaining,
            timestamp_ns: now_ns(),
        }
    }

    /// Record one state emitted by the real stack and normalize an omitted
    /// session id to the facade's currently active goal.
    pub async fn observe_stack_state(&self, observed: NavState) -> NavState {
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

    /// Atomically apply lease-owned consumer observations and stack state only
    /// while the previously correlated lease remains the active incarnation.
    pub(crate) async fn observe_correlated_navigation_state(
        &self,
        observed: NavState,
        expected_lease: &GoalLease,
        planner_observation: Option<PlannerInputObservation>,
        motion_sink_observation: Option<MotionSinkObservation>,
    ) -> Result<NavState, String> {
        self.observe_stack_state_inner(
            observed,
            Some(expected_lease),
            planner_observation,
            motion_sink_observation,
        )
        .await
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
            let cancel_gate = if observed_phase.is_some_and(NavPhase::is_terminal) {
                self.cancel.lock().ok().and_then(|mut cancel| {
                    let active_lease_matches = cancel.lease.as_ref().is_some_and(|lease| {
                        slot.goal
                            .as_ref()
                            .is_some_and(|active| active.lease == *lease)
                    });
                    let already_zeroed = cancel.phase == CancelPhase::Zeroed
                        && slot.phase == NavPhase::Cancelled
                        && slot.goal.is_none()
                        && cancel.goal_id == observed.goal_id;
                    if cancel.phase != CancelPhase::Idle && (active_lease_matches || already_zeroed)
                    {
                        cancel.stack_terminal_observed = true;
                        Some(cancel.phase)
                    } else {
                        None
                    }
                })
            } else {
                None
            };
            if cancel_gate == Some(CancelPhase::Pending) {
                slot.phase = NavPhase::CancelPending;
                slot.message =
                    "CANCEL_WAITING_FOR_ZERO_ACK: nav-stack terminal observed".to_string();
                observed.status = NavPhase::CancelPending.as_str().to_string();
                observed.message = slot.message.clone();
            } else if matches!(
                cancel_gate,
                Some(CancelPhase::Failed | CancelPhase::TimedOut)
            ) {
                slot.phase = NavPhase::Failed;
                slot.message = "CANCEL_NOT_ZEROED: nav-stack terminal is not actuator ack".into();
                observed.status = NavPhase::Failed.as_str().to_string();
                observed.message = slot.message.clone();
            } else if cancel_gate == Some(CancelPhase::Zeroed) {
                slot.phase = NavPhase::Cancelled;
                slot.goal = None;
                slot.message = "CANCEL_ZEROED: actuator acknowledgement is authoritative".into();
                observed.status = NavPhase::Cancelled.as_str().to_string();
                observed.message = slot.message.clone();
            } else if let Some(phase) = observed_phase {
                slot.phase = phase;
                slot.message = observed.message.clone();
                if phase.is_terminal() {
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

    /// Synchronous liveness poll used by process-level diagnostics.
    pub fn stack_is_live(&self) -> bool {
        self.stack_health_snapshot().fresh
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

    /// Record an unowned scheduler heartbeat after its guard transport connected.
    ///
    /// Idle heartbeats do not own a guard lease, so they intentionally carry no
    /// guard generation. Goal admission still requires a positive request proof,
    /// and every owned navigation update still requires a positive generation.
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

    /// Synchronous coarse gate used by process diagnostics.
    pub fn execution_chain_is_live(&self) -> bool {
        let snapshot = self.runtime_observation_snapshot();
        let profile_supported = self
            .inner
            .try_read()
            .is_ok_and(|slot| matches!(slot.profile, PROFILE_SIMPLE_2D | PROFILE_REPLANNING_ASTAR));
        profile_supported
            && self.stack_is_live()
            && snapshot.tf_fresh
            && snapshot.tf_complete
            && snapshot.planner_fresh
            && snapshot.planner_ready
            && snapshot.planner_frame_matches
            && snapshot.motion_sink_fresh
            && snapshot.motion_sink_ready
    }

    /// Return the current cancellation generation and typed state.
    pub fn cancel_snapshot(&self) -> CancelSnapshot {
        self.cancel
            .lock()
            .ok()
            .map(|cancel| CancelSnapshot {
                generation: cancel.generation,
                goal_id: cancel.goal_id.clone(),
                lease: cancel.lease.clone(),
                phase: cancel.phase,
                stack_terminal_observed: cancel.stack_terminal_observed,
                message: cancel.message.clone(),
            })
            .unwrap_or(CancelSnapshot {
                generation: 0,
                goal_id: String::new(),
                lease: None,
                phase: CancelPhase::Failed,
                stack_terminal_observed: false,
                message: "CANCEL_STATE_UNAVAILABLE".to_string(),
            })
    }

    /// Apply a cancellation result from the unique motion-safety-guard path.
    pub async fn observe_cancel_ack(&self, ack: MotionSinkCancelAck) -> CancelOutcome {
        let mut slot = self.inner.write().await;
        let Ok(mut cancel) = self.cancel.lock() else {
            return cancel_outcome_from_slot(
                &slot,
                ack.goal_id,
                "CANCEL_STATE_UNAVAILABLE".to_string(),
                None,
            );
        };
        let goal_id = cancel.goal_id.clone();
        let lease = cancel.lease.clone();
        let active_lease_matches = lease.as_ref().is_some_and(|lease| {
            slot.goal
                .as_ref()
                .is_some_and(|active| active.lease == *lease)
        });
        let phase_accepts_ack = match ack.status {
            MotionSinkCancelAckStatus::Zeroed => matches!(
                cancel.phase,
                CancelPhase::Pending | CancelPhase::Failed | CancelPhase::TimedOut
            ),
            MotionSinkCancelAckStatus::Failed => cancel.phase == CancelPhase::Pending,
        };
        let ack_matches = phase_accepts_ack
            && cancel.generation == ack.cancel_generation
            && cancel.goal_id == ack.goal_id
            && lease.as_ref().is_some_and(|lease| {
                lease.owner_kind == ack.owner_kind
                    && lease.parent_request_id == ack.parent_request_id
                    && lease.goal_generation == ack.goal_generation
            });
        if !ack_matches || !active_lease_matches {
            return cancel_outcome_from_slot(
                &slot,
                goal_id,
                "CANCEL_ACK_REJECTED: generation, goal, or active lease mismatch".to_string(),
                lease,
            );
        }

        let message = ack.message;
        match ack.status {
            MotionSinkCancelAckStatus::Zeroed => {
                slot.phase = NavPhase::Cancelled;
                slot.goal = None;
                slot.message = format!("CANCEL_ZEROED: {message}");
                cancel.phase = CancelPhase::Zeroed;
            }
            MotionSinkCancelAckStatus::Failed => {
                slot.phase = NavPhase::Failed;
                slot.message = format!("CANCEL_ZERO_FAILED: {message}");
                cancel.phase = CancelPhase::Failed;
            }
        }
        cancel.message = message.clone();
        cancel_outcome_from_slot(&slot, goal_id, message, lease)
    }

    /// Transition an overdue pending cancellation to an explicit timeout.
    pub async fn poll_cancel_timeout(&self) -> Option<CancelOutcome> {
        let mut slot = self.inner.write().await;
        let mut cancel = self.cancel.lock().ok()?;
        let requested_at = cancel.requested_at?;
        if cancel.phase != CancelPhase::Pending || requested_at.elapsed() < cancel_timeout() {
            return None;
        }
        let lease = cancel.lease.clone()?;
        if !slot
            .goal
            .as_ref()
            .is_some_and(|active| active.lease == lease)
        {
            return None;
        }
        let goal_id = cancel.goal_id.clone();
        let message = "CANCEL_ZERO_ACK_TIMEOUT".to_string();
        slot.phase = NavPhase::Failed;
        slot.message = message.clone();
        cancel.phase = CancelPhase::TimedOut;
        cancel.message = message.clone();
        Some(cancel_outcome_from_slot(
            &slot,
            goal_id,
            message,
            Some(lease),
        ))
    }

    async fn begin_cancel(&self, input: &CancelGoalInput) -> CancelStart {
        let slot = self.inner.read().await;
        let Some(active) = slot.goal.as_ref() else {
            return CancelStart::Rejected {
                goal_id: input.goal_id.clone(),
                message: "CANCEL_REJECTED[GOAL_LEASE_MISSING]".to_string(),
            };
        };
        let active_lease = &active.lease;
        if input.goal_id.trim() != active_lease.session_id
            || input.owner_kind != active_lease.owner_kind
            || input.parent_request_id.trim() != active_lease.parent_request_id
            || input.goal_generation != active_lease.goal_generation
            || input.producer_runtime_generation != active_lease.producer_runtime_generation
        {
            return CancelStart::Rejected {
                goal_id: input.goal_id.clone(),
                message: "CANCEL_REJECTED[GOAL_LEASE_MISMATCH]".to_string(),
            };
        }
        let lease = active_lease.clone();
        let goal_id = lease.session_id.clone();
        let Ok(mut cancel) = self.cancel.lock() else {
            return CancelStart::Rejected {
                goal_id,
                message: "CANCEL_STATE_UNAVAILABLE".to_string(),
            };
        };
        if cancel.phase == CancelPhase::Pending {
            if cancel.lease.as_ref() == Some(&lease) {
                return CancelStart::ExistingPending {
                    lease,
                    generation: cancel.generation,
                    goal_id,
                    message: cancel.message.clone(),
                };
            }
            return CancelStart::Rejected {
                goal_id,
                message: "CANCEL_REJECTED[CANCEL_IN_FLIGHT_CONFLICT]".to_string(),
            };
        }
        let generation = match self.next_cancel_generation.fetch_update(
            AtomicOrdering::AcqRel,
            AtomicOrdering::Acquire,
            |current| {
                current
                    .checked_add(1)
                    .filter(|next| *next <= i64::MAX as u64)
            },
        ) {
            Ok(previous) => previous + 1,
            Err(_) => {
                return CancelStart::Rejected {
                    goal_id,
                    message: "CANCEL_REJECTED[CANCEL_GENERATION_EXHAUSTED]".to_string(),
                };
            }
        };
        *cancel = CancelTracker {
            generation,
            goal_id: goal_id.clone(),
            lease: Some(lease.clone()),
            phase: CancelPhase::Pending,
            requested_at: Some(Instant::now()),
            stack_terminal_observed: false,
            message: "CANCEL_PENDING_ZERO_ACK".to_string(),
        };
        CancelStart::Publish {
            lease,
            generation,
            goal_id,
        }
    }

    fn reset_cancel(&self) {
        if let Ok(mut cancel) = self.cancel.lock() {
            *cancel = CancelTracker::default();
        }
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
    pub(crate) fn expire_cancel_for_test(&self) {
        if let Ok(mut cancel) = self.cancel.lock() {
            cancel.requested_at =
                Instant::now().checked_sub(cancel_timeout() + Duration::from_millis(1));
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

fn cancel_timeout() -> Duration {
    duration_from_env("NUWAX_NAV_CANCEL_TIMEOUT_MS", DEFAULT_CANCEL_TIMEOUT)
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
    observed_at.is_some_and(|observed_at| observed_at.elapsed() <= ttl)
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

/// 解析 `NUWAX_NAV_PROFILE` env 值,失败时回退到 default。
pub fn resolve_profile_from_env() -> &'static str {
    match std::env::var("NUWAX_NAV_PROFILE") {
        Ok(v) => match v.trim() {
            PROFILE_SIMPLE_2D => PROFILE_SIMPLE_2D,
            PROFILE_REPLANNING_ASTAR => PROFILE_REPLANNING_ASTAR,
            PROFILE_CMU_NATIVE => PROFILE_CMU_NATIVE,
            PROFILE_EXPLORATION_NATIVE => PROFILE_EXPLORATION_NATIVE,
            PROFILE_MLS3D => PROFILE_MLS3D,
            other => {
                warn!(
                    "NUWAX_NAV_PROFILE={:?} 不识别,回退 default={}",
                    other, DEFAULT_PROFILE
                );
                DEFAULT_PROFILE
            }
        },
        Err(_) => DEFAULT_PROFILE,
    }
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

/// 把 `NavStackHandle` 渲染成 dry-run 字符串(给 diagnostics 字段用)。
pub fn render_dry_run(handle: &NavStackHandle) -> String {
    handle.dry_run()
}

/// Stable machine-readable readiness result code.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum NavigationReadinessCode {
    /// Compatibility default for payloads produced before typed codes existed.
    #[default]
    Unknown,
    /// Every required observation is fresh and usable.
    Ready,
    /// The real LCM backend is unavailable.
    LcmBackendUnavailable,
    /// No guard-backed motion sink has reported health.
    MotionSinkUnobserved,
    /// Motion-sink health exceeded its freshness deadline.
    MotionSinkStale,
    /// The observed motion sink explicitly reported not ready.
    MotionSinkNotReady,
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
            Self::MotionSinkUnobserved => "MOTION_SINK_UNOBSERVED",
            Self::MotionSinkStale => "MOTION_SINK_STALE",
            Self::MotionSinkNotReady => "MOTION_SINK_NOT_READY",
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
    /// Stable primary readiness result.
    #[serde(default)]
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
    /// Backward-compatible alias of the guard-backed sink readiness.
    pub cmd_vel_sink: bool,
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

const P1_NATIVE_PROFILES: &[&str] = &[
    PROFILE_CMU_NATIVE,
    PROFILE_EXPLORATION_NATIVE,
    PROFILE_MLS3D,
];

/// Compute readiness exclusively from observations written by real consumers.
pub async fn compute_readiness(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
) -> NavigationReadinessDetail {
    let profile = state.profile().await;
    let profile_supported = matches!(profile, PROFILE_SIMPLE_2D | PROFILE_REPLANNING_ASTAR);
    let stack_health = state.stack_health_snapshot();
    let observations = state.runtime_observation_snapshot();
    let lcm_backend = handle.lcm_backend_available;
    let tf = observations.tf_fresh && observations.tf_complete;
    let costmap = observations.planner_observed && observations.planner_fresh;
    let cmd_vel_sink_ready = observations.motion_sink_fresh && observations.motion_sink_ready;
    let planner = profile_supported
        && stack_health.fresh
        && observations.planner_fresh
        && observations.planner_ready
        && observations.planner_frame_matches
        && costmap;
    let cmd_vel_sink = cmd_vel_sink_ready;
    let backend = match handle.transport_backend_name {
        "real-lcm" | "real-lcm-unavailable" => "RealLcm",
        other => {
            // 不在这里 panic;未知 backend 按当前 enum 语义映射并记 log。
            tracing::warn!("nav readiness: 未知 transport_backend_name={}", other);
            "Unknown"
        }
    };
    let ready = lcm_backend && tf && costmap && planner && cmd_vel_sink_ready;
    let mut blockers = Vec::new();
    if !lcm_backend {
        blockers.push(NavigationReadinessCode::LcmBackendUnavailable);
    }
    if !observations.motion_sink_observed {
        blockers.push(NavigationReadinessCode::MotionSinkUnobserved);
    } else if !observations.motion_sink_fresh {
        blockers.push(NavigationReadinessCode::MotionSinkStale);
    } else if !observations.motion_sink_ready {
        blockers.push(NavigationReadinessCode::MotionSinkNotReady);
    }
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
        if !observations.planner_frame_matches {
            blockers.push(NavigationReadinessCode::PlannerFrameMismatch);
        }
        if !observations.planner_ready {
            blockers.push(NavigationReadinessCode::PlannerNotReady);
        }
    }
    if !profile_supported {
        blockers.push(NavigationReadinessCode::ProfileUnsupported);
    }
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
        cmd_vel_sink,
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

fn readiness_reason(code: NavigationReadinessCode) -> &'static str {
    match code {
        NavigationReadinessCode::Unknown => "UNKNOWN: readiness payload lacks a typed code",
        NavigationReadinessCode::Ready => "READY: guard-backed navigation chain is ready",
        NavigationReadinessCode::LcmBackendUnavailable => {
            "LCM_BACKEND_UNAVAILABLE: real LCM backend unavailable"
        }
        NavigationReadinessCode::MotionSinkUnobserved => {
            "MOTION_SINK_UNOBSERVED: guard client has not reported health"
        }
        NavigationReadinessCode::MotionSinkStale => {
            "MOTION_SINK_STALE: guard client health expired"
        }
        NavigationReadinessCode::MotionSinkNotReady => {
            "MOTION_SINK_NOT_READY: private UDS execution chain unavailable"
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
    /// Guard generation observed by the request-level admission check.
    pub guard_generation: u64,
    /// Guard boot nonce paired with `guard_generation`.
    pub guard_boot_nonce: [u8; 16],
    /// Ordinary global-stop generation observed by the request-level check.
    pub motion_stop_generation: u64,
}

impl MotionAdmissionProof {
    /// Parse and validate a proof received over a variable-length wire format.
    pub fn try_from_wire(
        guard_generation: u64,
        guard_boot_nonce: &[u8],
        motion_stop_generation: u64,
    ) -> Result<Self, String> {
        let guard_boot_nonce: [u8; 16] = guard_boot_nonce
            .try_into()
            .map_err(|_| "motion_guard_boot_nonce must contain exactly 16 bytes".to_string())?;
        let proof = Self {
            guard_generation,
            guard_boot_nonce,
            motion_stop_generation,
        };
        proof.validate().map_err(str::to_string)?;
        Ok(proof)
    }

    /// Validate that the proof can be transported losslessly in `GoalEnvelope`.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.guard_generation == 0 || self.guard_generation > i64::MAX as u64 {
            return Err("motion_guard_generation must be in [1, i64::MAX]");
        }
        if self.guard_boot_nonce.iter().all(|byte| *byte == 0) {
            return Err("motion_guard_boot_nonce must not be all zero");
        }
        if self.motion_stop_generation > i64::MAX as u64 {
            return Err("motion_stop_generation must not exceed i64::MAX");
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) const fn test_motion_admission() -> MotionAdmissionProof {
    MotionAdmissionProof {
        guard_generation: 7,
        guard_boot_nonce: [7; 16],
        motion_stop_generation: 1,
    }
}

/// `set_goal` 的输入(已从 gRPC `NavigationGoal` 提取并归一化)。
#[derive(Clone)]
pub struct SetGoalInput {
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

/// Machine-readable reason that allows a navigation admission to be retried.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetGoalRetryReason {
    /// A typed navigation readiness blocker rejected admission.
    NavigationNotReady(NavigationReadinessCode),
}

/// `set_goal` 的输出:成功时返回 `NavigationState(ACCEPTED)`,
/// 失败时返回带 reason 的 `Failed` state。
pub struct SetGoalOutcome {
    /// 新的 NavState(status + message + goal_id + distance_remaining)。
    pub state: NavState,
    /// Sealed attempted/acquired lease for lossless response projection.
    pub lease: Option<GoalLease>,
    /// Retryable admission cause; `None` means accepted or permanently rejected.
    pub retry_reason: Option<SetGoalRetryReason>,
}

/// Complete CAS identity required to cancel one goal lease.
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

/// Submit a goal with an optional exploration boundary while preserving the
/// public `set_goal` ABI for ordinary navigation callers.
pub async fn set_goal_with_boundary(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    mut input: SetGoalInput,
    boundary: Option<GoalBoundaryConstraint>,
) -> SetGoalOutcome {
    input.goal_id = input.goal_id.trim().to_string();
    input.parent_request_id = input.parent_request_id.trim().to_string();
    input.target.frame_id = input.frame_id.clone();
    if input.target.timestamp_ns == 0 {
        input.target.timestamp_ns = now_ns();
    }
    let normalized_goal_frame = input.frame_id.trim().trim_start_matches('/');
    let boundary_invalid = boundary.as_ref().is_some_and(|boundary| {
        input.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
            || boundary.frame_id.trim().trim_start_matches('/') != normalized_goal_frame
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
        };
    }
    if let Err(reason) = input.motion_admission.validate() {
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!("SET_GOAL_REJECTED[MOTION_ADMISSION_PROOF_INVALID]: {reason}"),
                goal_id: input.goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: None,
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
        };
    }
    // 1. readiness 校验。
    let detail = compute_readiness(handle, state).await;
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
                distance_remaining: distance_to_target(handle, &input.target).await,
                timestamp_ns: now_ns(),
            },
            lease: None,
            retry_reason: Some(SetGoalRetryReason::NavigationNotReady(detail.reason_code)),
        };
    }
    if input.frame_id != detail.map_frame {
        let rejection = NavigationReadinessCode::GoalFrameMismatch;
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
        };
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
            };
        }
    };
    let (boundary_frame_id, boundary_points) = boundary
        .map(|boundary| {
            (
                boundary.frame_id.trim().trim_start_matches('/').to_string(),
                boundary.points,
            )
        })
        .unwrap_or_default();
    let mut envelope = schema::GoalEnvelope {
        schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
        session_id: input.goal_id.clone(),
        owner_kind: input.owner_kind,
        parent_request_id: input.parent_request_id.clone(),
        goal_generation: goal_generation as i64,
        producer_runtime_generation: producer_runtime_generation as i64,
        source_timestamp_ns: input.target.timestamp_ns as i64,
        motion_guard_generation: input.motion_admission.guard_generation as i64,
        motion_guard_boot_nonce: input.motion_admission.guard_boot_nonce,
        motion_stop_generation: input.motion_admission.motion_stop_generation as i64,
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
    if let Err(error) = schema::seal_goal_envelope(&mut envelope) {
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
        };
    }
    let lease = GoalLease {
        session_id: envelope.session_id.clone(),
        owner_kind: envelope.owner_kind,
        parent_request_id: envelope.parent_request_id.clone(),
        goal_generation,
        producer_runtime_generation,
        goal_payload_sha256: envelope.payload_sha256,
        path_revision: 0,
        path_payload_sha256: [0; 32],
    };

    let _goal_command_guard = state.goal_command_gate.write().await;
    let distance_remaining = distance_to_target(handle, &input.target).await;
    {
        let mut slot = state.inner.write().await;
        if let Some(active) = slot.goal.as_ref() {
            if active.lease.identity_matches(&lease)
                && active.lease.goal_payload_sha256 == lease.goal_payload_sha256
            {
                return SetGoalOutcome {
                    state: NavState {
                        status: slot.phase.as_str().to_string(),
                        message: "GOAL_LEASE_IDEMPOTENT: existing lease retained".to_string(),
                        goal_id: lease.session_id.clone(),
                        distance_remaining,
                        timestamp_ns: now_ns(),
                    },
                    lease: Some(active.lease.clone()),
                    retry_reason: None,
                };
            }
            return SetGoalOutcome {
                state: NavState {
                    status: NavPhase::Failed.as_str().to_string(),
                    message: format!(
                        "SET_GOAL_REJECTED[GOAL_LEASE_BUSY]: active={}/{}/{}/{}",
                        active.lease.owner_kind,
                        active.lease.parent_request_id,
                        active.lease.session_id,
                        active.lease.goal_generation,
                    ),
                    goal_id: lease.session_id.clone(),
                    distance_remaining,
                    timestamp_ns: now_ns(),
                },
                lease: Some(lease),
                retry_reason: None,
            };
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
                retry_reason: None,
            };
        }
        let Ok(mut cancel) = state.cancel.lock() else {
            return SetGoalOutcome {
                state: NavState {
                    status: NavPhase::Failed.as_str().to_string(),
                    message: "SET_GOAL_REJECTED[CANCEL_STATE_UNAVAILABLE]".to_string(),
                    goal_id: lease.session_id.clone(),
                    distance_remaining,
                    timestamp_ns: now_ns(),
                },
                lease: Some(lease),
                retry_reason: None,
            };
        };
        slot.last_goal_generation = goal_generation;
        slot.phase = NavPhase::Accepted;
        slot.message = format!(
            "导航目标已提交到真实 stack,等待 /nav/state (profile={})",
            slot.profile
        );
        slot.goal = Some(ActiveGoal {
            id: input.goal_id.clone(),
            frame_id: input.frame_id.clone(),
            target: input.target.clone(),
            accepted_at_ns: now_ns(),
            lease: lease.clone(),
        });
        *cancel = CancelTracker::default();
    }

    // 2. publish sealed GoalEnvelope. Facade does not create a follower/cmd_vel.
    if let Err(e) = publish_goal_frame(handle, &envelope).await {
        let failure_message = format!("SetNavigationGoal 失败:publish /nav/goal 错误: {e}");
        let mut slot = state.inner.write().await;
        if slot.goal.as_ref().is_some_and(|goal| goal.lease == lease) {
            let Ok(mut cancel) = state.cancel.lock() else {
                slot.phase = NavPhase::Failed;
                slot.message = "SET_GOAL_ROLLBACK_FAILED[CANCEL_STATE_UNAVAILABLE]".to_string();
                return SetGoalOutcome {
                    state: NavState {
                        status: NavPhase::Failed.as_str().to_string(),
                        message: slot.message.clone(),
                        goal_id: input.goal_id,
                        distance_remaining,
                        timestamp_ns: now_ns(),
                    },
                    lease: Some(lease),
                    retry_reason: None,
                };
            };
            if cancel.lease.as_ref() == Some(&lease) {
                *cancel = CancelTracker::default();
            }
            slot.goal = None;
            slot.phase = NavPhase::Failed;
            slot.message = failure_message.clone();
        }
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: failure_message,
                goal_id: input.goal_id,
                distance_remaining,
                timestamp_ns: now_ns(),
            },
            lease: Some(lease),
            retry_reason: None,
        };
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
        retry_reason: None,
    }
}

/// 请求取消当前 active goal。
///
/// The LCM cancellation frame only stops planning and is never an actuator
/// command. Terminal `CANCELLED` requires a correlated guard `ZEROED` ack.
pub async fn cancel(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    input: CancelGoalInput,
) -> CancelOutcome {
    let _goal_command_guard = state.goal_command_gate.read().await;
    match state.begin_cancel(&input).await {
        CancelStart::Publish {
            lease,
            generation,
            goal_id,
        } => {
            let cancel_result = publish_cancel_frame(handle, &lease, generation).await;
            finish_cancel_request(state, lease, goal_id, generation, cancel_result).await
        }
        CancelStart::ExistingPending {
            lease,
            generation,
            goal_id,
            ..
        } => {
            let retry_result = publish_cancel_frame(handle, &lease, generation).await;
            finish_existing_cancel_retry(state, lease, goal_id, generation, retry_result).await
        }
        CancelStart::Rejected { goal_id, message } => {
            cancel_outcome_with_phase(NavPhase::Failed, goal_id, message, None)
        }
    }
}

async fn finish_existing_cancel_retry(
    state: &NavRuntimeState,
    lease: GoalLease,
    goal_id: String,
    cancel_generation: u64,
    retry_result: Result<(), String>,
) -> CancelOutcome {
    let slot = state.inner.read().await;
    let Ok(cancel) = state.cancel.lock() else {
        return cancel_outcome_with_phase(
            NavPhase::Failed,
            goal_id,
            "CANCEL_STATE_UNAVAILABLE".to_string(),
            Some(lease),
        );
    };
    let tracker_matches = cancel.generation == cancel_generation
        && cancel.goal_id == goal_id
        && cancel.lease.as_ref() == Some(&lease);
    if !tracker_matches {
        return cancel_outcome_with_phase(
            NavPhase::Failed,
            goal_id,
            "CANCEL_RETRY_SUPERSEDED".to_string(),
            Some(lease),
        );
    }

    match cancel.phase {
        CancelPhase::Pending => {
            let active_lease_matches = slot
                .goal
                .as_ref()
                .is_some_and(|active| active.lease == lease);
            if !active_lease_matches {
                return cancel_outcome_with_phase(
                    NavPhase::Failed,
                    goal_id,
                    "CANCEL_RETRY_SUPERSEDED".to_string(),
                    Some(lease),
                );
            }
            let message = match retry_result {
                Ok(()) => "CANCEL_PENDING_RETRY_PUBLISHED".to_string(),
                Err(error) => {
                    warn!(
                        "pending cancel retry publish failed; original transaction remains pending: {}",
                        error
                    );
                    format!("{}; retry publish failed: {error}", cancel.message)
                }
            };
            cancel_outcome_with_phase(NavPhase::CancelPending, goal_id, message, Some(lease))
        }
        CancelPhase::Zeroed => cancel_outcome_with_phase(
            NavPhase::Cancelled,
            goal_id,
            cancel.message.clone(),
            Some(lease),
        ),
        CancelPhase::Failed | CancelPhase::TimedOut => cancel_outcome_with_phase(
            NavPhase::Failed,
            goal_id,
            cancel.message.clone(),
            Some(lease),
        ),
        CancelPhase::Idle => cancel_outcome_with_phase(
            NavPhase::Failed,
            goal_id,
            "CANCEL_RETRY_SUPERSEDED".to_string(),
            Some(lease),
        ),
    }
}

async fn finish_cancel_request(
    state: &NavRuntimeState,
    lease: GoalLease,
    goal_id: String,
    cancel_generation: u64,
    cancel_result: Result<(), String>,
) -> CancelOutcome {
    let mut slot = state.inner.write().await;
    let Ok(mut cancel) = state.cancel.lock() else {
        return cancel_outcome_with_phase(
            NavPhase::Failed,
            goal_id,
            "CANCEL_STATE_UNAVAILABLE".to_string(),
            Some(lease),
        );
    };
    let tracker_matches = cancel.generation == cancel_generation
        && cancel.goal_id == goal_id
        && cancel.lease.as_ref() == Some(&lease);
    if slot.phase == NavPhase::Cancelled && tracker_matches && cancel.phase == CancelPhase::Zeroed {
        cancel.message = "CANCEL_ALREADY_ZEROED".to_string();
        return cancel_outcome_from_slot(
            &slot,
            goal_id,
            "CANCEL_ALREADY_ZEROED".to_string(),
            Some(lease),
        );
    }
    let active_lease_matches = slot
        .goal
        .as_ref()
        .is_some_and(|active| active.lease == lease);
    if !tracker_matches || cancel.phase != CancelPhase::Pending || !active_lease_matches {
        return cancel_outcome_with_phase(
            NavPhase::Failed,
            goal_id,
            "CANCEL_COMPLETION_SUPERSEDED".to_string(),
            Some(lease),
        );
    }
    if let Err(error) = cancel_result {
        warn!("publish /nav/cancel 失败: {}", error);
        let message = format!("CANCEL_DISPATCH_FAILED: {error}");
        cancel.phase = CancelPhase::Failed;
        cancel.message = message.clone();
        slot.phase = NavPhase::Failed;
        slot.message = message.clone();
        return cancel_outcome_from_slot(&slot, goal_id, message, Some(lease));
    }
    slot.phase = NavPhase::CancelPending;
    slot.message = "CANCEL_PENDING_ZERO_ACK: waiting for motion guard".into();
    cancel_outcome_from_slot(
        &slot,
        goal_id,
        "CANCEL_PENDING_ZERO_ACK".to_string(),
        Some(lease),
    )
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

/// 计算当前位置到 target 的欧氏距离(2D 平面),给 `distance_remaining` 用。
pub async fn distance_to_target(handle: &GeometryRuntimeHandle, target: &PoseState) -> f32 {
    let s = handle.state.read().await;
    let dx = s.pose.x - target.x;
    let dy = s.pose.y - target.y;
    ((dx * dx + dy * dy) as f32).sqrt()
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
    runtime_key: &str,
    platform_key: &str,
    default_channel: &str,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let channel = channel_registry::resolve_channel(runtime_key, platform_key, default_channel);
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
        GOAL_CHANNEL_RUNTIME,
        GOAL_CHANNEL_PLATFORM,
        GOAL_CHANNEL_DEFAULT,
        "nuwax_msgs.GoalEnvelope",
    )
}

/// Resolve the strict versioned path channel consumed by geometry runtime.
pub fn path_envelope_binding() -> Result<(LcmTopic, LcmMessageType), String> {
    let channel = channel_registry::resolve_strict_channel(
        PATH_CHANNEL_RUNTIME,
        PATH_CHANNEL_PLATFORM,
        PATH_CHANNEL_DEFAULT,
    )
    .map_err(|error| error.to_string())?;
    strict_typed_binding(&channel, "nuwax_msgs.PathEnvelope")
}

/// publish `/nav/goal` typed frame。
async fn publish_goal_frame(
    handle: &GeometryRuntimeHandle,
    envelope: &schema::GoalEnvelope,
) -> Result<(), String> {
    let (topic, msg_type) = goal_envelope_binding()?;
    let payload = typed_codec::encode_goal_envelope_payload(envelope)
        .map_err(|e| format!("GoalEnvelope LCM encode failed: {e}"))?;
    let frame: RawLcmFrame =
        typed_codec::build_typed_frame(topic, msg_type, payload, seqno_for(handle).await);
    handle
        .transport
        .publish(frame)
        .await
        .map_err(|e| format!("LcmTransport::publish 失败: {}", e))
}

/// publish `/nav/cancel` typed frame。
async fn publish_cancel_frame(
    handle: &GeometryRuntimeHandle,
    lease: &GoalLease,
    cancel_generation: u64,
) -> Result<(), String> {
    let (topic, msg_type) = resolve_strict_typed_binding(
        "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
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

    struct NavProfileEnvRestore(Option<OsString>);

    impl Drop for NavProfileEnvRestore {
        fn drop(&mut self) {
            match self.0.take() {
                Some(value) => std::env::set_var("NUWAX_NAV_PROFILE", value),
                None => std::env::remove_var("NUWAX_NAV_PROFILE"),
            }
        }
    }

    fn set_nav_profile_for_test(value: Option<&str>) -> NavProfileEnvRestore {
        let previous = std::env::var_os("NUWAX_NAV_PROFILE");
        match value {
            Some(value) => std::env::set_var("NUWAX_NAV_PROFILE", value),
            None => std::env::remove_var("NUWAX_NAV_PROFILE"),
        }
        NavProfileEnvRestore(previous)
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

    /// 构造一个测试 handle:test backend(走 real-lcm + test_lcm_url 隔离端口)+ 一个非空 pose 与 map。
    ///
    /// 注意:不能直接构造 `LcmTransport`(其字段非 pub);这里用 `build_handle_for_tests`
    /// 构造,backend 走 real-lcm,LCM publish 走隔离端口。
    /// 测试 handle 走真实 backend，默认 true。
    async fn make_handle() -> GeometryRuntimeHandle {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("build test handle");
        let mut s = handle.state.write().await;
        s.pose = PoseState {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
            frame_id: "map".into(),
            timestamp_ns: now_ns(),
        };
        s.map = MapState {
            width: 10,
            height: 10,
            resolution: 0.5,
            origin_x: -2.5,
            origin_y: -2.5,
            frame_id: "map".into(),
            revision: 1,
            timestamp_ns: now_ns(),
            ..Default::default()
        };
        drop(s);
        handle
    }

    #[derive(Clone)]
    struct BlockingPublishBackend {
        frames: Arc<Mutex<Vec<RawLcmFrame>>>,
        publish_gate: Arc<Semaphore>,
        outcomes: Arc<Mutex<VecDeque<Option<String>>>>,
    }

    impl BlockingPublishBackend {
        fn new(outcomes: impl IntoIterator<Item = Option<String>>) -> Self {
            Self {
                frames: Arc::new(Mutex::new(Vec::new())),
                publish_gate: Arc::new(Semaphore::new(0)),
                outcomes: Arc::new(Mutex::new(outcomes.into_iter().collect())),
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

        fn published_channels(&self) -> Vec<String> {
            self.frames
                .lock()
                .expect("published frames")
                .iter()
                .map(|frame| frame.topic.fqn(frame.msg_type.fqn()))
                .collect()
        }

        fn published_cancel_generations(&self) -> Vec<i64> {
            self.frames
                .lock()
                .expect("published frames")
                .iter()
                .filter(|frame| frame.msg_type.fqn() == "nuwax_msgs.CancelNavigation")
                .map(|frame| {
                    schema::decode_cancel_navigation_body(&frame.payload)
                        .expect("published CancelNavigation")
                        .cancel_generation
                })
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
                Some(error) => Err(TransportError::LcmBackend(error)),
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

    fn test_goal_input(goal_id: &str) -> SetGoalInput {
        SetGoalInput {
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
            goal_payload_sha256: [generation as u8; 32],
            path_revision: 0,
            path_payload_sha256: [0; 32],
        }
    }

    fn test_active_goal(goal_id: &str, generation: u64) -> ActiveGoal {
        test_active_goal_with_lease(test_lease(goal_id, generation))
    }

    fn test_active_goal_with_lease(lease: GoalLease) -> ActiveGoal {
        ActiveGoal {
            id: lease.session_id.clone(),
            frame_id: "map".to_string(),
            target: PoseState {
                qw: 1.0,
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                ..PoseState::default()
            },
            accepted_at_ns: now_ns(),
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

    fn cancel_ack(
        lease: &GoalLease,
        cancel_generation: u64,
        status: MotionSinkCancelAckStatus,
    ) -> MotionSinkCancelAck {
        MotionSinkCancelAck {
            cancel_generation,
            goal_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            status,
            message: "test cancellation acknowledgement".to_string(),
        }
    }

    fn sealed_path(lease: &GoalLease, path_revision: u64, x: f64) -> schema::PathEnvelope {
        let mut path = schema::PathEnvelope {
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation as i64,
            planner_runtime_generation: 1,
            path_revision: path_revision as i64,
            source_timestamp_ns: now_ns() as i64,
            goal_payload_sha256: lease.goal_payload_sha256,
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
        schema::seal_path_envelope(&mut path).expect("seal test path");
        path
    }

    #[test]
    fn resolve_profile_uses_default_when_unset() {
        let _lock = NAV_PROFILE_ENV_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let _restore = set_nav_profile_for_test(None);
        assert_eq!(resolve_profile_from_env(), PROFILE_SIMPLE_2D);
    }

    #[test]
    fn resolve_profile_uses_set_value() {
        let _lock = NAV_PROFILE_ENV_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let _restore = set_nav_profile_for_test(Some("replanning-astar"));
        assert_eq!(resolve_profile_from_env(), PROFILE_REPLANNING_ASTAR);
    }

    #[test]
    fn resolve_profile_falls_back_on_unknown() {
        let _lock = NAV_PROFILE_ENV_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let _restore = set_nav_profile_for_test(Some("hologram"));
        assert_eq!(resolve_profile_from_env(), DEFAULT_PROFILE);
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
        assert_eq!(NavPhase::CancelPending.as_str(), "PENDING");
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
            cmd_vel_sink: true,
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
        assert_eq!(d.reason_code, NavigationReadinessCode::MotionSinkUnobserved);
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
        assert_eq!(d.profile, PROFILE_SIMPLE_2D);
        // Phase 7 字段:初始 backend=RealLcm(test handle),counters 全 0
        assert_eq!(d.backend, "RealLcm");
        assert_eq!(d.tf_count, 0);
        assert_eq!(d.costmap_revision, 0);
        assert!(!d.cmd_vel_sink_ready);
        assert!(!d.motion_sink_observed);
        assert_eq!(d.active_profile, PROFILE_SIMPLE_2D);
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
    fn motion_admission_wire_validation_is_strict() {
        assert_eq!(
            MotionAdmissionProof::try_from_wire(7, &[7; 16], 1).unwrap(),
            test_motion_admission()
        );
        assert!(MotionAdmissionProof::try_from_wire(7, &[7; 15], 1)
            .unwrap_err()
            .contains("exactly 16 bytes"));
        assert!(MotionAdmissionProof::try_from_wire(0, &[7; 16], 1)
            .unwrap_err()
            .contains("motion_guard_generation"));
        assert!(MotionAdmissionProof::try_from_wire(7, &[0; 16], 1)
            .unwrap_err()
            .contains("must not be all zero"));
        assert!(MotionAdmissionProof::try_from_wire(7, &[7; 16], u64::MAX)
            .unwrap_err()
            .contains("motion_stop_generation"));
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
        // 不动 pose / map → readiness.tf=false
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
            "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
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
        assert_eq!(outcome.state.status, "PENDING");
        assert_eq!(outcome.state.goal_id, "g3");
        let cancellation = tokio::time::timeout(Duration::from_secs(1), cancellations.next())
            .await
            .expect("cancellation was not published")
            .expect("cancellation stream closed")
            .expect("cancellation transport error");
        let cancellation =
            nuwax_codec::lcm::schema::decode_cancel_navigation_body(&cancellation.payload)
                .expect("decode cancellation");
        let pending = state.cancel_snapshot();
        assert_eq!(cancellation.session_id, "g3");
        assert_eq!(cancellation.owner_kind, lease.owner_kind);
        assert_eq!(cancellation.parent_request_id, lease.parent_request_id);
        assert_eq!(cancellation.goal_generation, lease.goal_generation as i64);
        assert_eq!(
            cancellation.producer_runtime_generation,
            lease.producer_runtime_generation as i64
        );
        assert_eq!(cancellation.cancel_generation, pending.generation as i64);
        assert!(cancellation.cancel_generation > 0);
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::CancelPending);
        assert!(slot.goal.is_some());
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
        assert_eq!(d.profile, PROFILE_SIMPLE_2D);
        // p1 profile 列表必须包含 cmu-native / exploration-native / mls3d
        assert!(d
            .p1_native_profiles
            .contains(&PROFILE_CMU_NATIVE.to_string()));
        assert!(d
            .p1_native_profiles
            .contains(&PROFILE_EXPLORATION_NATIVE.to_string()));
        assert!(d.p1_native_profiles.contains(&PROFILE_MLS3D.to_string()));
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
    async fn set_goal_only_publishes_goal_and_returns_accepted() {
        use futures::StreamExt;

        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let cmd_channel = channel_registry::resolve_channel(
            "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
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
            GOAL_CHANNEL_RUNTIME,
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
        assert_eq!(goal.motion_guard_generation, 7);
        assert_eq!(goal.motion_guard_boot_nonce, [7; 16]);
        assert_eq!(goal.motion_stop_generation, 1);
        assert!(goal.goal_generation > 0);
        assert_ne!(goal.payload_sha256, [0; 32]);
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
            GOAL_CHANNEL_RUNTIME,
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

        let outcome = set_goal_with_boundary(
            &handle,
            &state,
            input,
            Some(GoalBoundaryConstraint {
                frame_id: "/map".to_string(),
                points: points.clone(),
            }),
        )
        .await;

        assert_eq!(outcome.state.status, "ACCEPTED");
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
        {
            let mut shared = handle.state.write().await;
            shared.pose.frame_id = "robot/map".to_string();
            shared.map.frame_id = "robot/map".to_string();
        }
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
            GOAL_CHANNEL_RUNTIME,
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

        let outcome = set_goal_with_boundary(
            &handle,
            &state,
            input,
            Some(GoalBoundaryConstraint {
                frame_id: "/robot/map".to_string(),
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
        {
            let mut shared = handle.state.write().await;
            shared.pose.frame_id = "robot/map".to_string();
            shared.map.frame_id = "robot/map".to_string();
        }
        let state = NavRuntimeState::from_env();
        let mut input = test_goal_input("configured-boundary-mismatch");
        input.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        input.parent_request_id = "configured-explore-parent".to_string();
        input.frame_id = "robot/map".to_string();
        input.target.frame_id = "robot/map".to_string();

        let outcome = set_goal_with_boundary(
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
    async fn goal_lease_cas_is_idempotent_and_cross_owner_requests_are_busy() {
        let handle = make_handle().await;
        let public_state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&public_state).await;
        let mut public = test_goal_input("public-1");
        public.goal_generation = 10;
        let first = set_goal(&handle, &public_state, public.clone()).await;
        assert_eq!(first.state.status, "ACCEPTED");
        let replay = set_goal(&handle, &public_state, public).await;
        assert_eq!(replay.state.status, "ACCEPTED");
        assert!(replay.state.message.contains("GOAL_LEASE_IDEMPOTENT"));
        assert_eq!(first.lease, replay.lease);

        let mut child = test_goal_input("child-1");
        child.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        child.parent_request_id = "explore-1".to_string();
        child.goal_generation = 11;
        let child_rejected = set_goal(&handle, &public_state, child).await;
        assert_eq!(child_rejected.state.status, "FAILED");
        assert!(child_rejected.state.message.contains("GOAL_LEASE_BUSY"));
        assert_eq!(
            public_state.active_goal_lease().await.unwrap().owner_kind,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
        );

        let child_state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&child_state).await;
        let mut child = test_goal_input("child-2");
        child.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        child.parent_request_id = "explore-2".to_string();
        child.goal_generation = 20;
        assert_eq!(
            set_goal(&handle, &child_state, child).await.state.status,
            "ACCEPTED"
        );
        let mut public = test_goal_input("public-2");
        public.goal_generation = 21;
        let public_rejected = set_goal(&handle, &child_state, public).await;
        assert_eq!(public_rejected.state.status, "FAILED");
        assert!(public_rejected.state.message.contains("GOAL_LEASE_BUSY"));
        assert_eq!(
            child_state.active_goal_lease().await.unwrap().owner_kind,
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn set_goal_publishes_slot_and_resets_cancel_in_one_critical_section() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let lease_a = test_lease("set-goal-window-a", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease_a.clone()));
        }
        assert!(matches!(
            state.begin_cancel(&cancel_input(&lease_a)).await,
            CancelStart::Publish { .. }
        ));
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Cancelled;
            slot.goal = None;
        }

        let cancel_mutex = state.cancel.clone();
        let (locked_tx, locked_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let blocker = std::thread::spawn(move || {
            let _guard = cancel_mutex.lock().expect("cancel tracker");
            locked_tx.send(()).expect("report tracker lock");
            release_rx.recv().expect("release tracker lock");
        });
        locked_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("tracker blocker started");
        let task_handle = handle.clone();
        let task_state = state.clone();
        let (started_tx, started_rx) = tokio::sync::oneshot::channel();
        let set_b = tokio::spawn(async move {
            let _ = started_tx.send(());
            let mut input = test_goal_input("set-goal-window-b");
            input.goal_generation = 2;
            set_goal(&task_handle, &task_state, input).await
        });
        started_rx.await.expect("set-goal B task started");

        let leaked_b = tokio::time::timeout(Duration::from_millis(200), async {
            loop {
                if let Ok(slot) = state.inner.try_read() {
                    if slot
                        .goal
                        .as_ref()
                        .is_some_and(|goal| goal.id == "set-goal-window-b")
                    {
                        return;
                    }
                }
                tokio::task::yield_now().await;
            }
        })
        .await;
        assert!(
            leaked_b.is_err(),
            "B must not become visible before its cancel-tracker reset commits"
        );

        release_tx.send(()).expect("release tracker blocker");
        blocker.join().expect("tracker blocker thread");
        let accepted = tokio::time::timeout(Duration::from_secs(1), set_b)
            .await
            .expect("set-goal B must finish after tracker release")
            .expect("set-goal B task");
        assert_eq!(accepted.state.status, "ACCEPTED");
        let lease_b = accepted.lease.expect("B lease");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease_b));

        let cancel_b = cancel(&handle, &state, cancel_input(&lease_b)).await;
        assert_eq!(cancel_b.state.status, "PENDING");
        assert_eq!(cancel_b.lease.as_ref(), Some(&lease_b));
    }

    #[tokio::test]
    async fn cancel_waits_until_goal_publish_commits_and_preserves_frame_order() {
        let (handle, backend) = make_blocking_handle([None, None]).await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let set_handle = handle.clone();
        let set_state = state.clone();
        let set_goal_task = tokio::spawn(async move {
            let mut input = test_goal_input("goal-publish-order");
            input.goal_generation = 1;
            set_goal(&set_handle, &set_state, input).await
        });
        backend.wait_for_publish_count(1).await;
        let lease = state
            .active_goal_lease()
            .await
            .expect("prepared goal lease is available for the exact cancel request");

        let cancel_handle = handle.clone();
        let cancel_state = state.clone();
        let cancel_lease = lease.clone();
        let cancel_task = tokio::spawn(async move {
            cancel(&cancel_handle, &cancel_state, cancel_input(&cancel_lease)).await
        });
        assert!(
            tokio::time::timeout(
                Duration::from_millis(100),
                backend.wait_for_publish_count(2),
            )
            .await
            .is_err(),
            "a cancel frame must not overtake an in-flight goal frame"
        );

        backend.release_one();
        let accepted = tokio::time::timeout(Duration::from_secs(1), set_goal_task)
            .await
            .expect("goal publish must finish")
            .expect("set-goal task");
        assert_eq!(accepted.state.status, "ACCEPTED");
        backend.wait_for_publish_count(2).await;
        assert_eq!(
            backend.published_message_types(),
            vec![
                "nuwax_msgs.GoalEnvelope".to_string(),
                "nuwax_msgs.CancelNavigation".to_string(),
            ]
        );
        assert_eq!(
            backend.published_channels(),
            vec![
                "/nav/goal#nuwax_msgs.GoalEnvelope".to_string(),
                "/nav/cancel#nuwax_msgs.CancelNavigation".to_string(),
            ]
        );
        backend.release_one();
        let pending = tokio::time::timeout(Duration::from_secs(1), cancel_task)
            .await
            .expect("cancel publish must finish")
            .expect("cancel task");
        assert_eq!(pending.state.status, "PENDING");
        assert_eq!(pending.lease.as_ref(), Some(&lease));
        let cancel_snapshot = state.cancel_snapshot();
        assert_eq!(cancel_snapshot.phase, CancelPhase::Pending);
        assert_eq!(cancel_snapshot.generation, 1);
        assert_eq!(
            state.next_cancel_generation.load(AtomicOrdering::Acquire),
            cancel_snapshot.generation
        );
        assert_eq!(backend.publish_count(), 2);
    }

    #[tokio::test]
    async fn failed_goal_publish_rolls_back_before_exact_cancel_can_start() {
        let (handle, backend) =
            make_blocking_handle([Some("goal publish failed".to_string())]).await;
        let state = NavRuntimeState::from_env();
        observe_ready_execution_chain(&state).await;
        let set_handle = handle.clone();
        let set_state = state.clone();
        let set_goal_task = tokio::spawn(async move {
            let mut input = test_goal_input("goal-publish-failure");
            input.goal_generation = 1;
            set_goal(&set_handle, &set_state, input).await
        });
        backend.wait_for_publish_count(1).await;
        let lease = state
            .active_goal_lease()
            .await
            .expect("prepared goal lease");

        let cancel_handle = handle.clone();
        let cancel_state = state.clone();
        let cancel_lease = lease.clone();
        let cancel_task = tokio::spawn(async move {
            cancel(&cancel_handle, &cancel_state, cancel_input(&cancel_lease)).await
        });
        assert!(
            tokio::time::timeout(
                Duration::from_millis(100),
                backend.wait_for_publish_count(2),
            )
            .await
            .is_err(),
            "a cancel frame must not be published for an uncommitted goal"
        );

        backend.release_one();
        let failed = tokio::time::timeout(Duration::from_secs(1), set_goal_task)
            .await
            .expect("failed goal publish must return")
            .expect("set-goal task");
        let rejected_cancel = tokio::time::timeout(Duration::from_secs(1), cancel_task)
            .await
            .expect("cancel must reject after goal rollback")
            .expect("cancel task");
        assert_eq!(failed.state.status, "FAILED");
        assert_eq!(rejected_cancel.state.status, "FAILED");
        assert!(rejected_cancel.state.message.contains("GOAL_LEASE_MISSING"));
        assert_eq!(backend.publish_count(), 1);
        assert_eq!(
            backend.published_message_types(),
            vec!["nuwax_msgs.GoalEnvelope".to_string()]
        );
        assert!(state.active_goal_lease().await.is_none());
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);
        assert_eq!(
            state.next_cancel_generation.load(AtomicOrdering::Acquire),
            0
        );
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
        assert_eq!(accepted.path_revision, 1);
        assert_eq!(accepted.path_payload_sha256, first.path_payload_sha256);
        assert_eq!(
            state.observe_path_envelope(&first).await.unwrap(),
            accepted,
            "same revision/hash is idempotent"
        );

        let conflicting = sealed_path(&lease, 1, 2.0);
        assert!(state
            .observe_path_envelope(&conflicting)
            .await
            .unwrap_err()
            .contains("conflicting path hash"));
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
        wrong_goal.goal_payload_sha256 = [9; 32];
        schema::seal_path_envelope(&mut wrong_goal).expect("reseal wrong goal identity");
        assert!(state
            .observe_path_envelope(&wrong_goal)
            .await
            .unwrap_err()
            .contains("identity/goal hash"));
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
                lease.goal_payload_sha256,
                lease.path_payload_sha256,
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
                candidate.goal_payload_sha256,
                candidate.path_payload_sha256,
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
                    lease.goal_payload_sha256,
                    [0; 32],
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
                lease.goal_payload_sha256,
                second_path.path_payload_sha256,
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
                first_candidate.goal_payload_sha256,
                first_candidate.path_payload_sha256,
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
                candidate.goal_payload_sha256,
                candidate.path_payload_sha256,
            )
            .await
            .expect("scheduler acknowledgement promotes an observed candidate")
            .expect("active lease");
        assert_eq!(promoted, candidate);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&candidate));
    }

    #[tokio::test]
    async fn cancel_captures_confirmed_path_and_freezes_candidate_promotion() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("cancel-confirmed-path", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        let candidate = state
            .observe_path_envelope(&sealed_path(&lease, 1, 1.0))
            .await
            .unwrap();
        state
            .observe_path_envelope(&sealed_path(&lease, 2, 2.0))
            .await
            .expect("newer planner path candidate");

        let CancelStart::Publish {
            lease: cancel_lease,
            ..
        } = state.begin_cancel(&cancel_input(&lease)).await
        else {
            panic!("cancel must capture the confirmed lease");
        };
        assert_eq!(cancel_lease, lease);
        assert_eq!(state.cancel_snapshot().lease.as_ref(), Some(&lease));

        let rejected = state
            .correlate_navigation_state(
                &lease.session_id,
                lease.owner_kind,
                &lease.parent_request_id,
                lease.goal_generation,
                2,
                lease.goal_payload_sha256,
                sealed_path(&lease, 2, 2.0).path_payload_sha256,
            )
            .await
            .expect_err("pending cancel freezes the confirmed path lease");
        assert!(rejected.contains("path_revision"));
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));
        assert_eq!(candidate.path_revision, 1);
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
        assert_eq!(state.cancel_snapshot().generation, 0);
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
        lease_b.goal_payload_sha256 = [42; 32];
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
        assert_eq!(state.cancel_snapshot().generation, 0);

        let exact_handle = handle.clone();
        let exact_state = state.clone();
        let exact_lease = lease_b.clone();
        let exact = tokio::spawn(async move {
            cancel(&exact_handle, &exact_state, cancel_input(&exact_lease)).await
        });
        backend.wait_for_publish_count(1).await;
        let pending = state.cancel_snapshot();
        assert_eq!(pending.lease.as_ref(), Some(&lease_b));
        let zeroed = state
            .observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: lease_b.session_id.clone(),
                owner_kind: lease_b.owner_kind,
                parent_request_id: lease_b.parent_request_id.clone(),
                goal_generation: lease_b.goal_generation,
                status: MotionSinkCancelAckStatus::Zeroed,
                message: "epoch 42 zeroed".into(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        backend.release_one();
        assert_eq!(
            exact.await.expect("exact epoch cancel").state.status,
            "CANCELLED"
        );
        assert_eq!(backend.publish_count(), 1);
    }

    #[tokio::test]
    async fn cancel_first_freezes_path_lease_and_keeps_retry_and_zero_correlated() {
        let (handle, backend) = make_blocking_handle([None, None]).await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("cancel-first", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        let first_handle = handle.clone();
        let first_state = state.clone();
        let first_lease = lease.clone();
        let first = tokio::spawn(async move {
            cancel(&first_handle, &first_state, cancel_input(&first_lease)).await
        });
        backend.wait_for_publish_count(1).await;
        let pending = state.cancel_snapshot();

        let rejected_path = state
            .observe_path_envelope(&sealed_path(&lease, 1, 1.0))
            .await
            .expect_err("a pending cancellation must freeze the active path lease");
        assert!(rejected_path.contains("CANCEL_PENDING"));
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));

        let retry_handle = handle.clone();
        let retry_state = state.clone();
        let retry_lease = lease.clone();
        let retry = tokio::spawn(async move {
            cancel(&retry_handle, &retry_state, cancel_input(&retry_lease)).await
        });
        backend.wait_for_publish_count(2).await;
        assert_eq!(state.cancel_snapshot().generation, pending.generation);
        assert_eq!(
            backend.published_cancel_generations(),
            vec![pending.generation as i64, pending.generation as i64]
        );

        let correlated = state
            .correlate_navigation_state(
                &lease.session_id,
                lease.owner_kind,
                &lease.parent_request_id,
                lease.goal_generation,
                lease.path_revision,
                lease.goal_payload_sha256,
                lease.path_payload_sha256,
            )
            .await
            .expect("the retained pre-cancel path lease must correlate")
            .expect("active lease");
        assert_eq!(correlated, lease);
        let zeroed = state
            .observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: MotionSinkCancelAckStatus::Zeroed,
                message: "cancel-first zeroed".into(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        backend.release_one();
        backend.release_one();
        assert_eq!(first.await.expect("first cancel").state.status, "CANCELLED");
        let retry_outcome = retry.await.expect("retry cancel");
        assert_eq!(retry_outcome.state.status, "CANCELLED");
        assert_eq!(retry_outcome.lease.as_ref(), Some(&lease));
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
                lease.goal_payload_sha256,
                lease.path_payload_sha256,
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
                lease.goal_payload_sha256,
                lease.path_payload_sha256,
            )
            .await
            .expect_err("wrong owner must not match an owned navigation state");
        assert_eq!(
            error,
            "NAV_STATE_LEASE_REJECTED: mismatched fields: owner_kind"
        );
    }

    #[tokio::test]
    async fn path_first_cancel_captures_confirmed_pre_path_lease_and_exact_retry() {
        let (handle, backend) = make_blocking_handle([None, None]).await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("path-first", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        let candidate = state
            .observe_path_envelope(&sealed_path(&lease, 1, 2.0))
            .await
            .expect("path is retained before cancel");
        assert_eq!(candidate.path_revision, 1);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));

        let first_handle = handle.clone();
        let first_state = state.clone();
        let first_lease = lease.clone();
        let first = tokio::spawn(async move {
            cancel(&first_handle, &first_state, cancel_input(&first_lease)).await
        });
        backend.wait_for_publish_count(1).await;
        let pending = state.cancel_snapshot();
        assert_eq!(pending.lease.as_ref(), Some(&lease));

        let retry_handle = handle.clone();
        let retry_state = state.clone();
        let retry_lease = lease.clone();
        let retry = tokio::spawn(async move {
            cancel(&retry_handle, &retry_state, cancel_input(&retry_lease)).await
        });
        backend.wait_for_publish_count(2).await;
        assert_eq!(state.cancel_snapshot().generation, pending.generation);
        assert_eq!(
            backend.published_cancel_generations(),
            vec![pending.generation as i64, pending.generation as i64]
        );

        let correlated = state
            .correlate_navigation_state(
                &lease.session_id,
                lease.owner_kind,
                &lease.parent_request_id,
                lease.goal_generation,
                lease.path_revision,
                lease.goal_payload_sha256,
                lease.path_payload_sha256,
            )
            .await
            .expect("confirmed pre-path lease correlation")
            .expect("active lease");
        assert_eq!(correlated, lease);
        let zeroed = state
            .observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: MotionSinkCancelAckStatus::Zeroed,
                message: "path-first retry zeroed".into(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        backend.release_one();
        backend.release_one();
        assert_eq!(first.await.expect("first cancel").state.status, "CANCELLED");
        let retry_outcome = retry.await.expect("retry cancel");
        assert_eq!(retry_outcome.state.status, "CANCELLED");
        assert_eq!(retry_outcome.lease.as_ref(), Some(&lease));
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
                candidate_a.goal_payload_sha256,
                candidate_a.path_payload_sha256,
            )
            .await
            .expect_err("stale candidate A must not promote replacement B");
        assert!(error.contains("session_id"));
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease_b));
    }

    #[tokio::test]
    async fn queued_path_retry_and_zero_ack_commit_one_cancel_generation_atomically() {
        let (handle, backend) = make_blocking_handle([None]).await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("queued-zero", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }

        let first_handle = handle.clone();
        let first_state = state.clone();
        let first_lease = lease.clone();
        let first = tokio::spawn(async move {
            cancel(&first_handle, &first_state, cancel_input(&first_lease)).await
        });
        backend.wait_for_publish_count(1).await;
        let pending = state.cancel_snapshot();
        assert_eq!(pending.phase, CancelPhase::Pending);

        let slot_guard = state.inner.write().await;
        let queued_path_frame = sealed_path(&lease, 1, 1.0);
        let mut queued_path = Box::pin(state.observe_path_envelope(&queued_path_frame));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_path.as_mut())
                .await
                .is_err(),
            "path must be queued behind the held slot"
        );

        let mut queued_retry = Box::pin(cancel(&handle, &state, cancel_input(&lease)));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_retry.as_mut())
                .await
                .is_err(),
            "the exact retry must queue behind the path"
        );
        let mut second_queued_retry = Box::pin(cancel(&handle, &state, cancel_input(&lease)));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), second_queued_retry.as_mut())
                .await
                .is_err(),
            "the second exact retry must queue behind the first retry"
        );

        let mut queued_ack = Box::pin(state.observe_cancel_ack(MotionSinkCancelAck {
            cancel_generation: pending.generation,
            goal_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            status: MotionSinkCancelAckStatus::Zeroed,
            message: "queued ZEROED".into(),
        }));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_ack.as_mut())
                .await
                .is_err(),
            "the acknowledgement must queue behind the retry"
        );
        assert_eq!(
            state.cancel_snapshot().phase,
            CancelPhase::Pending,
            "the tracker must remain frozen until slot and tracker can commit together"
        );

        drop(slot_guard);
        for _ in 0..3 {
            backend.release_one();
        }
        let ((path_result, retry, second_retry, ack), first_outcome) =
            tokio::time::timeout(Duration::from_secs(1), async {
                let queued =
                    tokio::join!(queued_path, queued_retry, second_queued_retry, queued_ack);
                let first_outcome = first.await.expect("first cancel task");
                (queued, first_outcome)
            })
            .await
            .expect("queued cancel transaction must complete without deadlock");
        assert!(path_result
            .expect_err("the queued path must observe the pending freeze")
            .contains("CANCEL_PENDING"));
        assert_eq!(retry.state.status, "CANCELLED");
        assert_eq!(retry.lease.as_ref(), Some(&lease));
        assert_eq!(second_retry.state.status, "CANCELLED");
        assert_eq!(second_retry.lease.as_ref(), Some(&lease));
        assert_eq!(state.cancel_snapshot().generation, pending.generation);
        assert_eq!(
            backend.published_cancel_generations(),
            vec![pending.generation as i64; 3]
        );

        assert_eq!(ack.state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
        let terminal = state.cancel_snapshot();
        assert_eq!(terminal.generation, pending.generation);
        assert_eq!(terminal.phase, CancelPhase::Zeroed);

        assert_eq!(first_outcome.state.status, "CANCELLED");
        assert_eq!(backend.publish_count(), 3);
    }

    #[tokio::test]
    async fn queued_path_remains_frozen_until_failed_ack_commits_slot_and_tracker() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("queued-failed", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        assert_eq!(
            cancel(&handle, &state, cancel_input(&lease))
                .await
                .state
                .status,
            "PENDING"
        );
        let pending = state.cancel_snapshot();

        let slot_guard = state.inner.write().await;
        let queued_path_frame = sealed_path(&lease, 1, 2.0);
        let mut queued_path = Box::pin(state.observe_path_envelope(&queued_path_frame));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_path.as_mut())
                .await
                .is_err()
        );
        let mut queued_ack = Box::pin(state.observe_cancel_ack(MotionSinkCancelAck {
            cancel_generation: pending.generation,
            goal_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            status: MotionSinkCancelAckStatus::Failed,
            message: "queued failure".into(),
        }));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_ack.as_mut())
                .await
                .is_err()
        );
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Pending);

        drop(slot_guard);
        let (path_result, failed) = tokio::time::timeout(Duration::from_secs(1), async {
            tokio::join!(queued_path, queued_ack)
        })
        .await
        .expect("queued failed acknowledgement must complete without deadlock");
        assert!(path_result
            .expect_err("the queued path must remain frozen")
            .contains("CANCEL_PENDING"));
        assert_eq!(failed.state.status, "FAILED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Failed);
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::Failed);
        assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease));
    }

    #[tokio::test]
    async fn queued_path_remains_frozen_until_timeout_commits_slot_and_tracker() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("queued-timeout", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        assert_eq!(
            cancel(&handle, &state, cancel_input(&lease))
                .await
                .state
                .status,
            "PENDING"
        );
        state.expire_cancel_for_test();

        let slot_guard = state.inner.write().await;
        let queued_path_frame = sealed_path(&lease, 1, 3.0);
        let mut queued_path = Box::pin(state.observe_path_envelope(&queued_path_frame));
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_path.as_mut())
                .await
                .is_err()
        );
        let mut queued_timeout = Box::pin(state.poll_cancel_timeout());
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_timeout.as_mut())
                .await
                .is_err()
        );
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Pending);

        drop(slot_guard);
        let (path_result, timed_out) = tokio::time::timeout(Duration::from_secs(1), async {
            tokio::join!(queued_path, queued_timeout)
        })
        .await
        .expect("queued timeout transaction must complete without deadlock");
        assert!(path_result
            .expect_err("the queued path must remain frozen")
            .contains("CANCEL_PENDING"));
        let timed_out = timed_out.expect("expired cancel must time out");
        assert_eq!(timed_out.state.status, "FAILED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::TimedOut);
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::Failed);
        assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease));
    }

    #[tokio::test]
    async fn exact_zero_recovers_failed_and_timed_out_cancel_without_weakening_identity() {
        for recover_from_failure in [true, false] {
            let handle = make_handle().await;
            let state = NavRuntimeState::from_env();
            let lease = test_lease(
                if recover_from_failure {
                    "late-zero-after-failure"
                } else {
                    "late-zero-after-timeout"
                },
                1,
            );
            {
                let mut slot = state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal_with_lease(lease.clone()));
            }
            assert_eq!(
                cancel(&handle, &state, cancel_input(&lease))
                    .await
                    .state
                    .status,
                "PENDING"
            );
            let pending = state.cancel_snapshot();
            if recover_from_failure {
                let failed = state
                    .observe_cancel_ack(cancel_ack(
                        &lease,
                        pending.generation,
                        MotionSinkCancelAckStatus::Failed,
                    ))
                    .await;
                assert_eq!(failed.state.status, "FAILED");
                assert_eq!(state.cancel_snapshot().phase, CancelPhase::Failed);
            } else {
                state.expire_cancel_for_test();
                assert!(state.poll_cancel_timeout().await.is_some());
                assert_eq!(state.cancel_snapshot().phase, CancelPhase::TimedOut);
            }

            let mut wrong_generation = cancel_ack(
                &lease,
                pending.generation.saturating_add(1),
                MotionSinkCancelAckStatus::Zeroed,
            );
            wrong_generation.message = "wrong generation".to_string();
            assert_ne!(
                state
                    .observe_cancel_ack(wrong_generation)
                    .await
                    .state
                    .status,
                "CANCELLED"
            );
            let retained_phase = state.cancel_snapshot().phase;

            let mut wrong_lease = cancel_ack(
                &lease,
                pending.generation,
                MotionSinkCancelAckStatus::Zeroed,
            );
            wrong_lease.owner_kind += 1;
            wrong_lease.message = "wrong lease".to_string();
            assert_ne!(
                state.observe_cancel_ack(wrong_lease).await.state.status,
                "CANCELLED"
            );
            assert_eq!(state.cancel_snapshot().phase, retained_phase);

            let zeroed = state
                .observe_cancel_ack(cancel_ack(
                    &lease,
                    pending.generation,
                    MotionSinkCancelAckStatus::Zeroed,
                ))
                .await;
            assert_eq!(zeroed.state.status, "CANCELLED");
            assert_eq!(state.cancel_snapshot().phase, CancelPhase::Zeroed);
            assert!(state.active_goal_lease().await.is_none());
        }
    }

    #[tokio::test]
    async fn failed_ack_remains_rejected_after_cancel_timeout() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("failed-after-timeout", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        cancel(&handle, &state, cancel_input(&lease)).await;
        let pending = state.cancel_snapshot();
        state.expire_cancel_for_test();
        assert!(state.poll_cancel_timeout().await.is_some());

        let rejected = state
            .observe_cancel_ack(cancel_ack(
                &lease,
                pending.generation,
                MotionSinkCancelAckStatus::Failed,
            ))
            .await;

        assert_ne!(rejected.state.status, "CANCELLED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::TimedOut);
        assert_eq!(state.active_goal_lease().await.as_ref(), Some(&lease));
    }

    #[tokio::test]
    async fn timeout_and_exact_zero_race_always_finishes_zeroed() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("timeout-zero-race", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        cancel(&handle, &state, cancel_input(&lease)).await;
        let pending = state.cancel_snapshot();
        state.expire_cancel_for_test();

        let (timeout_result, zeroed) = tokio::join!(
            state.poll_cancel_timeout(),
            state.observe_cancel_ack(cancel_ack(
                &lease,
                pending.generation,
                MotionSinkCancelAckStatus::Zeroed,
            )),
        );

        if let Some(timeout_result) = timeout_result {
            assert_eq!(timeout_result.state.status, "FAILED");
        }
        assert_eq!(zeroed.state.status, "CANCELLED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Zeroed);
        assert!(state.active_goal_lease().await.is_none());
    }

    #[tokio::test]
    async fn retry_generation_rejects_the_old_zero_ack() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("retry-old-zero", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        cancel(&handle, &state, cancel_input(&lease)).await;
        let first = state.cancel_snapshot();
        state.expire_cancel_for_test();
        assert!(state.poll_cancel_timeout().await.is_some());

        assert_eq!(
            cancel(&handle, &state, cancel_input(&lease))
                .await
                .state
                .status,
            "PENDING"
        );
        let retry = state.cancel_snapshot();
        assert!(retry.generation > first.generation);

        let stale = state
            .observe_cancel_ack(cancel_ack(
                &lease,
                first.generation,
                MotionSinkCancelAckStatus::Zeroed,
            ))
            .await;
        assert_ne!(stale.state.status, "CANCELLED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Pending);
        assert_eq!(state.cancel_snapshot().generation, retry.generation);

        let zeroed = state
            .observe_cancel_ack(cancel_ack(
                &lease,
                retry.generation,
                MotionSinkCancelAckStatus::Zeroed,
            ))
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Zeroed);
    }

    #[tokio::test]
    async fn stale_zero_or_failed_ack_cannot_mutate_a_replacement_incarnation() {
        for status in [
            MotionSinkCancelAckStatus::Zeroed,
            MotionSinkCancelAckStatus::Failed,
        ] {
            let handle = make_handle().await;
            let state = NavRuntimeState::from_env();
            let lease = test_lease("same-business-id", 1);
            {
                let mut slot = state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal_with_lease(lease.clone()));
            }
            assert_eq!(
                cancel(&handle, &state, cancel_input(&lease))
                    .await
                    .state
                    .status,
                "PENDING"
            );
            let pending = state.cancel_snapshot();

            let mut replacement = lease.clone();
            replacement.path_revision = 1;
            replacement.path_payload_sha256 = [9; 32];
            {
                let mut slot = state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal_with_lease(replacement.clone()));
            }
            let stale = tokio::time::timeout(
                Duration::from_secs(1),
                state.observe_cancel_ack(MotionSinkCancelAck {
                    cancel_generation: pending.generation,
                    goal_id: lease.session_id.clone(),
                    owner_kind: lease.owner_kind,
                    parent_request_id: lease.parent_request_id.clone(),
                    goal_generation: lease.goal_generation,
                    status,
                    message: "stale incarnation ack".into(),
                }),
            )
            .await
            .expect("stale acknowledgement must fail closed without blocking");

            assert_ne!(stale.state.status, "CANCELLED");
            assert_eq!(state.cancel_snapshot().phase, CancelPhase::Pending);
            let slot = state.inner.read().await;
            assert_eq!(slot.phase, NavPhase::Running);
            assert_eq!(
                slot.goal.as_ref().map(|goal| &goal.lease),
                Some(&replacement)
            );
        }
    }

    #[tokio::test]
    async fn replacement_and_cancel_reset_queued_before_ack_leave_new_goal_cancelable() {
        for status in [
            MotionSinkCancelAckStatus::Zeroed,
            MotionSinkCancelAckStatus::Failed,
        ] {
            let handle = make_handle().await;
            let state = NavRuntimeState::from_env();
            let lease_a = test_lease("queued-replacement-a", 1);
            let lease_b = test_lease("queued-replacement-b", 2);
            {
                let mut slot = state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal_with_lease(lease_a.clone()));
            }
            assert_eq!(
                cancel(&handle, &state, cancel_input(&lease_a))
                    .await
                    .state
                    .status,
                "PENDING"
            );
            let pending = state.cancel_snapshot();

            let slot_guard = state.inner.write().await;
            let replacement_state = state.clone();
            let queued_lease_b = lease_b.clone();
            let mut queued_replacement = Box::pin(async move {
                let mut slot = replacement_state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal_with_lease(queued_lease_b));
                drop(slot);
                replacement_state.reset_cancel();
            });
            assert!(
                tokio::time::timeout(Duration::from_millis(20), queued_replacement.as_mut())
                    .await
                    .is_err()
            );
            let mut queued_ack = Box::pin(state.observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: lease_a.session_id.clone(),
                owner_kind: lease_a.owner_kind,
                parent_request_id: lease_a.parent_request_id.clone(),
                goal_generation: lease_a.goal_generation,
                status,
                message: "late ack after replacement".into(),
            }));
            assert!(
                tokio::time::timeout(Duration::from_millis(20), queued_ack.as_mut())
                    .await
                    .is_err()
            );

            drop(slot_guard);
            let ((), stale) = tokio::time::timeout(Duration::from_secs(1), async {
                tokio::join!(queued_replacement, queued_ack)
            })
            .await
            .expect("replacement and stale acknowledgement must not deadlock");
            assert_ne!(stale.state.status, "CANCELLED");
            assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);
            {
                let slot = state.inner.read().await;
                assert_eq!(slot.phase, NavPhase::Running);
                assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
            }

            let cancel_b = cancel(&handle, &state, cancel_input(&lease_b)).await;
            assert_eq!(cancel_b.state.status, "PENDING");
            assert_eq!(cancel_b.lease.as_ref(), Some(&lease_b));
        }
    }

    #[tokio::test]
    async fn replacement_and_cancel_reset_queued_before_timeout_leave_new_goal_cancelable() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease_a = test_lease("queued-timeout-a", 1);
        let lease_b = test_lease("queued-timeout-b", 2);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease_a.clone()));
        }
        assert_eq!(
            cancel(&handle, &state, cancel_input(&lease_a))
                .await
                .state
                .status,
            "PENDING"
        );
        state.expire_cancel_for_test();

        let slot_guard = state.inner.write().await;
        let replacement_state = state.clone();
        let queued_lease_b = lease_b.clone();
        let mut queued_replacement = Box::pin(async move {
            let mut slot = replacement_state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(queued_lease_b));
            drop(slot);
            replacement_state.reset_cancel();
        });
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_replacement.as_mut())
                .await
                .is_err()
        );
        let mut queued_timeout = Box::pin(state.poll_cancel_timeout());
        assert!(
            tokio::time::timeout(Duration::from_millis(20), queued_timeout.as_mut())
                .await
                .is_err()
        );

        drop(slot_guard);
        let ((), timeout_outcome) = tokio::time::timeout(Duration::from_secs(1), async {
            tokio::join!(queued_replacement, queued_timeout)
        })
        .await
        .expect("replacement and stale timeout must not deadlock");
        assert!(timeout_outcome.is_none());
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);
        {
            let slot = state.inner.read().await;
            assert_eq!(slot.phase, NavPhase::Running);
            assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
        }

        let cancel_b = cancel(&handle, &state, cancel_input(&lease_b)).await;
        assert_eq!(cancel_b.state.status, "PENDING");
        assert_eq!(cancel_b.lease.as_ref(), Some(&lease_b));
    }

    #[tokio::test]
    async fn cancel_generation_rejects_int64_exhaustion_without_reuse() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("cancel-generation-int64", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        state
            .next_cancel_generation
            .store(i64::MAX as u64 - 1, AtomicOrdering::Release);

        let CancelStart::Publish { generation, .. } =
            state.begin_cancel(&cancel_input(&lease)).await
        else {
            panic!("the final positive int64 generation must be reservable");
        };
        assert_eq!(generation, i64::MAX as u64);
        state.reset_cancel();
        let CancelStart::Rejected { message, .. } = state.begin_cancel(&cancel_input(&lease)).await
        else {
            panic!("generation exhaustion must fail closed");
        };
        assert!(message.contains("CANCEL_GENERATION_EXHAUSTED"));
        assert_eq!(
            state.next_cancel_generation.load(AtomicOrdering::Acquire),
            i64::MAX as u64
        );
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);
    }

    #[tokio::test]
    async fn cancel_generation_rejects_u64_max_without_wrapping_to_zero() {
        let state = NavRuntimeState::from_env();
        let lease = test_lease("cancel-generation-u64", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal_with_lease(lease.clone()));
        }
        state
            .next_cancel_generation
            .store(u64::MAX, AtomicOrdering::Release);

        let CancelStart::Rejected { message, .. } = state.begin_cancel(&cancel_input(&lease)).await
        else {
            panic!("u64::MAX must not be published or reused");
        };
        assert!(message.contains("CANCEL_GENERATION_EXHAUSTED"));
        assert_eq!(
            state.next_cancel_generation.load(AtomicOrdering::Acquire),
            u64::MAX
        );
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);
    }

    #[tokio::test]
    async fn exact_pending_cancel_retry_republishes_the_same_generation() {
        let (handle, backend) = make_blocking_handle([None, None]).await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("retry-goal", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("retry-goal", 1));
        }

        let first_handle = handle.clone();
        let first_state = state.clone();
        let first_input = cancel_input(&lease);
        let first =
            tokio::spawn(async move { cancel(&first_handle, &first_state, first_input).await });
        backend.wait_for_publish_count(1).await;
        let first_snapshot = state.cancel_snapshot();
        assert_eq!(first_snapshot.phase, CancelPhase::Pending);

        let retry_handle = handle.clone();
        let retry_state = state.clone();
        let retry_lease = lease.clone();
        let retry = tokio::spawn(async move {
            cancel(&retry_handle, &retry_state, cancel_input(&retry_lease)).await
        });
        backend.wait_for_publish_count(2).await;
        assert_eq!(
            state.cancel_snapshot().generation,
            first_snapshot.generation
        );
        assert_eq!(
            backend.published_cancel_generations(),
            vec![
                first_snapshot.generation as i64,
                first_snapshot.generation as i64
            ]
        );

        backend.release_one();
        backend.release_one();
        let first_outcome = first.await.expect("first cancel task");
        let retry_outcome = retry.await.expect("retry cancel task");
        assert_eq!(first_outcome.state.status, "PENDING");
        assert_eq!(retry_outcome.state.status, "PENDING");
        assert_eq!(
            retry_outcome.state.message,
            "CANCEL_PENDING_RETRY_PUBLISHED"
        );
        assert_eq!(retry_outcome.lease.as_ref(), Some(&lease));

        let zeroed = state
            .observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: first_snapshot.generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: MotionSinkCancelAckStatus::Zeroed,
                message: "retry generation zeroed".into(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        assert!(state.active_goal_lease().await.is_none());
        assert_eq!(
            state.cancel_snapshot().generation,
            first_snapshot.generation
        );
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Zeroed);
        assert_eq!(backend.publish_count(), 2);
    }

    #[tokio::test]
    async fn completed_cancel_retry_reports_the_current_terminal_phase() {
        for (cancel_phase, nav_phase, expected_status) in [
            (CancelPhase::Zeroed, NavPhase::Cancelled, "CANCELLED"),
            (CancelPhase::Failed, NavPhase::Failed, "FAILED"),
            (CancelPhase::TimedOut, NavPhase::Failed, "FAILED"),
        ] {
            let state = NavRuntimeState::from_env();
            let lease = test_lease("completed-retry", 1);
            let generation = 7;
            {
                let mut slot = state.inner.write().await;
                slot.phase = nav_phase;
                slot.goal = (cancel_phase != CancelPhase::Zeroed)
                    .then(|| test_active_goal_with_lease(lease.clone()));
            }
            {
                let mut cancel = state.cancel.lock().expect("cancel tracker");
                *cancel = CancelTracker {
                    generation,
                    goal_id: lease.session_id.clone(),
                    lease: Some(lease.clone()),
                    phase: cancel_phase,
                    requested_at: Some(Instant::now()),
                    stack_terminal_observed: true,
                    message: format!("terminal {cancel_phase:?}"),
                };
            }

            let outcome = finish_existing_cancel_retry(
                &state,
                lease.clone(),
                lease.session_id.clone(),
                generation,
                Ok(()),
            )
            .await;
            assert_eq!(outcome.state.status, expected_status);
            assert_eq!(outcome.state.message, format!("terminal {cancel_phase:?}"));
            assert_eq!(outcome.lease.as_ref(), Some(&lease));
        }
    }

    #[tokio::test]
    async fn pending_cancel_for_a_different_full_lease_is_not_overwritten() {
        let (handle, backend) = make_blocking_handle([None]).await;
        let state = NavRuntimeState::from_env();
        let lease_a = test_lease("lease-a", 1);
        let lease_b = test_lease("lease-b", 2);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("lease-a", 1));
        }

        let first_handle = handle.clone();
        let first_state = state.clone();
        let first = tokio::spawn(async move {
            cancel(&first_handle, &first_state, cancel_input(&lease_a)).await
        });
        backend.wait_for_publish_count(1).await;
        let pending_a = state.cancel_snapshot();

        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("lease-b", 2));
        }
        let rejected = tokio::time::timeout(
            Duration::from_millis(100),
            cancel(&handle, &state, cancel_input(&lease_b)),
        )
        .await
        .expect("a conflicting pending lease must be rejected without publishing");
        assert_eq!(rejected.state.status, "FAILED");
        assert!(rejected.state.message.contains("CANCEL_IN_FLIGHT_CONFLICT"));
        assert_eq!(state.cancel_snapshot(), pending_a);
        assert_eq!(backend.publish_count(), 1);

        backend.release_one();
        let late_a = first.await.expect("lease A cancel task");
        assert!(late_a
            .state
            .message
            .contains("CANCEL_COMPLETION_SUPERSEDED"));
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::Running);
        assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
    }

    #[tokio::test]
    async fn late_cancel_success_or_error_cannot_mutate_a_replacement_slot() {
        for first_error in [None, Some("late A publish error".to_string())] {
            let (handle, backend) = make_blocking_handle([first_error, None]).await;
            let state = NavRuntimeState::from_env();
            let lease_a = test_lease("late-a", 1);
            let lease_b = test_lease("replacement-b", 2);
            {
                let mut slot = state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal("late-a", 1));
            }

            let first_handle = handle.clone();
            let first_state = state.clone();
            let first = tokio::spawn(async move {
                cancel(&first_handle, &first_state, cancel_input(&lease_a)).await
            });
            backend.wait_for_publish_count(1).await;
            let generation_a = state.cancel_snapshot().generation;

            {
                let mut slot = state.inner.write().await;
                slot.phase = NavPhase::Running;
                slot.goal = Some(test_active_goal("replacement-b", 2));
            }
            state.reset_cancel();

            backend.release_one();
            let late_a = first.await.expect("late A cancel task");
            assert!(late_a
                .state
                .message
                .contains("CANCEL_COMPLETION_SUPERSEDED"));
            {
                let slot = state.inner.read().await;
                assert_eq!(slot.phase, NavPhase::Running);
                assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
            }
            assert_eq!(state.cancel_snapshot().phase, CancelPhase::Idle);

            let second_handle = handle.clone();
            let second_state = state.clone();
            let second_lease = lease_b.clone();
            let second = tokio::spawn(async move {
                cancel(&second_handle, &second_state, cancel_input(&second_lease)).await
            });
            backend.wait_for_publish_count(2).await;
            let generation_b = state.cancel_snapshot().generation;
            assert!(generation_b > generation_a);
            assert_eq!(state.cancel_snapshot().lease.as_ref(), Some(&lease_b));
            backend.release_one();
            let pending_b = second.await.expect("replacement B cancel task");
            assert_eq!(pending_b.state.status, "PENDING");
            assert_eq!(pending_b.lease.as_ref(), Some(&lease_b));
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
        let state = NavRuntimeState::from_env();
        let lease = test_lease("cancel-me", 1);
        {
            let mut slot = state.inner.write().await;
            slot.goal = Some(test_active_goal("cancel-me", 1));
        }

        let CancelStart::Publish {
            lease: reserved_lease,
            generation,
            goal_id,
        } = state.begin_cancel(&cancel_input(&lease)).await
        else {
            panic!("first cancellation must publish");
        };
        assert_eq!(reserved_lease, lease);
        let outcome = finish_cancel_request(
            &state,
            lease,
            goal_id,
            generation,
            Err("cancel publish failed".into()),
        )
        .await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("cancel publish failed"));
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Failed);
        assert!(state.inner.read().await.goal.is_some());
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
        assert!(!stale.ready);
        assert_eq!(stale.reason_code, NavigationReadinessCode::MotionSinkStale);

        state
            .observe_motion_sink(MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("same-generation heartbeat refreshes freshness");
        assert!(compute_readiness(&handle, &state).await.ready);
    }

    #[tokio::test]
    async fn cancel_waits_for_guard_zeroed_ack_after_stack_terminal() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let lease = test_lease("guard-zero", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("guard-zero", 1));
        }

        let pending_outcome = cancel(&handle, &state, cancel_input(&lease)).await;
        assert_eq!(pending_outcome.state.status, "PENDING");
        let pending = state.cancel_snapshot();
        assert_eq!(pending.phase, CancelPhase::Pending);
        assert!(pending.generation > 0);

        let stack_terminal = state
            .observe_stack_state(NavState {
                status: "CANCELLED".into(),
                message: "nav-stack stopped planning".into(),
                goal_id: "guard-zero".into(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            })
            .await;
        assert_eq!(stack_terminal.status, "PENDING");
        assert_eq!(state.inner.read().await.phase, NavPhase::CancelPending);
        assert!(state.inner.read().await.goal.is_some());

        let terminal = state
            .observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: "guard-zero".into(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: MotionSinkCancelAckStatus::Zeroed,
                message: "motion guard ZEROED".into(),
            })
            .await;
        assert_eq!(terminal.state.status, "CANCELLED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Zeroed);
        assert!(state.inner.read().await.goal.is_none());

        let stale_stack_failure = state
            .observe_stack_state(NavState {
                status: "FAILED".into(),
                message: "late planner terminal".into(),
                goal_id: "guard-zero".into(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            })
            .await;
        assert_eq!(
            stale_stack_failure.status, "CANCELLED",
            "guard ZEROED remains authoritative over late planner terminal frames"
        );
    }

    #[tokio::test]
    async fn cancel_failure_and_timeout_are_explicit_typed_states() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let failed_lease = test_lease("cancel-failure", 1);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("cancel-failure", 1));
        }
        let _ = cancel(&handle, &state, cancel_input(&failed_lease)).await;
        let pending = state.cancel_snapshot();
        let failed = state
            .observe_cancel_ack(MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: "cancel-failure".into(),
                owner_kind: failed_lease.owner_kind,
                parent_request_id: failed_lease.parent_request_id.clone(),
                goal_generation: failed_lease.goal_generation,
                status: MotionSinkCancelAckStatus::Failed,
                message: "guard rejected zero command".into(),
            })
            .await;
        assert_eq!(failed.state.status, "FAILED");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Failed);
        let stack_after_failure = state
            .observe_stack_state(NavState {
                status: "CANCELLED".into(),
                message: "planner stopped after guard failure".into(),
                goal_id: "cancel-failure".into(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            })
            .await;
        assert_eq!(
            stack_after_failure.status, "FAILED",
            "planner terminal must not impersonate an actuator ZEROED ack"
        );

        let timeout_lease = test_lease("cancel-timeout", 2);
        {
            let mut slot = state.inner.write().await;
            slot.phase = NavPhase::Running;
            slot.goal = Some(test_active_goal("cancel-timeout", 2));
        }
        let _ = cancel(&handle, &state, cancel_input(&timeout_lease)).await;
        let pending = state.cancel_snapshot();
        state.expire_cancel_for_test();
        let timed_out = state
            .poll_cancel_timeout()
            .await
            .expect("pending cancellation must time out");
        assert_eq!(timed_out.state.status, "FAILED");
        let snapshot = state.cancel_snapshot();
        assert_eq!(snapshot.generation, pending.generation);
        assert_eq!(snapshot.phase, CancelPhase::TimedOut);
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
            "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
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
        assert_eq!(outcome.state.status, "PENDING");
        assert_eq!(state.cancel_snapshot().phase, CancelPhase::Pending);
        assert!(
            tokio::time::timeout(Duration::from_millis(150), commands.next())
                .await
                .is_err(),
            "facade cancellation must not publish /cmd_vel; the guard UDS is the sole actuator ingress"
        );
    }
}
