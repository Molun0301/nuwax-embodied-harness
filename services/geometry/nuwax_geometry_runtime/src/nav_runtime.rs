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
//! - 不持有 LCM 订阅线程,所有数据流都通过 `GeometryRuntimeHandle` 的
//!   `state.pose` / `state.map` 读取;这样可以与 `lcm_runtime.rs` 的
//!   typed codec 路径保持一致。
//! - facade 只维护提交中的 `ACCEPTED/PENDING`；`PLANNING/RUNNING/SUCCEEDED/
//!   CANCELLED/FAILED` 均由观测到的 stack state 推进。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

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

/// `/nav/goal` typed channel 平台级 channel name。
pub const GOAL_CHANNEL_PLATFORM: &str = "NUWAX_LCM_CHANNEL_GOAL_POSE";
/// `/nav/goal` 本进程内 alias。
pub const GOAL_CHANNEL_RUNTIME: &str = "NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL";
/// `/nav/goal` 内置 default。
pub const GOAL_CHANNEL_DEFAULT: &str = "/nav/goal#geometry_msgs.PoseStamped";

/// `LcmMessageType::new("geometry_msgs/PoseStamped")` 在启动期验证一次,
/// 失败应直接 panic(因为这是 typed protocol 层,不可能 fail)。
fn goal_msg_type() -> LcmMessageType {
    LcmMessageType::new("geometry_msgs/PoseStamped").expect("typed geometry_msgs/PoseStamped")
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
    /// 用户取消(走 /nav/cancel + zero cmd_vel 路径)。
    Cancelled,
    /// cancel 已提交，等待 stack state 确认终态。
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

/// 当前 active goal 的内存表示。`id` 唯一,`frame_id` 用于 typed channel。
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
}

#[derive(Clone, Default)]
struct StackObservation {
    observed_at: Option<Instant>,
    state: Option<NavState>,
}

const DEFAULT_STACK_STATE_TTL: Duration = Duration::from_secs(2);

impl NavRuntimeState {
    /// 新建 nav runtime:读 env profile、build module graph、把节点 ID
    /// 写入 slot。
    pub fn from_env() -> Self {
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
        }
    }

    /// 读内部 slot 句柄(给 server.rs 写 diagnostics 用)。
    pub fn inner(&self) -> Arc<RwLock<NavRuntimeSlot>> {
        self.inner.clone()
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
    pub async fn observe_stack_state(&self, mut observed: NavState) -> NavState {
        let previous_state = self
            .stack_observation
            .lock()
            .ok()
            .and_then(|stack| stack.state.clone());
        {
            let mut slot = self.inner.write().await;
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
            if let Some(phase) = NavPhase::from_stack_status(&observed.status) {
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
        observed
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

    #[cfg(test)]
    fn expire_stack_observation_for_test(&self) {
        if let Ok(mut stack) = self.stack_observation.lock() {
            stack.observed_at =
                Instant::now().checked_sub(stack_state_ttl() + Duration::from_millis(1));
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

/// 细分 readiness 子项。`reason` 字段写入 JSON,Python 侧
/// `get_navigation_readiness` 解析后返回 dict(满足"costmap、tf、planner、
/// cmd_vel sink、LCM backend 子项"要求)。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigationReadinessDetail {
    /// 整体 ready。
    pub ready: bool,
    /// 顶层 reason(中文 + 关键 blocker 标签)。
    pub reason: String,
    /// LCM backend 是否可用。
    pub lcm_backend: bool,
    /// /tf 是否有有效 map→odom→base_link 链(粗略检查 `state.pose` 已被更新过)。
    pub tf: bool,
    /// /map/occupancy 是否有 occupancy grid(检查 `state.map.revision > 0`)。
    pub costmap: bool,
    /// profile 受支持，且最近收到真实 stack heartbeat/state。
    pub planner: bool,
    /// /cmd_vel channel 是否可写(transport 存在即 true)。
    pub cmd_vel_sink: bool,
    /// 当前 profile(给 driver / debug 用)。
    pub profile: String,
    /// module graph 节点 ID 数量(给 operator 看是否成功 build graph)。
    pub module_node_count: usize,
    /// 在 default profile 之外的,本阶段未启用的 native profile 列表。
    pub p1_native_profiles: Vec<String>,
    // === Phase 7 增量字段 ===
    /// canonical LCM backend 名(2026-07-08 fake-lcm 物理删除后,字段值固定为 "RealLcm")。
    pub backend: String,
    /// 累计有效 tf 帧计数(`pose.timestamp_ns > 0` 时 +1)。
    pub tf_count: u64,
    /// 当前 `state.map.revision` 快照。
    pub costmap_revision: u64,
    /// Phase 6 cmd_vel consumer bridge real-LCM consumer + watchdog
    /// 是否已就绪。real production profile + `false` → readiness.ready=false
    /// 且 `SetNavigationGoal` 被拒绝。单元测试可调
    /// `set_cmd_vel_sink_ready_override()` 注入强制分支;运行期默认按
    /// `lcm_backend_available` 估算(Lane 6 P1 跨进程 watchdog 未实装)。
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
}

/// cmd_vel sink ready override(Lane 6 unit test 注入通道)。
///
/// Phase 6 cmd_vel consumer bridge 在生产环境 real-LCM 下应该把 watchdog
/// 已就绪的真实状态写到这里。本期(Phase 7)保留 override 入口,Lane 6
/// 实装完后会被跨进程 ack 替换;单元测试 / Python diagnostics 测试
/// 用 `set_cmd_vel_sink_ready_override(Some(true/false))`。
///
/// 默认(`None`)时 `compute_readiness` 按 `lcm_backend_available` 估算。
static CMD_VEL_SINK_READY_OVERRIDE: StdMutex<Option<bool>> = StdMutex::new(None);

/// 单元测试 / Lane 6 注入接口。传 `None` 解除 override,恢复按 backend 估算。
pub fn set_cmd_vel_sink_ready_override(value: Option<bool>) {
    if let Ok(mut g) = CMD_VEL_SINK_READY_OVERRIDE.lock() {
        *g = value;
    }
}

/// 当前 cmd_vel sink ready override(给 compute_readiness 内部用)。
fn cmd_vel_sink_ready_override() -> Option<bool> {
    CMD_VEL_SINK_READY_OVERRIDE.lock().ok().and_then(|g| *g)
}

const P1_NATIVE_PROFILES: &[&str] = &[
    PROFILE_CMU_NATIVE,
    PROFILE_EXPLORATION_NATIVE,
    PROFILE_MLS3D,
];

/// Phase 6: 是否启用"production profile 强制要求 cmd_vel sink ready" 门。
///
/// 备注:Phase 7 lane 已经实现无条件 gate(`cmd_vel_sink_ready=false` → 拒绝);
/// 本函数保留作为未来通过 env 关闭 gate 的扩展点。当前不在 `compute_readiness`
/// 中调用,以避免与 lane-7 决策冲突;若需启用,需在生产 env 显式
/// `NUWAX_NAV_REQUIRE_CMD_VEL_SINK=true`,然后再让 compute_readiness 调它。
#[allow(dead_code)]
pub fn require_cmd_vel_sink_ready() -> bool {
    matches!(
        std::env::var("NUWAX_NAV_REQUIRE_CMD_VEL_SINK")
            .unwrap_or_default()
            .to_ascii_lowercase()
            .as_str(),
        "1" | "true" | "yes" | "on"
    )
}

// Phase 7:tf_count 累计计数。`pose.timestamp_ns == 0` 视为未定位初态;
// 一旦 timestamp_ns > 0 就在每次 readiness call site 累加 1,作为 readiness.
// tf_count 字段暴露给 diagnostics。Lane 6 阶段会改成订阅层 set,本阶段保留
// monotonic 计数(fork 进程不重置;单元测试可调用 reset_tf_count_for_test())。
static TF_COUNT: AtomicU64 = AtomicU64::new(0);

#[cfg(test)]
fn reset_tf_count_for_test() {
    TF_COUNT.store(0, std::sync::atomic::Ordering::Relaxed);
}

/// Phase 7:tf feed path 通知 readiness 计数器增加 1。
/// 该函数应由 lcm_runtime `/tf` typed frame 到达时被调用,
/// 而**不**在 `compute_readiness` 这种查询路径里 fetch_add —— 否则
/// parallel test 把 read+conditional-increment 当中 read-modify-write
/// 跑会丢更新。Unit test env 没有真实 LCM feed,tf_count 永久 0。
pub fn note_tf_observed() {
    TF_COUNT.fetch_add(1, AtomicOrdering::Relaxed);
}

/// 计算细分 readiness。读 LCM handle 状态 + 内部 slot。
pub async fn compute_readiness(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
) -> NavigationReadinessDetail {
    let s = handle.state.read().await;
    let tf = s.pose.timestamp_ns > 0;
    let costmap_revision = s.map.revision;
    let costmap = costmap_revision > 0;
    // Phase 7:tf_count 暴露给 diagnostics,真计数由 lcm_runtime tf feed path
    // 调用 `note_tf_observed()` 维护,这里只读 —— 避免在查询路径上
    // 做状态变更引起 parallel test race。
    let tf_count = TF_COUNT.load(AtomicOrdering::Relaxed);
    drop(s);
    let profile = state.profile().await;
    let profile_supported = matches!(profile, PROFILE_SIMPLE_2D | PROFILE_REPLANNING_ASTAR);
    let stack_health = state.stack_health_snapshot();
    let planner = profile_supported && stack_health.fresh;
    let lcm_backend = handle.lcm_backend_available;
    // Phase 7:cmd_vel_sink_ready 由 override 控制,缺省按 lcm_backend 估算。
    // Phase 6:Lane 6 注入接口(set_cmd_vel_sink_ready_override)与 handle 上
    // 的 cmd_vel_sink_ready atomic bool 都参与决策;override > handle,
    // handle 缺省时回退 lcm_backend。
    let cmd_vel_sink = lcm_backend; // backward-compatible 字段
    let cmd_vel_sink_ready =
        cmd_vel_sink_ready_override().unwrap_or_else(|| handle.cmd_vel_sink_ready());
    // Phase 7:backend canonical 名(用 transport_backend_name 反推)。
    let backend = match handle.transport_backend_name {
        "real-lcm" | "real-lcm-unavailable" => "RealLcm",
        other => {
            // 不在这里 panic;未知 backend 按当前 enum 语义映射并记 log。
            tracing::warn!("nav readiness: 未知 transport_backend_name={}", other);
            "Unknown"
        }
    };
    let ready = lcm_backend && tf && costmap && planner && cmd_vel_sink_ready;

    let reason = if !lcm_backend {
        "LCM backend 不可用,nav runtime 无法发布 typed channel"
    } else if !cmd_vel_sink_ready {
        "/cmd_vel consumer + watchdog 未就绪,nav runtime 拒绝接目标"
    } else if !stack_health.observed {
        "尚未观测到 nuwax-nav-stack /nav/state heartbeat"
    } else if !stack_health.fresh {
        "nuwax-nav-stack /nav/state heartbeat 已过期"
    } else if !tf {
        "/tf 尚无有效位姿,等待 SLAM/定位 first frame"
    } else if !costmap {
        "/map/occupancy 尚未生成,nav runtime 拒绝接目标"
    } else if !profile_supported {
        "current profile 不是 simple-2d / replanning-astar,P1 native profile 默认禁用"
    } else {
        "nuwax-nav-stack 已就绪"
    }
    .to_string();

    NavigationReadinessDetail {
        ready,
        reason,
        lcm_backend,
        tf,
        costmap,
        planner,
        cmd_vel_sink,
        profile: profile.to_string(),
        module_node_count: state.module_nodes().await.len(),
        p1_native_profiles: P1_NATIVE_PROFILES.iter().map(|s| s.to_string()).collect(),
        backend: backend.to_string(),
        tf_count,
        costmap_revision,
        cmd_vel_sink_ready,
        active_profile: profile.to_string(),
        stack_state_observed: stack_health.observed,
        stack_state_fresh: stack_health.fresh,
        stack_last_seen_age_ms: stack_health.age_ms,
        stack_last_status: stack_health.status,
    }
}

/// Phase 7:`SetNavigationGoal` readiness reject 时给出的 reason 标签。
///
/// 当 real production profile 下 `cmd_vel_sink_ready=false` 时,即便
/// tf / costmap / planner 都齐,set_goal 也立即返回 FAILED + 该 reason,
/// 让上层 / 前端能在日志 / NavState.message 里看到 `cmd_vel_sink=down`
/// 阻塞原因,而不是被泛化的"未就绪"吞掉。
pub const REJECT_REASON_CMD_VEL_SINK_DOWN: &str =
    "cmd_vel_sink 未就绪,SetNavigationGoal 被拒绝 (Phase 7 gate)";

/// `set_goal` 的输入(已从 gRPC `NavigationGoal` 提取并归一化)。
pub struct SetGoalInput {
    /// 目标点 ID(用户传或自动生成)。
    pub goal_id: String,
    /// frame_id(`map` 默认)。
    pub frame_id: String,
    /// 目标位姿(在 `frame_id` 坐标系下)。
    pub target: PoseState,
}

/// `set_goal` 的输出:成功时返回 `NavigationState(ACCEPTED)`,
/// 失败时返回带 reason 的 `Failed` state。
pub struct SetGoalOutcome {
    /// 新的 NavState(status + message + goal_id + distance_remaining)。
    pub state: NavState,
}

/// `cancel` 的输出:同 set_goal。
pub struct CancelOutcome {
    /// 新的 NavState。
    pub state: NavState,
}

/// Submit one goal to the real stack.
///
/// 失败时返回带 `status=FAILED` 的 NavState(但 `goal_id` 保留),
/// 不写入 active goal 也不 publish `/nav/goal`。
pub async fn set_goal(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    mut input: SetGoalInput,
) -> SetGoalOutcome {
    input.target.frame_id = input.frame_id.clone();
    // 1. readiness 校验。
    let detail = compute_readiness(handle, state).await;
    if !detail.ready {
        // Phase 7:real production profile 下 cmd_vel_sink_ready=false 时,
        // 单独给出 REJECT_REASON_CMD_VEL_SINK_DOWN,避免与"lcm_backend down /
        // tf 缺失 / costmap 缺失"被一起泛化为"nav_runtime 未就绪"。
        let reason_label = if !detail.cmd_vel_sink_ready {
            REJECT_REASON_CMD_VEL_SINK_DOWN
        } else {
            detail.reason.as_str()
        };
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!(
                    "SetNavigationGoal 失败:nav_runtime 未就绪 ({})",
                    reason_label
                ),
                goal_id: input.goal_id.clone(),
                distance_remaining: distance_to_target(handle, &input.target).await,
                timestamp_ns: now_ns(),
            },
        };
    }

    let distance_remaining = distance_to_target(handle, &input.target).await;
    {
        let mut slot = state.inner.write().await;
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
        });
    }

    // 2. publish /nav/goal typed frame。Facade 不创建 follower,也不发布 cmd_vel。
    if let Err(e) = publish_goal_frame(handle, &input).await {
        let mut slot = state.inner.write().await;
        if slot
            .goal
            .as_ref()
            .is_some_and(|goal| goal.id == input.goal_id)
        {
            slot.goal = None;
            slot.phase = NavPhase::Failed;
            slot.message = format!("SetNavigationGoal 失败:publish /nav/goal 错误: {e}");
        }
        return SetGoalOutcome {
            state: NavState {
                status: NavPhase::Failed.as_str().to_string(),
                message: format!("SetNavigationGoal 失败:publish /nav/goal 错误: {}", e),
                goal_id: input.goal_id.clone(),
                distance_remaining: distance_to_target(handle, &input.target).await,
                timestamp_ns: now_ns(),
            },
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
    }
}

/// 请求取消当前 active goal。
///
/// facade 的 publish 结果不是 terminal ack；无论成功或失败都保持 `PENDING`,
/// 直到 `/nav/state` 明确报告 terminal cancellation。
pub async fn cancel(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    goal_id: String,
) -> CancelOutcome {
    let goal_id = if goal_id.trim().is_empty() {
        state
            .inner
            .read()
            .await
            .goal
            .as_ref()
            .map(|goal| goal.id.clone())
            .unwrap_or_default()
    } else {
        goal_id.trim().to_string()
    };
    let cancel_result = publish_cancel_frame(handle, &goal_id).await;
    let zero_result = publish_zero_cmd_vel(handle).await;
    finish_cancel_request(state, goal_id, cancel_result, zero_result).await
}

async fn finish_cancel_request(
    state: &NavRuntimeState,
    goal_id: String,
    cancel_result: Result<(), String>,
    zero_result: Result<(), String>,
) -> CancelOutcome {
    let mut slot = state.inner.write().await;
    if slot.phase == NavPhase::Cancelled {
        return CancelOutcome {
            state: NavState {
                status: slot.phase.as_str().to_string(),
                message: slot.message.clone(),
                goal_id,
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            },
        };
    }
    let mut failures = Vec::new();
    if let Err(error) = cancel_result {
        warn!("publish /nav/cancel 失败: {}", error);
        failures.push(error);
    }
    if let Err(error) = zero_result {
        warn!("publish zero cmd_vel 失败: {}", error);
        failures.push(error);
    }
    slot.phase = NavPhase::CancelPending;
    slot.message = if failures.is_empty() {
        "取消请求已提交,等待真实 stack terminal state".into()
    } else {
        format!("取消请求尚未确认: {}", failures.join("; "))
    };
    CancelOutcome {
        state: NavState {
            status: slot.phase.as_str().to_string(),
            message: slot.message.clone(),
            goal_id,
            distance_remaining: 0.0,
            timestamp_ns: now_ns(),
        },
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

/// publish `/nav/goal` typed frame。
async fn publish_goal_frame(
    handle: &GeometryRuntimeHandle,
    input: &SetGoalInput,
) -> Result<(), String> {
    let channel = channel_registry::resolve_channel(
        GOAL_CHANNEL_RUNTIME,
        GOAL_CHANNEL_PLATFORM,
        GOAL_CHANNEL_DEFAULT,
    );
    let (topic, _suffix) = parse_channel_with_suffix(&channel)?;
    let payload = typed_codec::encode_pose_stamped_payload(&input.target)
        .map_err(|e| format!("PoseStamped LCM 编码失败: {e}"))?;
    let frame: RawLcmFrame =
        typed_codec::build_typed_frame(topic, goal_msg_type(), payload, seqno_for(handle).await);
    handle
        .transport
        .publish(frame)
        .await
        .map_err(|e| format!("LcmTransport::publish 失败: {}", e))
}

/// publish `/nav/cancel` typed frame。
async fn publish_cancel_frame(handle: &GeometryRuntimeHandle, goal_id: &str) -> Result<(), String> {
    let channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel#nuwax_msgs.CancelNavigation",
    );
    let (topic, _suffix) = parse_channel_with_suffix(&channel)?;
    let payload = typed_codec::encode_nav_cancel_payload(goal_id, 0)
        .map_err(|error| format!("CancelNavigation LCM 编码失败: {error}"))?;
    let msg_type = LcmMessageType::new("nuwax_msgs/CancelNavigation")
        .map_err(|e| format!("LcmMessageType::new 失败: {}", e))?;
    let frame: RawLcmFrame =
        typed_codec::build_typed_frame(topic, msg_type, payload, seqno_for(handle).await);
    handle
        .transport
        .publish(frame)
        .await
        .map_err(|e| format!("LcmTransport::publish 失败: {}", e))
}

/// publish `/cmd_vel` typed frame(zero)。
async fn publish_zero_cmd_vel(handle: &GeometryRuntimeHandle) -> Result<(), String> {
    publish_cmd_vel(handle, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]).await
}

/// publish `/cmd_vel` typed frame。
async fn publish_cmd_vel(
    handle: &GeometryRuntimeHandle,
    linear: [f32; 3],
    angular: [f32; 3],
) -> Result<(), String> {
    let channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_CMD_VEL",
        "/cmd_vel#geometry_msgs.Twist",
    );
    let (topic, _suffix) = parse_channel_with_suffix(&channel)?;
    let payload = typed_codec::encode_cmd_vel_payload(linear, angular, now_ns());
    let msg_type = LcmMessageType::new("geometry_msgs/Twist")
        .map_err(|e| format!("LcmMessageType::new 失败: {}", e))?;
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
    use std::ffi::OsString;
    use std::sync::Mutex;
    use std::time::Duration;
    use tokio::time::sleep;

    static NAV_PROFILE_ENV_LOCK: Mutex<()> = Mutex::new(());

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

    /// 构造一个测试 handle:test backend(走 real-lcm + test_lcm_url 隔离端口)+ 一个非空 pose 与 map。
    ///
    /// 注意:不能直接构造 `LcmTransport`(其字段非 pub);这里用 `build_handle_for_tests`
    /// 构造,backend 走 real-lcm,LCM publish 走隔离端口。
    /// 2026-07-08 fake-lcm 物理删除后,test handle 改走 real backend,默认 true
    /// 仍兼容旧测试断言。
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
    }

    /// Phase 6 / Rework-9:Lane 6 注入接口(setter) → handle.cmd_vel_sink_ready() → readiness。
    ///
    /// 验证:
    /// - test helper `build_handle_for_tests` 默认 `true`(单测兼容,
    ///   2026-07-08 fake-lcm 物理删除后,test helper 改走 real backend)
    /// - 生产 `spawn_geometry_runtime` 默认 `false`(由 Python 写回)
    /// - setter 写入后立刻被 readiness 读到
    /// - override 优先级 > handle(setter 不影响 override 分支)
    #[tokio::test]
    async fn phase6_handle_cmd_vel_sink_ready_setter_propagates_to_readiness() {
        // Phase 7:static TF_COUNT 跨 test 累计 → 重置本 test 看到的基础值。
        super::reset_tf_count_for_test();
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        // 1. test helper 默认值 true(无 override 时 readiness.cmd_vel_sink_ready 跟 lcm_backend)
        assert!(
            handle.cmd_vel_sink_ready(),
            "test helper 默认 cmd_vel_sink_ready 必须 true"
        );
        // 2. setter 关掉
        handle.set_cmd_vel_sink_ready(false);
        // 3. readiness 必须反映 handle 关掉的状态(override=None → 回退 handle.cmd_vel_sink_ready())
        let d = compute_readiness(&handle, &state).await;
        assert!(
            !d.cmd_vel_sink_ready,
            "setter 写入 false 后,readiness.cmd_vel_sink_ready 必须 false"
        );
        assert!(
            !d.ready,
            "cmd_vel_sink_ready=false 时 readiness.ready 必须 false"
        );
        assert!(
            d.reason.contains("cmd_vel"),
            "readiness.reason 必须含 cmd_vel 提示,实际={}",
            d.reason
        );
        // 4. override 优先级 > handle:override=true 覆盖 handle=false
        set_cmd_vel_sink_ready_override(Some(true));
        let d2 = compute_readiness(&handle, &state).await;
        assert!(d2.cmd_vel_sink_ready, "override=true 应覆盖 handle=false");
        set_cmd_vel_sink_ready_override(None);
        // 5. 清理
        handle.set_cmd_vel_sink_ready(true);
    }

    /// Phase 6 / Rework-9:test helper 默认仍 `true`,setter 翻转后 readiness 拒绝。
    ///
    /// 证明 test helper profile 行为不变:旧单测断言
    /// `默认 cmd_vel_sink_ready 必须 true` 仍然成立,因为生产 spawn path
    /// (`spawn_geometry_runtime`)与 test helper 是两条独立
    /// 构造路径(参见 `lcm_runtime.rs` build_handle_for_tests / spawn 区分说明)。
    /// 2026-07-08 fake-lcm 物理删除后,test helper 改走 real backend,默认 true
    /// 仍兼容旧测试断言。
    /// 本 test 同时把生产侧的"setter 关掉 → readiness 拒绝 → reason 含
    /// cmd_vel"分支再固化一次,作为 rework9 的回归测试锚点。
    #[tokio::test]
    async fn rework9_test_helper_default_true_setter_blocks_readiness() {
        super::reset_tf_count_for_test();
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        assert!(
            handle.cmd_vel_sink_ready(),
            "test helper 必须默认 true(test helper 不接 Python bridge)"
        );
        // setter 关掉后:
        handle.set_cmd_vel_sink_ready(false);
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.ready, "setter=false 后 readiness 必须 false");
        assert!(!d.cmd_vel_sink_ready);
        let reason_contains =
            d.reason.contains("cmd_vel") || d.reason.contains(REJECT_REASON_CMD_VEL_SINK_DOWN);
        assert!(
            reason_contains,
            "reason 必须指向 cmd_vel sink 未就绪,实际={}",
            d.reason
        );
        // 清理
        handle.set_cmd_vel_sink_ready(true);
    }

    #[tokio::test]
    async fn readiness_unready_when_cmd_vel_sink_down() {
        // Phase 7:real production profile + cmd_vel_sink_ready=false → readiness false
        // 同样 reset TF_COUNT,避免并行测试累计导致其他 test 不稳。
        super::reset_tf_count_for_test();
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        // 注入 cmd_vel_sink_ready=false,模拟 Lane 6 watchdog 未就绪
        set_cmd_vel_sink_ready_override(Some(false));
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.ready, "cmd_vel_sink_ready=false 时 readiness 必须 false");
        assert!(!d.cmd_vel_sink_ready);
        // 即使 tf / costmap 都齐,readiness.ready 仍 false
        let mut s = handle.state.write().await;
        use crate::lcm_runtime::{MapState, PoseState};
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
        let d2 = compute_readiness(&handle, &state).await;
        assert!(
            !d2.ready,
            "即便 tf+costmap 都齐,cmd_vel_sink_ready=false 仍 false"
        );
        assert!(d2.tf);
        assert!(d2.costmap);
        assert!(!d2.cmd_vel_sink_ready);
        // reason 必须包含 cmd_vel_sink 提示
        assert!(d2.reason.contains("cmd_vel"));
        // 清理 override,避免污染其他测试
        set_cmd_vel_sink_ready_override(None);
    }

    #[tokio::test]
    async fn readiness_unready_lists_new_fields() {
        // Phase 7:tf_count 由 lcm_runtime tf feed path 调用
        // `note_tf_observed()` 维护;test env 没有真实 LCM 推 /tf,
        // 所以本测试 reset 后整个 process 期间 tf_count 始终为 0,
        // 不存在 parallel test race。
        super::reset_tf_count_for_test();
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
        assert!(
            d.cmd_vel_sink_ready,
            "test handle 默认 lcm_backend=true → sink ready=true"
        );
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

    #[tokio::test]
    async fn set_goal_failed_when_no_pose() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        // 不动 pose / map → readiness.tf=false
        let state = NavRuntimeState::from_env();
        let input = SetGoalInput {
            goal_id: "g2".into(),
            frame_id: "map".into(),
            target: PoseState {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
                frame_id: "map".into(),
                timestamp_ns: 0,
            },
        };
        let outcome = set_goal(&handle, &state, input).await;
        assert_eq!(outcome.state.status, "FAILED");
        assert!(outcome.state.message.contains("未就绪"));
    }

    #[tokio::test]
    async fn cancel_clears_active_goal() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        observe_healthy_stack(&state).await;
        let input = SetGoalInput {
            goal_id: "g3".into(),
            frame_id: "map".into(),
            target: PoseState {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
                frame_id: "map".into(),
                timestamp_ns: 0,
            },
        };
        let _ = set_goal(&handle, &state, input).await;
        let outcome = cancel(&handle, &state, String::new()).await;
        assert_eq!(outcome.state.status, "PENDING");
        assert_eq!(outcome.state.goal_id, "g3");
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::CancelPending);
        assert!(slot.goal.is_some());
    }

    #[tokio::test]
    async fn cancel_with_no_active_goal_is_safe() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let outcome = cancel(&handle, &state, "".into()).await;
        assert_eq!(outcome.state.status, "PENDING");
    }

    #[tokio::test]
    async fn readiness_unready_lists_all_false() {
        // Phase 7:static TF_COUNT 跨 test 累计 → 重置本 test 看到的基础值。
        super::reset_tf_count_for_test();
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

    #[tokio::test]
    async fn readiness_requires_fresh_stack_state_observation() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();

        let missing = compute_readiness(&handle, &state).await;
        assert!(!missing.ready);
        assert!(!missing.planner);
        assert!(!missing.stack_state_observed);

        observe_healthy_stack(&state).await;
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
        observe_healthy_stack(&state).await;
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
                goal_id: "stack-owned-goal".into(),
                frame_id: "map".into(),
                target: PoseState {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                    qx: 0.0,
                    qy: 0.0,
                    qz: 0.0,
                    qw: 1.0,
                    frame_id: String::new(),
                    timestamp_ns: now_ns(),
                },
            },
        )
        .await;

        assert_eq!(outcome.state.status, "ACCEPTED");
        assert_eq!(outcome.state.goal_id, "stack-owned-goal");
        let goal = tokio::time::timeout(Duration::from_secs(1), goals.next())
            .await
            .expect("goal was not published")
            .expect("goal stream closed")
            .expect("goal transport error");
        let mut encoded_goal = nuwax_codec::lcm::schema::POSE_STAMPED_FINGERPRINT.to_vec();
        encoded_goal.extend_from_slice(&goal.payload);
        let goal = nuwax_codec::lcm::schema::decode_pose_stamped(&encoded_goal)
            .expect("decode submitted goal");
        assert_eq!(goal.header.frame_id, "map");
        assert_eq!(goal.pose.position[0], 1.0);
        assert!(
            tokio::time::timeout(Duration::from_millis(150), commands.next())
                .await
                .is_err(),
            "geometry facade must not publish an immediate /cmd_vel"
        );
    }

    #[tokio::test]
    async fn cancel_publish_failures_remain_nonterminal_until_stack_state() {
        let state = NavRuntimeState::from_env();
        {
            let mut slot = state.inner.write().await;
            slot.goal = Some(ActiveGoal {
                id: "cancel-me".into(),
                frame_id: "map".into(),
                target: PoseState::default(),
                accepted_at_ns: now_ns(),
            });
        }

        let outcome = finish_cancel_request(
            &state,
            "cancel-me".into(),
            Err("cancel publish failed".into()),
            Err("zero publish failed".into()),
        )
        .await;
        assert_eq!(outcome.state.status, "PENDING");
        assert!(outcome.state.message.contains("cancel publish failed"));
        assert!(outcome.state.message.contains("zero publish failed"));
        assert!(state.inner.read().await.goal.is_some());

        let observed = state
            .observe_stack_state(NavState {
                status: "CANCELLED".into(),
                message: "stack acknowledged cancellation".into(),
                goal_id: "cancel-me".into(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            })
            .await;
        assert_eq!(observed.status, "CANCELLED");
        assert!(state.inner.read().await.goal.is_none());

        let repeated = state
            .observe_stack_state(NavState {
                status: "CANCELLED".into(),
                message: "stack acknowledged cancellation".into(),
                goal_id: String::new(),
                distance_remaining: 0.0,
                timestamp_ns: now_ns(),
            })
            .await;
        assert_eq!(repeated.goal_id, "cancel-me");
    }
}
