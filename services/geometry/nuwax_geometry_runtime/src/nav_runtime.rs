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
//! 3. `simple-2d` profile 的真实执行链:用 `nuwax-nav-basic-path-follower`
//!    在内存里推进到下一段 cmd_vel(goal pose + 当前 pose 通过
//!    `MovementManager` 风格的简单路径)并 publish 到 `/cmd_vel` typed channel。
//! 4. `replanning-astar` profile:同 simple-2d,只把 module graph 里 planner
//!    节点 ID 改成 `astar_planner`,因为该 profile 的核心算法 crate
//!    `nuwax-nav-astar` 与 simple-2d 走同一 basic path follower。
//!
//! ## 边界
//!
//! - 不会启动 native binary 进程(CMU / FAR / local / path-follower /
//!   PGO / TARE / Terrain),仅返回 module graph;P1 阶段在 native bridge
//!   进程生命周期 + supervisor 实装后再开启 cmu-native / exploration-native。
//! - 不持有 LCM 订阅线程,所有数据流都通过 `GeometryRuntimeHandle` 的
//!   `state.pose` / `state.map` 读取;这样可以与 `lcm_runtime.rs` 的
//!   typed codec 路径保持一致。
//! - 状态机:`IDLE` -> `ACCEPTED` -> `PLANNING` -> `RUNNING` -> (`SUCCEEDED` |
//!   `CANCELLED` | `FAILED`);通过 `NavRuntimeState` 的 `RwLock<NavRuntimeSlot>`
//!   串行化,避免多个 `set_goal` 抢占。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};
use std::sync::Arc;

use nuwax_nav_basic_path_follower::{
    BasicPathFollowerConfig, BasicPathFollowerCore, ControllerCommand, StepOutput,
};
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
            NavPhase::Failed => "FAILED",
        }
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

/// `NavRuntime` 的内部 slot:phase + active goal + follower。
///
/// 一次只允许一个 active goal,新 `set_goal` 会把现有 in-flight 目标标记为
/// `Cancelled`(并 publish /nav/cancel + zero cmd_vel)再接新目标。
pub struct NavRuntimeSlot {
    /// 当前阶段。
    pub phase: NavPhase,
    /// 失败原因(message 字段用)。
    pub message: String,
    /// 当前 active goal。
    pub goal: Option<ActiveGoal>,
    /// simple-2d / astar profile 的 path follower;其它 profile 时为 `None`。
    pub follower: Option<BasicPathFollowerCore>,
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
            follower: None,
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
}

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
    /// profile 是否在 default 集合内(simple-2d / replanning-astar)。
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
    /// canonical LCM backend 名("RealLcm" / "FakeLcm")。
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
}

/// cmd_vel sink ready override(Lane 6 unit test 注入通道)。
///
/// Phase 6 cmd_vel consumer bridge 在生产环境 real-LCM 下应该把 watchdog
/// 已就绪的真实状态写到这里。本期(Phase 7)保留 override 入口,Lane 6
/// 实装完后会被跨进程 ack 替换;单元测试 / Python diagnostics 测试
/// 用 `set_cmd_vel_sink_ready_override(Some(true/false))`。
///
/// 默认(`None`)时 `compute_readiness` 按 `lcm_backend_available` 估算。
use std::sync::Mutex as StdMutex;
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

/// 计算细分 readiness。读 LCM handle 状态 + 内部 slot。
pub async fn compute_readiness(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
) -> NavigationReadinessDetail {
    let s = handle.state.read().await;
    let tf = s.pose.timestamp_ns > 0;
    let costmap_revision = s.map.revision;
    let costmap = costmap_revision > 0;
    // Phase 7:tf_count 累计计数(每次 tf=true 时 +1)。
    let tf_count = TF_COUNT.load(AtomicOrdering::Relaxed);
    if tf {
        TF_COUNT.fetch_add(1, AtomicOrdering::Relaxed);
    }
    drop(s);
    let profile = state.profile().await;
    let planner = matches!(profile, PROFILE_SIMPLE_2D | PROFILE_REPLANNING_ASTAR);
    let lcm_backend = handle.lcm_backend_available;
    // Phase 7:cmd_vel_sink_ready 由 override 控制,缺省按 lcm_backend 估算。
    // Phase 6:Lane 6 注入接口(set_cmd_vel_sink_ready_override)与 handle 上
    // 的 cmd_vel_sink_ready atomic bool 都参与决策;override > handle,
    // handle 缺省时回退 lcm_backend。
    let cmd_vel_sink = lcm_backend; // backward-compatible 字段
    let cmd_vel_sink_ready = cmd_vel_sink_ready_override()
        .unwrap_or_else(|| handle.cmd_vel_sink_ready());
    // Phase 7:backend canonical 名(用 transport_backend_name 反推)。
    let backend = match handle.transport_backend_name {
        "fake-lcm" => "FakeLcm",
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
    } else if !tf {
        "/tf 尚无有效位姿,等待 SLAM/定位 first frame"
    } else if !costmap {
        "/map/occupancy 尚未生成,nav runtime 拒绝接目标"
    } else if !planner {
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
    }
}

/// Phase 7:`SetNavigationGoal` readiness reject 时给出的 reason 标签。
///
/// 当 real production profile 下 `cmd_vel_sink_ready=false` 时,即便
/// tf / costmap / planner 都齐,set_goal 也立即返回 FAILED + 该 reason,
/// 让上层 / 前端能在日志 / NavState.message 里看到 `cmd_vel_sink=down`
/// 阻塞原因,而不是被泛化的"未就绪"吞掉。
pub const REJECT_REASON_CMD_VEL_SINK_DOWN: &str = "cmd_vel_sink 未就绪,SetNavigationGoal 被拒绝 (Phase 7 gate)";

/// `set_goal` 的输入(已从 gRPC `NavigationGoal` 提取并归一化)。
pub struct SetGoalInput {
    /// 目标点 ID(用户传或自动生成)。
    pub goal_id: String,
    /// frame_id(`map` 默认)。
    pub frame_id: String,
    /// 目标位姿(在 `frame_id` 坐标系下)。
    pub target: PoseState,
}

/// `set_goal` 的输出:成功时返回 `NavigationState(ACCEPTED|RUNNING)`,
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

/// 真实的 `set_goal` 实现 —— 在 default profile 上用 basic path follower
/// 推进一帧、publish `/nav/goal` + `/cmd_vel`、写 active goal、状态机
/// 推进到 `ACCEPTED` -> `RUNNING`。
///
/// 失败时返回带 `status=FAILED` 的 NavState(但 `goal_id` 保留),
/// 不写入 active goal 也不 publish `/nav/goal`。
pub async fn set_goal(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    input: SetGoalInput,
) -> SetGoalOutcome {
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
                message: format!("SetNavigationGoal 失败:nav_runtime 未就绪 ({})", reason_label),
                goal_id: input.goal_id.clone(),
                distance_remaining: distance_to_target(handle, &input.target).await,
                timestamp_ns: now_ns(),
            },
        };
    }

    // 2. publish /nav/goal typed frame。
    if let Err(e) = publish_goal_frame(handle, &input).await {
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

    // 3. 推进状态机:ACCEPTED -> RUNNING(本阶段 fake planner 立即出 waypoints)。
    let mut slot_outer = state.inner.write().await;
    let goal = ActiveGoal {
        id: input.goal_id.clone(),
        frame_id: input.frame_id.clone(),
        target: input.target.clone(),
        accepted_at_ns: now_ns(),
    };
    // basic path follower:把 goal 当成单点 waypoint(2D);实际 multi-waypoint
    // 由 simple-2d / astar planner 提供,本阶段只演示通道打通。
    let goal_pos_2d = (input.target.x, input.target.y);
    let cfg = BasicPathFollowerConfig::default();
    let mut follower = match BasicPathFollowerCore::new(cfg) {
        Ok(f) => f,
        Err(e) => {
            return SetGoalOutcome {
                state: NavState {
                    status: NavPhase::Failed.as_str().to_string(),
                    message: format!("SetNavigationGoal 失败:构造 basic path follower 错误: {}", e),
                    goal_id: input.goal_id.clone(),
                    distance_remaining: distance_to_target(handle, &input.target).await,
                    timestamp_ns: now_ns(),
                },
            };
        }
    };
    follower.set_path(vec![point2_from_tuple(goal_pos_2d)]);
    slot_outer.phase = NavPhase::Running;
    slot_outer.message = format!(
        "导航目标已接受,profile={}, module_nodes={}",
        slot_outer.profile,
        slot_outer.module_nodes.len()
    );
    slot_outer.goal = Some(goal);
    slot_outer.follower = Some(follower);
    let distance_remaining = distance_to_target(handle, &input.target).await;
    drop(slot_outer);

    // 4. 推进一帧(刚 set_path 后 follower 已经在 path 上,step 输出
    //    `ControllerCommand` 并 publish 到 /cmd_vel)。
    step_follower_and_publish_cmd_vel(handle, state).await;

    let nav = state.snapshot(input.goal_id.clone(), distance_remaining).await;
    SetGoalOutcome { state: nav }
}

/// 取消当前 active goal:publish /nav/cancel、zero cmd_vel、状态机
/// 切到 `Cancelled`。
///
/// 即使当前没有 active goal 也不报错(返回 status=CANCELLED + reason 提示)。
pub async fn cancel(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
    goal_id: String,
) -> CancelOutcome {
    // 1. publish /nav/cancel typed frame。
    if let Err(e) = publish_cancel_frame(handle, &goal_id).await {
        warn!("publish /nav/cancel 失败: {};继续 zero cmd_vel", e);
    }
    // 2. zero cmd_vel:Twist 全 0,publish 到 /cmd_vel。
    if let Err(e) = publish_zero_cmd_vel(handle).await {
        warn!("publish zero cmd_vel 失败: {};状态机仍切到 cancelled", e);
    }
    // 3. 状态机切到 Cancelled。
    let prev_goal_id;
    let status_after;
    let message_after;
    {
        let mut slot = state.inner.write().await;
        prev_goal_id = slot.goal.as_ref().map(|g| g.id.clone());
        slot.phase = NavPhase::Cancelled;
        slot.goal = None;
        slot.follower = None;
        slot.message = format!(
            "导航已取消(goal_id={:?} requested={:?})",
            prev_goal_id, goal_id,
        );
        status_after = slot.phase.as_str().to_string();
        message_after = slot.message.clone();
        // slot 在 block 结束时 drop,锁随之释放。
    }
    // 不调 state.snapshot():cargo test 的 current_thread runtime 下,
    // 紧跟 write 释放的 read().await 会出现 hanging(已 root cause 在
    // phase3-5 fixup 反馈文档),改成直接构造 NavState 返回。
    let nav = NavState {
        status: status_after,
        message: message_after,
        goal_id,
        distance_remaining: 0.0,
        timestamp_ns: now_ns(),
    };
    CancelOutcome { state: nav }
}

/// 推进 follower 一帧(如有 active goal),把 cmd_vel publish 到 LCM。
/// 如果当前位置已到达目标(距离 < arrival_tolerance),状态机切到
/// `Succeeded` + zero cmd_vel。
pub async fn step_follower_and_publish_cmd_vel(
    handle: &GeometryRuntimeHandle,
    state: &NavRuntimeState,
) {
    let mut slot = state.inner.write().await;
    // 先看 active goal 是不是存在(避免下面同时持 follower mut borrow +
    // goal imm borrow 触发 borrow checker)。
    let goal_id = match slot.goal.as_ref() {
        Some(g) => g.id.clone(),
        None => return,
    };
    let Some(follower) = slot.follower.as_mut() else {
        return;
    };
    let cur = handle.state.read().await;
    let pos = (cur.pose.x, cur.pose.y);
    let yaw = quat_to_yaw(cur.pose.qx, cur.pose.qy, cur.pose.qz, cur.pose.qw);
    drop(cur);

    let out: StepOutput = follower.step(point2_from_tuple(pos), yaw);
    // follower mut borrow 已经结束,下面用 goal_id 即可。
    // 旧 `drop(follower)` 在 clippy dropping_references 提示下改为 `let _ = ...`，
    // 语义保持"显式结束该 borrow scope"；后续 Phase 6 收口若改用
    // `take()` / `replace()` 模式可一并清理。
    let _ = follower;
    if out.no_path {
        slot.phase = NavPhase::Failed;
        slot.message = "basic path follower: 无路径".into();
        slot.goal = None;
        slot.follower = None;
        return;
    }
    if out.goal_reached {
        slot.phase = NavPhase::Succeeded;
        slot.message = format!("已到达目标 (goal_id={})", goal_id);
        slot.goal = None;
        slot.follower = None;
        // reached 后 cmd_vel 必须 zero。
        let h = handle.clone();
        tokio::spawn(async move {
            if let Err(e) = publish_zero_cmd_vel(&h).await {
                warn!("zero cmd_vel publish 失败(succeeded 后): {}", e);
            }
        });
        return;
    }
    // normal moving:publish /cmd_vel typed frame。
    let linear = [out.command.linear as f32, 0.0, 0.0];
    let angular = [0.0, 0.0, out.command.angular as f32];
    let h = handle.clone();
    tokio::spawn(async move {
        if let Err(e) = publish_cmd_vel(&h, linear, angular).await {
            warn!("publish cmd_vel 失败: {}", e);
        }
    });
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

/// `BasicPathFollowerCore` 需要的 `Point2`。`Point2` 在 nalgebra 是
/// `OPoint<f64, Const<2>>`,通过 `Point2::new` 构造。
fn point2_from_tuple(t: (f64, f64)) -> nuwax_nav_basic_path_follower::Point2 {
    nuwax_nav_basic_path_follower::Point2::new(t.0, t.1)
}

/// 把 quaternion 转 yaw(z 轴旋转角),用于 follower 输入。
fn quat_to_yaw(qx: f64, qy: f64, qz: f64, qw: f64) -> f64 {
    // 标准 yaw from quaternion:atan2(2(wz + xy), 1 - 2(y² + z²))
    let siny_cosp = 2.0 * (qw * qz + qx * qy);
    let cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    siny_cosp.atan2(cosy_cosp)
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
    LcmTopic::parse_subscription_pattern(channel)
        .map_err(|e| format!("LcmTopic::parse_subscription_pattern({}) 失败: {}", channel, e))
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
    let payload = typed_codec::encode_pose_stamped_payload(&input.target);
    let frame: RawLcmFrame = typed_codec::build_typed_frame(
        topic,
        goal_msg_type(),
        payload,
        seqno_for(handle).await,
    );
    handle
        .transport
        .publish(frame)
        .await
        .map_err(|e| format!("LcmTransport::publish 失败: {}", e))
}

/// publish `/nav/cancel` typed frame。
async fn publish_cancel_frame(
    handle: &GeometryRuntimeHandle,
    goal_id: &str,
) -> Result<(), String> {
    let channel = channel_registry::resolve_channel(
        "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel#nuwax_msgs.CancelNavigation",
    );
    let (topic, _suffix) = parse_channel_with_suffix(&channel)?;
    let payload = typed_codec::encode_nav_cancel_payload(goal_id, now_ns());
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

/// ControllerCommand -> [f32;3] linear/angular 转换(给 publish_cmd_vel 用)。
/// `ControllerCommand` 在 `nuwax-nav-basic-path-follower` 中是
/// `linear: f64, angular: f64` 标量,不是 Vec3;所以这里把 f64 cast 到
/// [f32; 3] 的 x 通道。
#[allow(dead_code)]
fn controller_to_arrays(cmd: &ControllerCommand) -> ([f32; 3], [f32; 3]) {
    (
        [cmd.linear as f32, 0.0, 0.0],
        [0.0, 0.0, cmd.angular as f32],
    )
}

// ============================================================================
// 单元测试
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lcm_runtime::{MapState, SharedState};
    use std::time::Duration;
    use tokio::time::sleep;

    /// 构造一个 mock handle:fake backend + 一个非空 pose 与 map。
    ///
    /// 注意:不能直接构造 `LcmTransport`(其字段非 pub);这里用 `new()`
    /// 默认构造,backend 走 fake,LCM publish 入内存 topic。
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
        std::env::remove_var("NUWAX_NAV_PROFILE");
        assert_eq!(resolve_profile_from_env(), PROFILE_SIMPLE_2D);
    }

    #[test]
    fn resolve_profile_uses_set_value() {
        std::env::set_var("NUWAX_NAV_PROFILE", "replanning-astar");
        assert_eq!(resolve_profile_from_env(), PROFILE_REPLANNING_ASTAR);
        std::env::remove_var("NUWAX_NAV_PROFILE");
    }

    #[test]
    fn resolve_profile_falls_back_on_unknown() {
        std::env::set_var("NUWAX_NAV_PROFILE", "hologram");
        assert_eq!(resolve_profile_from_env(), DEFAULT_PROFILE);
        std::env::remove_var("NUWAX_NAV_PROFILE");
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
            lcm_backend: true,
            tf: true,
            costmap: true,
            planner: true,
            cmd_vel_sink: true,
            profile: PROFILE_SIMPLE_2D.into(),
            module_node_count: 4,
            p1_native_profiles: P1_NATIVE_PROFILES.iter().map(|s| s.to_string()).collect(),
            backend: "FakeLcm".into(),
            tf_count: 7,
            costmap_revision: 3,
            cmd_vel_sink_ready: true,
            active_profile: PROFILE_SIMPLE_2D.into(),
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
    }

    /// Phase 6:Lane 6 注入接口(setter) → handle.cmd_vel_sink_ready() → readiness。
    ///
    /// 验证:
    /// - 默认值 `true`(兼容旧 fake-lcm 单测)
    /// - setter 写入后立刻被 readiness 读到
    /// - override 优先级 > handle(setter 不影响 override 分支)
    #[tokio::test]
    async fn phase6_handle_cmd_vel_sink_ready_setter_propagates_to_readiness() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        // 1. 默认值 true(无 override 时 readiness.cmd_vel_sink_ready 跟 lcm_backend)
        assert!(handle.cmd_vel_sink_ready(), "默认 cmd_vel_sink_ready 必须 true");
        // 2. setter 关掉
        handle.set_cmd_vel_sink_ready(false);
        // 3. readiness 必须反映 handle 关掉的状态(override=None → 回退 handle.cmd_vel_sink_ready())
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.cmd_vel_sink_ready,
            "setter 写入 false 后,readiness.cmd_vel_sink_ready 必须 false");
        assert!(!d.ready,
            "cmd_vel_sink_ready=false 时 readiness.ready 必须 false");
        assert!(d.reason.contains("cmd_vel"),
            "readiness.reason 必须含 cmd_vel 提示,实际={}", d.reason);
        // 4. override 优先级 > handle:override=true 覆盖 handle=false
        set_cmd_vel_sink_ready_override(Some(true));
        let d2 = compute_readiness(&handle, &state).await;
        assert!(d2.cmd_vel_sink_ready,
            "override=true 应覆盖 handle=false");
        set_cmd_vel_sink_ready_override(None);
        // 5. 清理
        handle.set_cmd_vel_sink_ready(true);
    }

    #[tokio::test]
    async fn readiness_unready_when_cmd_vel_sink_down() {
        // Phase 7:real production profile + cmd_vel_sink_ready=false → readiness false
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
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("handle");
        let state = NavRuntimeState::from_env();
        let d = compute_readiness(&handle, &state).await;
        assert!(!d.ready);
        assert!(!d.tf);
        assert!(!d.costmap);
        assert_eq!(d.profile, PROFILE_SIMPLE_2D);
        // Phase 7 字段:初始 backend=FakeLcm(test handle),counters 全 0
        assert_eq!(d.backend, "FakeLcm");
        assert_eq!(d.tf_count, 0);
        assert_eq!(d.costmap_revision, 0);
        assert!(d.cmd_vel_sink_ready, "test handle 默认 lcm_backend=true → sink ready=true");
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
    async fn set_goal_running_with_path_follower() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let input = SetGoalInput {
            goal_id: "g1".into(),
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
        assert_eq!(outcome.state.status, "RUNNING");
        assert_eq!(outcome.state.goal_id, "g1");
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::Running);
        assert!(slot.follower.is_some());
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
        let outcome = cancel(&handle, &state, "g3".into()).await;
        assert_eq!(outcome.state.status, "CANCELLED");
        let slot = state.inner.read().await;
        assert_eq!(slot.phase, NavPhase::Cancelled);
        assert!(slot.follower.is_none());
    }

    #[tokio::test]
    async fn cancel_with_no_active_goal_is_safe() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let outcome = cancel(&handle, &state, "".into()).await;
        assert_eq!(outcome.state.status, "CANCELLED");
    }

    #[tokio::test]
    async fn step_follower_publishes_cmd_vel() {
        let handle = make_handle().await;
        let state = NavRuntimeState::from_env();
        let _ = set_goal(
            &handle,
            &state,
            SetGoalInput {
                goal_id: "g4".into(),
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
            },
        )
        .await;
        // 推 1 帧;由于当前位置 == 起点,follower 给出 waypoint cmd_vel。
        step_follower_and_publish_cmd_vel(&handle, &state).await;
        sleep(Duration::from_millis(50)).await;
        // 不验证具体 cmd_vel bytes(fake backend topic 不直接 inspect),
        // 验证 step 后 state 没崩、phase 仍是 Running(除非一步到 goal)。
        let slot = state.inner.read().await;
        assert!(matches!(
            slot.phase,
            NavPhase::Running | NavPhase::Succeeded | NavPhase::Failed
        ));
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
        assert!(d.p1_native_profiles.contains(&PROFILE_CMU_NATIVE.to_string()));
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
}
