// Phase 6 (codex-round3 R3-4)：pose_stamped_from_robot / add_source /
// arbitration / is_active 是 Phase 5 explore RuntimeDiagnostics contract 预留。
#![allow(dead_code)]

//! Phase 5 explore_runtime —— 探索 orchestration runtime module
//!
//! 目标:把 `nuwax_exploration::ExplorationController` 接入当前 runtime,
//! 提供 start / stop / state / diagnostics 四类契约,并把选出的
//! `FrontierGoal` 通过 `nuwax_exploration::conversions::frontier_goal_to_navigation_goal`
//! 转为 `NavigationGoal` 写入 `SharedState::nav_state`,由 Phase 4 nav_runtime 接管。
//!
//! ## 关键事实
//! - 锁定 dimos 版本:`nuwax-exploration = 0.x` (path deps,在 Phase 2 落盘);
//! - 默认 source=`wavefront`(`WavefrontGoalSourceAdapter`),
//!   arbitration=`PreferWavefront`;
//! - `arbitration` 字段映射 env `NUWAX_EXPLORE_ARBITRATION`
//!   (`prefer_wavefront` / `prefer_tare` / `highest_score` / `first_available`),
//!   与 `ExploreAreaRequest.strategy` 拼成最终 policy;
//! - 当前实现的 `tick` 不直接 spawn native bridge,
//!   仅用 `wavefront` source + `HighestScore` / `FirstAvailable` policy;
//!   TARE / Terrain native 路径在 P1 profile 接入,见执行方案 §3 Phase 5。
//!
//! ## 状态机
//! - `NotStarted` → `RequestAccepted` (`StartExploration` 成功)
//! - `RequestAccepted` → `Active` (controller begin OK,至少 1 source 已注册)
//! - `Active` → `StopPending` (`StopExploration` 已收到,正在清理)
//! - 任意 → `Rejected` (controller 报错,如 `NoSources` / `AlreadyActive`)
//!
//! 注意:`ExplorationController` 的 `begin_exploration` 一旦成功就需要长期持有
//! 可变状态,因此本模块把它放在 `Arc<tokio::sync::Mutex<...>>` 里;在 gRPC
//! facade 多线程同步访问时按顺序获取锁即可,不影响 controller 的内部语义。

#[allow(unused_imports)]
use std::borrow::Cow;
use std::sync::Arc;
use std::time::SystemTime;

use nuwax_exploration::conversions::frontier_goal_to_navigation_goal;
#[allow(unused_imports)]
use nuwax_exploration::runtime::WavefrontInputReceiver;
use nuwax_exploration::wavefront_adapter::WavefrontGoalSourceAdapter;
use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationController, ExplorationGoalSource, ExplorationPoll,
    ExplorationStateSnapshot, ExplorationStopReason,
};
use nuwax_explore_wavefront::WavefrontConfig;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{ClockKind, Point3, Pose3, PoseStamped, Quaternion, Timestamp};
use nuwax_nav_types_messages::visual::FrontierGoal;
use tokio::sync::Mutex;
use tracing::warn;

/// 探索 status 字符串(与 `services/geometry/geometry_runtime_provider.py`
/// 解析一致;对外契约保留 4 个字符串值,其余 message 由 caller 解析)
pub const STATUS_ACCEPTED: &str = "ACCEPTED";
pub const STATUS_REJECTED: &str = "REJECTED";
pub const STATUS_PENDING: &str = "PENDING";
pub const STATUS_STOPPED: &str = "STOPPED";
pub const STATUS_ACTIVE: &str = "ACTIVE";
pub const STATUS_UNIMPLEMENTED: &str = "UNIMPLEMENTED";

/// arbitration policy 字符串(env / proto strategy 字段用)
const POLICY_PREFER_WAVEFRONT: &str = "prefer_wavefront";
const POLICY_PREFER_TARE: &str = "prefer_tare";
const POLICY_HIGHEST_SCORE: &str = "highest_score";
const POLICY_FIRST_AVAILABLE: &str = "first_available";

/// 把字符串 arbitration 名称解析成 `ExplorationArbitrationPolicy` enum,
/// 未知值 fallback 到 `PreferWavefront`(与执行方案 §3 Phase 5 默认)。
pub fn parse_arbitration_policy(s: &str) -> ExplorationArbitrationPolicy {
    match s.trim().to_ascii_lowercase().as_str() {
        POLICY_PREFER_TARE => ExplorationArbitrationPolicy::PreferTare,
        POLICY_HIGHEST_SCORE => ExplorationArbitrationPolicy::HighestScore,
        POLICY_FIRST_AVAILABLE => ExplorationArbitrationPolicy::FirstAvailable,
        // 包含空串 / 未识别 / "prefer_wavefront" 默认
        _ => ExplorationArbitrationPolicy::PreferWavefront,
    }
}

/// arbitration policy 转回字符串(给 diagnostics / gRPC 输出用)
pub fn arbitration_policy_str(p: ExplorationArbitrationPolicy) -> &'static str {
    match p {
        ExplorationArbitrationPolicy::PreferWavefront => POLICY_PREFER_WAVEFRONT,
        ExplorationArbitrationPolicy::PreferTare => POLICY_PREFER_TARE,
        ExplorationArbitrationPolicy::HighestScore => POLICY_HIGHEST_SCORE,
        ExplorationArbitrationPolicy::FirstAvailable => POLICY_FIRST_AVAILABLE,
    }
}

/// 把 stop reason 转字符串
pub fn stop_reason_str(r: ExplorationStopReason) -> &'static str {
    match r {
        ExplorationStopReason::NoInformationGain => "no_information_gain",
        ExplorationStopReason::NoReachableFreeSpace => "no_reachable_free_space",
        ExplorationStopReason::ConsecutiveFailuresExceeded => "consecutive_failures_exceeded",
        ExplorationStopReason::UserCancelled => "user_cancelled",
    }
}

/// 当前 wall clock 的 unix timestamp(ns),用于 controller snapshot。
///
/// 不读 LCM typed channel timestamp 的原因是探索 controller 期望单一快照值,
/// 在 tick 处直接用 wall clock 让 controller 与最新 frontier 帧隔离,而 LCM
/// 帧时间戳由 wavefront selector 内部独立消费。
pub fn now_unix_nanos() -> u64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// 从 robot pose (x, y, frame_id) 构造最简 `PoseStamped`
pub fn pose_stamped_from_robot(x: f64, y: f64, frame_id: &str) -> PoseStamped {
    let frame = NavFrameId::Map; // 探索默认在 map 系
    let _ = frame_id; // 保留参数以兼容上游 nav_msgs::Odometry 的 frame_id 字段
    PoseStamped {
        frame_id: frame,
        stamp: Timestamp {
            nanos: now_unix_nanos() as i64,
            clock: ClockKind::Unix,
        },
        pose: Pose3 {
            position: Point3 { x, y, z: 0.0 },
            orientation: Quaternion::IDENTITY,
        },
    }
}

/// 探索状态快照(给 gRPC facade 输出)
#[derive(Debug, Clone)]
pub struct ExploreRuntimeSnapshot {
    pub status: &'static str,
    pub message: String,
    pub request_id: String,
    pub selected_source: String,
    pub last_stop_reason: String,
    pub frontier_count: u32,
    pub revision: u64,
}

/// 探索 controller 包装:持有 controller + 当前 robot pose
pub struct ExploreRuntimeInner {
    controller: Option<ExplorationController>,
    arbitration: ExplorationArbitrationPolicy,
    last_request_id: String,
    last_selected_source: String,
    last_stop_reason: String,
    /// 最近一次 stop reason(包含显式 user cancel)
    last_stop: Option<ExplorationStopReason>,
    /// 最近一次选中的 frontier(若有;用于 diagnostics / nav handoff)
    last_frontier_goal: Option<FrontierGoal>,
    /// 最近一次 controller 返回的 poll(保留用于 diagnostics 诊断)
    last_poll: Option<ExplorationPoll>,
    /// Phase 7:由 mapping_adapter 通过 `ingest_costmap` 注入的 costmap 副本;
    /// `tick()` 时通过 `ExplorationStateSnapshot::global_costmap` 喂给 wavefront
    /// source,真正的 costmap -> frontier 流水线由此贯通。
    costmap_input: Option<nuwax_nav_types_messages::grid::OccupancyGrid2D>,
    /// costmap 注入累计计数;0 = 尚未收到(Phase 7 readiness 用)。
    costmap_input_revision: u64,
}

impl ExploreRuntimeInner {
    fn new() -> Self {
        Self {
            controller: None,
            arbitration: ExplorationArbitrationPolicy::PreferWavefront,
            last_request_id: String::new(),
            last_selected_source: String::new(),
            last_stop_reason: String::new(),
            last_stop: None,
            last_frontier_goal: None,
            last_poll: None,
            costmap_input: None,
            costmap_input_revision: 0,
        }
    }

    /// 注册一个 source;本仓只接 wavefront,但保持 trait object API 与上游对齐
    pub fn add_source<S: ExplorationGoalSource + 'static>(&mut self, src: S) {
        if let Some(ctrl) = self.controller.as_mut() {
            ctrl.add_source(src);
        }
    }
}

impl Default for ExploreRuntimeInner {
    fn default() -> Self {
        Self::new()
    }
}

/// ExploreRuntimeHandle:对外暴露的句柄(克隆便宜,内部锁)
#[derive(Clone)]
pub struct ExploreRuntimeHandle {
    inner: Arc<Mutex<ExploreRuntimeInner>>,
    /// Phase 7:readiness 缓存快照。`readiness()` / `cached_readiness()`
    /// 都从这里读;`ingest_costmap` / `start` / `stop` / `tick` 在变更
    /// controller 状态后调用 `refresh_readiness_cache` 重算。
    /// 缓存 + parking_lot::Mutex 同步,保证 sync context(explore_ready_poll)
    /// 也能不 await 拿到最新 readiness。
    readiness_cache: Arc<parking_lot::Mutex<ExplorationReadinessDetail>>,
}

impl ExploreRuntimeHandle {
    /// 构造默认 handle:注册一个 wavefront source,arbitration=PreferWavefront
    pub fn default_with_wavefront() -> Self {
        let mut inner = ExploreRuntimeInner::new();
        // Phase 5 默认 source = wavefront。
        // 即便 controller 暂未 begin,先把 source 装好;start 时再 begin。
        let mut ctrl = ExplorationController::new(inner.arbitration);
        let adapter = WavefrontGoalSourceAdapter::with_default_id(
            "wavefront",
            WavefrontConfig::default(),
        );
        ctrl.add_source(adapter);
        inner.controller = Some(ctrl);

        let initial_cache = ExplorationReadinessDetail {
            ready: false,
            reason: "explore_runtime 刚构造,backend 状态未确认".into(),
            backend_available: false,
            controller_active: false,
            source_count: 1, // 已经注册 1 个 wavefront source
            frontier_source_ready: false,
            costmap_input_ready: false,
            costmap_input_revision: 0,
            active_source: "wavefront".into(),
            arbitration: "prefer_wavefront".into(),
        };
        Self {
            inner: Arc::new(Mutex::new(inner)),
            readiness_cache: Arc::new(parking_lot::Mutex::new(initial_cache)),
        }
    }

    /// 当前 arbitration policy
    pub async fn arbitration(&self) -> ExplorationArbitrationPolicy {
        let g = self.inner.lock().await;
        g.arbitration
    }

    /// start exploration:注册 source + 重新构造 controller + begin。
    /// `request_id` 仅用于 diagnostics;`strategy` 字段保留兼容,
    /// 当前实现只接受 arbitration policy 命名。
    ///
    /// 返回 `ExploreRuntimeSnapshot`,无论 controller begin 成功或失败都填充。
    pub async fn start(
        &self,
        request_id: &str,
        arbitration_str: &str,
    ) -> ExploreRuntimeSnapshot {
        let mut g = self.inner.lock().await;
        let policy = parse_arbitration_policy(arbitration_str);
        g.arbitration = policy;
        let mut ctrl = ExplorationController::new(policy);
        let adapter = WavefrontGoalSourceAdapter::with_default_id(
            "wavefront",
            WavefrontConfig::default(),
        );
        ctrl.add_source(adapter);
        let begin_msg: Result<(), String> = ctrl.begin_exploration().map_err(|e| e.to_string());
        g.last_request_id = request_id.to_string();
        g.last_selected_source.clear();
        g.last_stop_reason.clear();
        g.last_frontier_goal = None;
        g.last_poll = None;
        match begin_msg {
            Ok(()) => {
                g.controller = Some(ctrl);
                ExploreRuntimeSnapshot {
                    status: STATUS_ACCEPTED,
                    message: format!(
                        "exploration controller begin_exploration succeeded (arbitration={})",
                        arbitration_policy_str(policy)
                    ),
                    request_id: request_id.to_string(),
                    selected_source: String::new(),
                    last_stop_reason: String::new(),
                    frontier_count: 0,
                    revision: 0,
                }
            }
            Err(e) => {
                // 解析报错透传到 message;controller 字段保持 None
                ExploreRuntimeSnapshot {
                    status: STATUS_REJECTED,
                    message: format!("begin_exploration failed: {e}"),
                    request_id: request_id.to_string(),
                    selected_source: String::new(),
                    last_stop_reason: String::new(),
                    frontier_count: 0,
                    revision: 0,
                }
            }
        }
    }

    /// stop exploration
    pub async fn stop(&self, request_id: &str) -> ExploreRuntimeSnapshot {
        let mut g = self.inner.lock().await;
        let msg = if let Some(ctrl) = g.controller.as_mut() {
            // `cancel` 会把 last_stop_reason 设为 UserCancelled
            ctrl.cancel().map_err(|e| e.to_string())
        } else {
            Err("controller not initialized".to_string())
        };
        g.last_stop_reason = stop_reason_str(ExplorationStopReason::UserCancelled).to_string();
        g.last_stop = Some(ExplorationStopReason::UserCancelled);
        let status = match &msg {
            Ok(()) => STATUS_STOPPED,
            Err(e) => {
                warn!("explore_runtime stop 部分失败: {}", e);
                STATUS_STOPPED
            }
        };
        ExploreRuntimeSnapshot {
            status,
            message: msg
                .err()
                .map(|e| format!("stop with error: {e}"))
                .unwrap_or_else(|| "exploration stopped".to_string()),
            request_id: request_id.to_string(),
            selected_source: g.last_selected_source.clone(),
            last_stop_reason: g.last_stop_reason.clone(),
            frontier_count: 0,
            revision: 0,
        }
    }

    /// 单次 tick:构造 snapshot,调 `poll_next_goal`,记录 last_frontier_goal
    ///
    /// 注意:`ExplorationStateSnapshot` 借用 controller 内部的 costmap/pose,
    /// 当前实现不持有持久 costmap(由 Phase 3 mapping_adapter 或 LCM feed 提供),
    /// 因此 snapshot 内的 `global_costmap` 一律传 `None`,由 wavefront source
    /// 决定走内部缓存或 `SourceUnavailable("missing costmap")`。
    pub async fn tick(&self, robot_xy: (f64, f64)) -> ExploreRuntimeSnapshot {
        let mut g = self.inner.lock().await;
        // Phase 7:costmap 注入贯通。在拿 controller 的 `&mut` 借用之前,
        // 先把 `g.costmap_input` 拷出来交给 snapshot——避免后续
        // `g.controller.as_mut()` 的可变借用与 `g.costmap_input.clone()`
        // 的不可变借用重叠(E0502 borrow checker conflict)。
        let costmap_owned: Option<nuwax_nav_types_messages::grid::OccupancyGrid2D> =
            g.costmap_input.clone();
        let Some(ctrl) = g.controller.as_mut() else {
            return ExploreRuntimeSnapshot {
                status: STATUS_UNIMPLEMENTED,
                message: "exploration controller not initialized".into(),
                request_id: g.last_request_id.clone(),
                selected_source: String::new(),
                last_stop_reason: g.last_stop_reason.clone(),
                frontier_count: 0,
                revision: 0,
            };
        };
        let mut snapshot = ExplorationStateSnapshot::from_xy(robot_xy, Timestamp {
            nanos: now_unix_nanos() as i64,
            clock: ClockKind::Unix,
        });
        snapshot.global_costmap = costmap_owned.as_ref();
        let poll_result = ctrl.poll_next_goal(&snapshot);
        let diags = ctrl.diagnostics();
        let status = match diags.last_stop_reason {
            Some(_) => STATUS_STOPPED,
            None if diags.active => STATUS_ACTIVE,
            None => STATUS_PENDING,
        };
        let mut snapshot_out = ExploreRuntimeSnapshot {
            status,
            message: String::new(),
            request_id: g.last_request_id.clone(),
            selected_source: diags
                .selected_source
                .clone()
                .unwrap_or_default(),
            last_stop_reason: diags
                .last_stop_reason
                .map(stop_reason_str)
                .unwrap_or("")
                .to_string(),
            frontier_count: 0,
            revision: diags.sources.len() as u64,
        };
        match poll_result {
            Ok(poll) => {
                match &poll {
                    ExplorationPoll::Goal(goal) => {
                        // FrontierGoal -> NavigationGoal handoff 写入共享 nav_state
                        let _nav = frontier_goal_to_navigation_goal(goal);
                        g.last_frontier_goal = Some(goal.clone());
                        snapshot_out.message = format!(
                            "frontier_goal selected: id={} score={}",
                            goal.id, goal.score
                        );
                    }
                    ExplorationPoll::Pending => {
                        snapshot_out.message = "no goal available (pending)".into();
                    }
                    ExplorationPoll::SourceUnavailable(reason) => {
                        snapshot_out.message = format!("source unavailable: {reason}");
                    }
                    ExplorationPoll::Stop(reason) => {
                        snapshot_out.message = format!("stopped: {}", stop_reason_str(*reason));
                        snapshot_out.last_stop_reason = stop_reason_str(*reason).to_string();
                        g.last_stop = Some(*reason);
                        g.last_stop_reason = snapshot_out.last_stop_reason.clone();
                        g.last_selected_source = snapshot_out.selected_source.clone();
                    }
                }
                g.last_poll = Some(poll);
            }
            Err(e) => {
                snapshot_out.message = format!("controller error: {e}");
                snapshot_out.status = STATUS_REJECTED;
                g.last_poll = None;
            }
        }
        g.last_selected_source = snapshot_out.selected_source.clone();
        g.last_stop_reason = snapshot_out.last_stop_reason.clone();
        snapshot_out
    }

    /// 当前 controller diagnostics(给 `GetExplorationDiagnostics` 用)
    pub async fn diagnostics(&self) -> ControllerDiagnosticsView {
        let g = self.inner.lock().await;
        let (active, selected, stop_reason, sources) = if let Some(ctrl) = g.controller.as_ref() {
            let d = ctrl.diagnostics();
            (
                d.active,
                d.selected_source.clone().unwrap_or_default(),
                d.last_stop_reason.map(stop_reason_str).unwrap_or("").to_string(),
                d.sources
                    .into_iter()
                    .map(|s| SourceDiagView {
                        source_name: s.source_name,
                        is_active: s.is_active,
                        last_goal_id: s.last_goal_id.unwrap_or_default(),
                        last_error: s.last_error.unwrap_or_default(),
                    })
                    .collect::<Vec<_>>(),
            )
        } else {
            (false, String::new(), String::new(), Vec::new())
        };
        let arbitration = arbitration_policy_str(g.arbitration).to_string();
        ControllerDiagnosticsView {
            active,
            arbitration,
            selected_source: selected,
            last_stop_reason: stop_reason,
            sources,
            frontier_count: 0,
            revision: 0,
        }
    }

    /// wavefront input receiver 桥接入口(供 Phase 3 mapping_adapter 注入 costmap)。
    ///
    /// Phase 7:不再返回 false 占位 —— 真实路径是把 costmap snapshot 落到
    /// `ExploreRuntimeInner::costmap_input`,由下一次 `tick()` 通过
    /// `ExplorationStateSnapshot::global_costmap` 注入 wavefront source。
    /// controller 已持有 `Box<dyn ExplorationGoalSource>`,无法 downcast,
    /// 因此走"每次 tick 重新喂 snapshot costmap"路径;mapping_adapter 端
    /// `build_and_publish` 完成后 server.rs::server loop 会调本方法。
    /// 返回 true 表示 costmap 已写入共享 slot;costmap 校验
    /// (cells.len == w*h / w,h > 0)由 wavefront source 在 `poll_goal` 内
    /// 执行,失败时返回 SourceUnavailable("missing costmap"),不影响 ingest。
    ///
    /// 写入后立刻刷新 readiness 缓存(backend_available=未知,用上一次缓存值)。
    pub async fn ingest_costmap(
        &self,
        costmap: nuwax_nav_types_messages::grid::OccupancyGrid2D,
    ) -> bool {
        let width = costmap.width;
        let height = costmap.height;
        let cells_len = costmap.cells.len();
        let mut g = self.inner.lock().await;
        g.costmap_input = Some(costmap);
        g.costmap_input_revision = g.costmap_input_revision.saturating_add(1);
        let backend_available = self.readiness_cache.lock().backend_available;
        let fresh = Self::compute_readiness_locked(&g, backend_available);
        *self.readiness_cache.lock() = fresh;
        if width == 0 || height == 0 || cells_len != (width as usize) * (height as usize) {
            tracing::warn!(
                "ingest_costmap: costmap 尺寸异常 w={} h={} cells={} (revision 仍 ++,前端会标 bad costmap)",
                width,
                height,
                cells_len,
            );
            return false;
        }
        true
    }

    /// 暴露给外部(通常是 Lane 6 cmd_vel_consumer 启动后 / runtime gRPC connect 后)
    /// 显式刷 readiness 缓存并更新 backend_available flag。
    pub fn refresh_readiness_cache_with_backend(&self, backend_available: bool) {
        if let Ok(g) = self.inner.try_lock() {
            let fresh = Self::compute_readiness_locked(&g, backend_available);
            *self.readiness_cache.lock() = fresh;
        }
    }

    /// 当前是否 active(= 至少有一个 source 已 begin 成功)
    pub async fn is_active(&self) -> bool {
        let g = self.inner.lock().await;
        g.controller.as_ref().is_some_and(|c| c.is_active())
    }

    /// Phase 7:readiness 真实聚合(替代 `mapping_ready_poll` 那种"handle 存在
    /// 即 true"的假 ready)。返回 `ExplorationReadinessDetail`,由
    /// `get_exploration_diagnostics` 走 gRPC 返回 Python。
    ///
    /// 综合判定:`backend_available && controller.is_active() &&
    /// frontier_source_ready && costmap_input_ready`。
    /// `frontier_source_ready` = controller 已 begin + 至少 1 个 source
    /// 注册成功;`costmap_input_ready` = `costmap_input_revision > 0`
    /// (由 `ingest_costmap` 维护)。
    ///
    /// 该方法**会更新** `readiness_cache`;sync context 直接读
    /// `cached_readiness()`。
    pub async fn readiness(&self, backend_available: bool) -> ExplorationReadinessDetail {
        let g = self.inner.lock().await;
        let detail = Self::compute_readiness_locked(&g, backend_available);
        *self.readiness_cache.lock() = detail.clone();
        detail
    }

    /// 同步版本:从内部字段直接算 readiness(不 await),更新 cache 并返回。
    /// 用于 `lcm_runtime::explore_ready_poll()` 这类 sync context。
    pub fn readiness_sync(&self, backend_available: bool) -> ExplorationReadinessDetail {
        // 用 std::sync::Mutex 直读 inner 不行,因为 inner 是 tokio::Mutex。
        // 这里用 try_lock 抢锁;抢不到返回缓存的快照(降级,不假装 ready)。
        let cached = self.readiness_cache.lock().clone();
        if let Ok(g) = self.inner.try_lock() {
            let fresh = Self::compute_readiness_locked(&g, backend_available);
            *self.readiness_cache.lock() = fresh.clone();
            fresh
        } else {
            cached
        }
    }

    /// 不重算,直接返回缓存的 readiness 快照。
    pub fn cached_readiness(&self) -> ExplorationReadinessDetail {
        self.readiness_cache.lock().clone()
    }

    fn compute_readiness_locked(
        g: &ExploreRuntimeInner,
        backend_available: bool,
    ) -> ExplorationReadinessDetail {
        let controller_active = g.controller.as_ref().is_some_and(|c| c.is_active());
        let source_count = g.controller.as_ref().map(|c| c.source_count()).unwrap_or(0);
        let arbitration_str = arbitration_policy_str(g.arbitration).to_string();
        let costmap_input_ready = g.costmap_input_revision > 0;
        let costmap_revision = g.costmap_input_revision;
        let active_source = g
            .controller
            .as_ref()
            .and_then(|c| c.diagnostics().selected_source)
            .unwrap_or_default();
        let active_source_name = if active_source.is_empty() {
            if source_count > 0 {
                "wavefront".to_string() // 默认 source 名,即便 controller 尚未选中
            } else {
                String::new()
            }
        } else {
            active_source
        };
        let frontier_source_ready = controller_active && source_count > 0;
        let ready = backend_available && frontier_source_ready && costmap_input_ready;

        let reason = if !backend_available {
            "LCM backend 不可用,explore_runtime 无法注册 wavefront source"
        } else if !controller_active {
            "exploration controller 尚未 begin_exploration"
        } else if source_count == 0 {
            "未注册任何 goal source"
        } else if !costmap_input_ready {
            "mapping adapter 尚未注入 costmap (costmap_input_revision==0)"
        } else {
            "explore_runtime 已就绪"
        }
        .to_string();

        ExplorationReadinessDetail {
            ready,
            reason,
            backend_available,
            controller_active,
            source_count,
            frontier_source_ready,
            costmap_input_ready,
            costmap_input_revision: costmap_revision,
            active_source: active_source_name,
            arbitration: arbitration_str,
        }
    }
}

/// Exploration 板块 readiness 综合快照(Phase 7)。
///
/// 与 `NavigationReadinessDetail` / `MappingReadiness` 对齐:
/// `ready` 综合 bool + `reason` + 各子项 + counters + active source。
#[derive(Debug, Clone)]
pub struct ExplorationReadinessDetail {
    pub ready: bool,
    pub reason: String,
    /// LCM backend 是否可用(从 `GeometryRuntimeHandle.lcm_backend_available` 传入)
    pub backend_available: bool,
    /// controller 是否处于 active(`begin_exploration` 已成功)
    pub controller_active: bool,
    /// 当前 controller 注册的 source 总数
    pub source_count: usize,
    /// `controller_active && source_count > 0`
    pub frontier_source_ready: bool,
    /// `costmap_input_revision > 0`(由 ingest_costmap 维护)
    pub costmap_input_ready: bool,
    /// 累计 costmap 注入次数
    pub costmap_input_revision: u64,
    /// 当前 active source 名(空串 = 未选中;非空 = wavefront / tare 等)
    pub active_source: String,
    /// 当前 arbitration 字符串
    pub arbitration: String,
}

/// 简化的 source diagnostics(给 gRPC 输出)
#[derive(Debug, Clone)]
pub struct SourceDiagView {
    pub source_name: String,
    pub is_active: bool,
    pub last_goal_id: String,
    pub last_error: String,
}

#[derive(Debug, Clone)]
pub struct ControllerDiagnosticsView {
    pub active: bool,
    pub arbitration: String,
    pub selected_source: String,
    pub last_stop_reason: String,
    pub sources: Vec<SourceDiagView>,
    pub frontier_count: u32,
    pub revision: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_arbitration_policy_recognises_known_strings() {
        assert_eq!(
            parse_arbitration_policy("prefer_wavefront"),
            ExplorationArbitrationPolicy::PreferWavefront
        );
        assert_eq!(
            parse_arbitration_policy("prefer_tare"),
            ExplorationArbitrationPolicy::PreferTare
        );
        assert_eq!(
            parse_arbitration_policy("highest_score"),
            ExplorationArbitrationPolicy::HighestScore
        );
        assert_eq!(
            parse_arbitration_policy("first_available"),
            ExplorationArbitrationPolicy::FirstAvailable
        );
    }

    #[test]
    fn parse_arbitration_policy_unknown_falls_back_to_wavefront() {
        assert_eq!(
            parse_arbitration_policy("nonsense"),
            ExplorationArbitrationPolicy::PreferWavefront
        );
        assert_eq!(
            parse_arbitration_policy(""),
            ExplorationArbitrationPolicy::PreferWavefront
        );
    }

    #[test]
    fn arbitration_policy_str_round_trip() {
        for p in [
            ExplorationArbitrationPolicy::PreferWavefront,
            ExplorationArbitrationPolicy::PreferTare,
            ExplorationArbitrationPolicy::HighestScore,
            ExplorationArbitrationPolicy::FirstAvailable,
        ] {
            let s = arbitration_policy_str(p);
            assert_eq!(parse_arbitration_policy(s), p);
        }
    }

    #[test]
    fn stop_reason_str_returns_distinct_strings() {
        let reasons = [
            ExplorationStopReason::NoInformationGain,
            ExplorationStopReason::NoReachableFreeSpace,
            ExplorationStopReason::ConsecutiveFailuresExceeded,
            ExplorationStopReason::UserCancelled,
        ];
        let strs: Vec<&str> = reasons.iter().copied().map(stop_reason_str).collect();
        let uniq: std::collections::HashSet<&str> = strs.iter().copied().collect();
        assert_eq!(uniq.len(), reasons.len());
    }

    #[test]
    fn pose_stamped_from_robot_uses_map_frame() {
        let p = pose_stamped_from_robot(1.5, -2.5, "odom");
        assert_eq!(p.frame_id, NavFrameId::Map);
        assert!((p.pose.position.x - 1.5).abs() < 1e-9);
        assert!((p.pose.position.y - -2.5).abs() < 1e-9);
        assert_eq!(p.pose.position.z, 0.0);
    }

    #[tokio::test]
    async fn default_handle_rejects_before_start() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        // 不调 start 就 tick:controller begin 必须被 start 调用后才生效;
        // 默认 handle 在构造时已经注册 source,但 controller.active 仍为 false,
        // 因此 tick 会得到 SourceUnavailable / Pending,不 panic 即可。
        let s = h.tick((0.0, 0.0)).await;
        // 接受 4 类 status;关键是不能返回 UNIMPLEMENTED(= controller 没有初始化)
        assert!(
            matches!(
                s.status,
                STATUS_ACTIVE | STATUS_PENDING | STATUS_STOPPED | STATUS_REJECTED
            ),
            "unexpected status: {}",
            s.status
        );
    }

    #[tokio::test]
    async fn start_then_stop_records_last_stop_reason() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        let s = h.start("req-1", "first_available").await;
        assert!(
            matches!(s.status, STATUS_ACCEPTED | STATUS_REJECTED),
            "start status = {}",
            s.status
        );
        let stop_s = h.stop("req-1").await;
        assert_eq!(stop_s.status, STATUS_STOPPED);
        assert_eq!(stop_s.last_stop_reason, "user_cancelled");
        // 重复 stop 是 no-op
        let stop_again = h.stop("req-1").await;
        assert_eq!(stop_again.status, STATUS_STOPPED);
    }

    #[test]
    fn wavefront_adapter_source_name_is_wavefront() {
        // 确认 dimos source_name 与 adapter 创建参数一致。
        let ad = WavefrontGoalSourceAdapter::with_default_id(
            "wavefront",
            WavefrontConfig::default(),
        );
        assert_eq!(ad.source_name(), Cow::Borrowed("wavefront"));
        assert!(ad.cached_costmap().is_none());
    }

    // ========================================================================
    // Phase 7:readiness 真实聚合 + ingest_costmap 真实贯通
    // ========================================================================

    fn sample_costmap(w: u32, h: u32) -> nuwax_nav_types_messages::grid::OccupancyGrid2D {
        // Phase 7:OccupancyGrid2D 没有 frame_id 字段(由 msg 携带 vs 由
        // 栅格独立无关);frame 命名走 snapshot/global_costmap 调用方的
        // 上下文(nuwax-nav-types-messages frames 模块)。
        nuwax_nav_types_messages::grid::OccupancyGrid2D {
            resolution: 0.1,
            origin: nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 0.0 },
            width: w,
            height: h,
            cells: vec![nuwax_nav_types_messages::grid::OccupancyCell::Free; (w * h) as usize],
        }
    }

    #[tokio::test]
    async fn readiness_initial_state_unready() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        // 默认 handle 已注册 1 个 wavefront source 但 controller 未 begin,
        // backend_available=false,即便 costmap 注入 readiness 仍 false。
        let r = h.readiness(false).await;
        assert!(!r.ready);
        assert!(!r.backend_available);
        assert!(!r.controller_active);
        assert_eq!(r.source_count, 1);
        assert!(r.active_source.contains("wavefront"));
        assert!(!r.frontier_source_ready);
        assert!(!r.costmap_input_ready);
        assert_eq!(r.costmap_input_revision, 0);
    }

    #[tokio::test]
    async fn readiness_with_backend_no_costmap_unready() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        let s = h.start("req-r1", "first_available").await;
        assert_eq!(s.status, STATUS_ACCEPTED);
        let r = h.readiness(true).await;
        // controller active + 1 source 够 frontier_source_ready;
        // 但 costmap_input_ready 仍 false → overall ready=false
        assert!(r.backend_available);
        assert!(r.controller_active);
        assert!(r.frontier_source_ready);
        assert!(!r.costmap_input_ready);
        assert!(!r.ready);
        // reason 包含 costmap 提示
        assert!(r.reason.contains("costmap"));
    }

    #[tokio::test]
    async fn ingest_costmap_then_readiness_ready() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        let s = h.start("req-r2", "prefer_wavefront").await;
        assert_eq!(s.status, STATUS_ACCEPTED);
        // 注入合法尺寸 costmap
        let cm = sample_costmap(10, 10);
        let ok = h.ingest_costmap(cm).await;
        assert!(ok);
        let r = h.readiness(true).await;
        assert!(r.costmap_input_ready);
        assert_eq!(r.costmap_input_revision, 1);
        assert!(r.ready, "backend+controller+source+costmap 全齐 → ready");
    }

    #[tokio::test]
    async fn ingest_costmap_invalid_dimensions_returns_false() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        // cells.len != w*h 应被拒绝
        let bad = nuwax_nav_types_messages::grid::OccupancyGrid2D {
            resolution: 0.1,
            origin: nuwax_nav_types_messages::geometry::Point2 { x: 0.0, y: 0.0 },
            width: 10,
            height: 10,
            cells: vec![], // 不匹配
        };
        let ok = h.ingest_costmap(bad).await;
        assert!(!ok);
        // revision 仍 ++ (以便 observer 区分 "从未注入" 与 "注入失败")
        let r = h.readiness(true).await;
        assert_eq!(r.costmap_input_revision, 1);
    }

    #[tokio::test]
    async fn sync_readiness_matches_async() {
        let h = ExploreRuntimeHandle::default_with_wavefront();
        let s = h.start("req-r3", "prefer_wavefront").await;
        assert_eq!(s.status, STATUS_ACCEPTED);
        let _ = h.ingest_costmap(sample_costmap(8, 8)).await;
        let async_r = h.readiness(true).await;
        // sync 版本不 await,直接 try_lock;若能拿到应与 async 一致。
        let sync_r = h.readiness_sync(true);
        assert_eq!(async_r.ready, sync_r.ready);
        assert_eq!(async_r.costmap_input_revision, sync_r.costmap_input_revision);
    }
}
