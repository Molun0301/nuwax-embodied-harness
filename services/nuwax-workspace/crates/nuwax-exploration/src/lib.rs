//! `nuwax-exploration`:板块 3 探索 orchestration facade
//!
//! ## 职责
//!
//! - 统一多个 heterogeneous exploration goal source(`Vec<Box<dyn ExplorationGoalSource>>`)
//!   的 lifecycle 与 arbitration;具体 source 由 `nuwax-explore-wavefront` 与
//!   `nuwax-explore-tare-bridge` 通过具体 adapter 提供。
//! - 提供 `FrontierGoal -> NavigationGoal` / `FrontierGoal -> FrontierGoalInput`
//!   的唯一转换入口(`conversions` 模块)。
//! - `runtime` feature 下提供最小 sink contract 与具体 source adapter。
//!
//! ## 不做事项
//!
//! - 不重新定义 `FrontierGoal` / `NavigationGoal` / `FrontierGoalInput` 等强类型。
//! - 不直接管理 TARE / Terrain Analysis 原生进程生命周期(归属各自 `BridgeSupervisor`)。
//! - topic-only `nuwax-explore-terrain-bridge` 不作为 facade dependency,
//!   `terrain_map` topic 由 stack wiring 单独管理。
//!
//! 详见 `docs/子任务/探索模块重写/探索模块重写方案.md` §8。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::borrow::Cow;

use nuwax_nav_base::{NavError, NavResult};
use nuwax_nav_types_messages::geometry::{Point2, PoseStamped, Timestamp};
use nuwax_nav_types_messages::grid::OccupancyGrid2D;
use nuwax_nav_types_messages::visual::FrontierGoal;
use thiserror::Error;

pub mod conversions;

#[cfg(all(feature = "runtime", feature = "tare"))]
pub mod tare_adapter;
#[cfg(all(feature = "runtime", feature = "wavefront"))]
pub mod wavefront_adapter;

#[cfg(feature = "runtime")]
pub mod runtime;

/// Phase E6 板块 3 整合点:typed native spec summary
#[cfg(feature = "spec-summary")]
pub mod spec_summary;
#[cfg(feature = "spec-summary")]
pub use spec_summary::NativeSpecSummary;

/// `nuwax-exploration` facade 操作结果
pub type ExploreResult<T> = Result<T, ExploreError>;

/// `nuwax-exploration` facade 错误
///
/// 按方案 §8.2 最小 enum;来源 / 转换 / native bridge / nav-side 错误统一
/// 收敛为字符串 variant,避免 facade 暴露具体 crate 的 typed error 类型。
#[derive(Debug, Error)]
pub enum ExploreError {
    /// controller 没有注册任何 goal source
    #[error("no exploration goal sources registered")]
    NoSources,
    /// snapshot 中没有 global costmap 且 source adapter 也未缓存
    #[error("missing global costmap for exploration snapshot")]
    MissingCostmap,
    /// controller 已经在 active 状态被请求 begin
    #[error("exploration controller is already active")]
    AlreadyActive,
    /// 具体 source adapter 返回的 source-level 错误(诊断信息透传)
    #[error("exploration source error: {0}")]
    SourceError(String),
    /// 类型 / 数值 / 协议转换错误(如 `TareWaypoint::validate` 失败)
    #[error("exploration conversion error: {0}")]
    Conversion(String),
    /// 导航 / patrolling 侧错误(写入 sink 时透传)
    #[error("exploration nav error: {0}")]
    Nav(String),
    /// boundary 几何无效或不闭合
    #[error("invalid exploration boundary: {0}")]
    InvalidBoundary(String),
}

impl From<ExploreError> for NavResult<()> {
    fn from(err: ExploreError) -> Self {
        Err(NavError::Internal(format!("exploration error: {err}")))
    }
}

/// 探索停止原因
///
/// 方案 §8.2 列出四种。`UserCancelled` 只由 controller / 上层 stop 命令产生,
/// 不来自单个 source 的 internal stop detection。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExplorationStopReason {
    /// 连续无信息增益
    NoInformationGain,
    /// BFS 找不到可达 free cell
    NoReachableFreeSpace,
    /// 连续失败次数超限
    ConsecutiveFailuresExceeded,
    /// 主动取消(用户 / stack / 用户命令)
    UserCancelled,
    /// 已达到请求的覆盖率阈值
    CoverageReached,
    /// 连续无 frontier 超过请求阈值
    NoFrontierTimeout,
    /// 探索总时长超过请求阈值
    DurationExceeded,
    /// 已派发目标数达到请求上限
    MaxGoalsReached,
    /// 被 pin 的地图 revision 在会话中发生变化
    MapRevisionChanged,
}

/// 探索终态类别，避免把自然完成、用户取消和故障压成同一个 STOPPED。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExplorationTerminalKind {
    /// 自然完成，调用方应映射为 succeeded。
    Succeeded,
    /// 用户或上层主动取消。
    Cancelled,
    /// 运行失败或安全条件触发。
    Failed,
}

impl ExplorationStopReason {
    /// 将停止原因分类为稳定的跨层终态。
    #[must_use]
    pub fn terminal_kind(self) -> ExplorationTerminalKind {
        match self {
            Self::NoInformationGain
            | Self::NoReachableFreeSpace
            | Self::CoverageReached
            | Self::NoFrontierTimeout
            | Self::MaxGoalsReached => ExplorationTerminalKind::Succeeded,
            Self::UserCancelled => ExplorationTerminalKind::Cancelled,
            Self::ConsecutiveFailuresExceeded
            | Self::DurationExceeded
            | Self::MapRevisionChanged => ExplorationTerminalKind::Failed,
        }
    }
}

/// child navigation 对 exploration source 的终态反馈。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExplorationGoalFeedback {
    /// child navigation 已到达。
    Reached,
    /// child navigation 失败，保留原始原因用于诊断和 retry/blacklist。
    Failed(String),
    /// child navigation 被取消。
    Cancelled,
}

/// boundary 的一条有向线段。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundarySegment {
    /// 起点。
    pub start: Point2,
    /// 终点。
    pub end: Point2,
}

/// 闭合探索边界。边界上的点视为范围内。
#[derive(Debug, Clone, PartialEq)]
pub struct ExplorationBoundary {
    vertices: Vec<Point2>,
}

impl ExplorationBoundary {
    /// 从按顺序排列的顶点构造闭合 polygon，末点无需重复首点。
    pub fn from_vertices(vertices: Vec<Point2>) -> ExploreResult<Self> {
        if vertices.len() < 3 {
            return Err(ExploreError::InvalidBoundary(
                "at least three vertices are required".to_string(),
            ));
        }
        if vertices
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(ExploreError::InvalidBoundary(
                "vertices must be finite".to_string(),
            ));
        }
        for index in 0..vertices.len() {
            let next = (index + 1) % vertices.len();
            if point_distance(vertices[index], vertices[next]) <= 1.0e-9 {
                return Err(ExploreError::InvalidBoundary(
                    "zero-length boundary segment".to_string(),
                ));
            }
        }
        let (min_x, max_x, min_y, max_y) = vertices.iter().fold(
            (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ),
            |(min_x, max_x, min_y, max_y), point| {
                (
                    min_x.min(point.x),
                    max_x.max(point.x),
                    min_y.min(point.y),
                    max_y.max(point.y),
                )
            },
        );
        let scale = (max_x - min_x).max(max_y - min_y).max(1.0);
        for first in 0..vertices.len() {
            let first_next = (first + 1) % vertices.len();
            for second in (first + 1)..vertices.len() {
                let second_next = (second + 1) % vertices.len();
                if first == second_next || first_next == second {
                    continue;
                }
                if segments_intersect(
                    vertices[first],
                    vertices[first_next],
                    vertices[second],
                    vertices[second_next],
                ) {
                    return Err(ExploreError::InvalidBoundary(
                        "boundary polygon must not self-intersect".to_string(),
                    ));
                }
            }
        }
        let twice_area = vertices
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let next = vertices[(index + 1) % vertices.len()];
                point.x * next.y - next.x * point.y
            })
            .sum::<f64>();
        if twice_area.abs() <= 1.0e-9 * scale * scale {
            return Err(ExploreError::InvalidBoundary(
                "boundary polygon area is zero or numerically degenerate".to_string(),
            ));
        }
        Ok(Self { vertices })
    }

    /// 从连续且闭合的线段构造 polygon。
    pub fn from_segments(segments: Vec<BoundarySegment>) -> ExploreResult<Self> {
        if segments.len() < 3 {
            return Err(ExploreError::InvalidBoundary(
                "at least three segments are required".to_string(),
            ));
        }
        for index in 0..segments.len() {
            let next = (index + 1) % segments.len();
            if point_distance(segments[index].end, segments[next].start) > 1.0e-6 {
                return Err(ExploreError::InvalidBoundary(
                    "segments must be ordered, contiguous, and closed".to_string(),
                ));
            }
        }
        Self::from_vertices(segments.into_iter().map(|segment| segment.start).collect())
    }

    /// polygon 顶点只读视图。
    #[must_use]
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// 判断点是否在 polygon 内部或边界上。
    #[must_use]
    pub fn contains(&self, point: Point2) -> bool {
        let mut inside = false;
        let mut previous = self.vertices.len() - 1;
        for current in 0..self.vertices.len() {
            let a = self.vertices[previous];
            let b = self.vertices[current];
            if point_on_segment(point, a, b) {
                return true;
            }
            let crosses = (a.y > point.y) != (b.y > point.y)
                && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
            if crosses {
                inside = !inside;
            }
            previous = current;
        }
        inside
    }
}

fn point_distance(a: Point2, b: Point2) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn point_on_segment(point: Point2, a: Point2, b: Point2) -> bool {
    let cross = (point.y - a.y) * (b.x - a.x) - (point.x - a.x) * (b.y - a.y);
    if cross.abs() > 1.0e-8 {
        return false;
    }
    let dot = (point.x - a.x) * (b.x - a.x) + (point.y - a.y) * (b.y - a.y);
    dot >= -1.0e-8 && dot <= point_distance(a, b).powi(2) + 1.0e-8
}

fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let orientation =
        |p: Point2, q: Point2, r: Point2| (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    let ab_c = orientation(a, b, c);
    let ab_d = orientation(a, b, d);
    let cd_a = orientation(c, d, a);
    let cd_b = orientation(c, d, b);
    let epsilon = 1.0e-9 * point_distance(a, b).max(point_distance(c, d)).max(1.0);

    if ((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon))
        && ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon))
    {
        return true;
    }
    (ab_c.abs() <= epsilon && point_on_segment(c, a, b))
        || (ab_d.abs() <= epsilon && point_on_segment(d, a, b))
        || (cd_a.abs() <= epsilon && point_on_segment(a, c, d))
        || (cd_b.abs() <= epsilon && point_on_segment(b, c, d))
}

/// controller 单次 `poll_next_goal` 输出
///
/// 按方案 §8.2:`Goal / Pending / Stop / SourceUnavailable`。
/// 不再以 `Option<FrontierGoal>` 隐式表达 pending / no-gain / stop。
#[derive(Debug, Clone, PartialEq)]
pub enum ExplorationPoll {
    /// 当前 source 已经选择出 frontier goal
    Goal(FrontierGoal),
    /// 当前 source 暂时没有可用 goal(`HighestScore` 与 `FirstAvailable` 之下
    /// 会继续 fallback 或继续扫描其它 source)
    Pending,
    /// 当前 source / controller 发出停止信号;controller 立即结束本轮 exploration,
    /// 释放 active,不 fallback 到其它 source
    Stop(ExplorationStopReason),
    /// 当前 source 当前不可用(包含错误诊断);按 pending 处理,policy 可 fallback
    SourceUnavailable(String),
}

/// controller 与 source adapter 之间共享的 explore-time 状态快照
///
/// lifetime `'a` 用于零拷贝引用 `OccupancyGrid2D`;caller 既可以传入
/// `ExplorationStateSnapshot.global_costmap`,也可以传 `None`,由具体 source
/// adapter 决定是否使用其内部缓存(`WavefrontInputReceiver::ingest_costmap` 等)。
#[derive(Debug, Clone)]
pub struct ExplorationStateSnapshot<'a> {
    /// 机器人当前位姿
    pub robot_pose: PoseStamped,
    /// 全局 costmap(若有)
    pub global_costmap: Option<&'a OccupancyGrid2D>,
    /// controller 当前已发布但尚未到达的 navigation goal
    pub active_goal: Option<nuwax_nav_types_messages::goal::NavigationGoal>,
    /// 最近选出的 frontier goal(用于 cancel / dedup)
    pub latest_frontier_goal: Option<FrontierGoal>,
    /// 当前时间戳;Wavefront selector 必须使用此值而不是 wall clock
    pub now: Timestamp,
}

impl<'a> ExplorationStateSnapshot<'a> {
    /// 构造最简 snapshot(无 costmap / 无 active goal)
    #[must_use]
    pub fn new(robot_pose: PoseStamped, now: Timestamp) -> Self {
        Self {
            robot_pose,
            global_costmap: None,
            active_goal: None,
            latest_frontier_goal: None,
            now,
        }
    }

    /// 从已有 robot position `(x, y)` 构造最简 snapshot(position 由 helper 转 `PoseStamped`)
    #[must_use]
    pub fn from_xy(robot_xy: (f64, f64), now: Timestamp) -> Self {
        use nuwax_nav_types_messages::frames::NavFrameId;
        use nuwax_nav_types_messages::geometry::{Point3, Pose3, Quaternion};
        let pose = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: now,
            pose: Pose3 {
                position: Point3 {
                    x: robot_xy.0,
                    y: robot_xy.1,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        Self::new(pose, now)
    }

    /// 获取 robot position 作为 `Point2`(供 source adapter 调用 `world_to_cell`)
    #[must_use]
    pub fn robot_position(&self) -> Point2 {
        Point2 {
            x: self.robot_pose.pose.position.x,
            y: self.robot_pose.pose.position.y,
        }
    }
}

/// 单个 exploration goal source diagnostics
#[derive(Debug, Clone, PartialEq)]
pub struct ExplorationSourceDiagnostics {
    /// source name
    pub source_name: String,
    /// 当前是否已 begin
    pub is_active: bool,
    /// 最近选出的 frontier goal id(若有)
    pub last_goal_id: Option<String>,
    /// 最近一次 source error(若有,来自 `SourceUnavailable` 或 `SourceError`)
    pub last_error: Option<String>,
}

/// controller 整体 diagnostics
#[derive(Debug, Clone, PartialEq)]
pub struct ExplorationControllerDiagnostics {
    /// 当前 controller 是否 active
    pub active: bool,
    /// 最近一次选中的 source name
    pub selected_source: Option<String>,
    /// 最近一次 stop reason(若有;包含主动 `end_exploration` 时设为 `UserCancelled`)
    pub last_stop_reason: Option<ExplorationStopReason>,
    /// 每个 source 的 diagnostics 列表
    pub sources: Vec<ExplorationSourceDiagnostics>,
}

/// exploration goal source trait
///
/// 同步 arbitration contract,便于 pure unit test。
/// `runtime` feature 下新增 async task handles 可桥接到具体 sync source 上。
pub trait ExplorationGoalSource: Send {
    /// source name(多实例时必须唯一;duplicate name 的 tie-break 由 controller 固定)
    fn source_name(&self) -> Cow<'static, str>;

    /// 启动 source;失败时 controller 按 reverse-order rollback 已 begin 的 sources
    fn begin(&mut self) -> ExploreResult<()>;

    /// 关闭 source;cancel / shutdown 时调用;failure 仅记录到 diagnostics
    fn end(&mut self) -> ExploreResult<()>;

    /// controller 单次轮询;按方案 §8.2 arbitration matrix 返回 `ExplorationPoll`
    fn poll_goal(&mut self, state: &ExplorationStateSnapshot<'_>)
        -> ExploreResult<ExplorationPoll>;

    /// 将 child navigation 终态反馈回 source。无状态 source 可使用默认 no-op。
    fn report_goal_feedback(
        &mut self,
        _goal_id: &str,
        _feedback: ExplorationGoalFeedback,
    ) -> ExploreResult<()> {
        Ok(())
    }

    /// source 当前可发布的 frontier 快照。默认 source 没有 frontier 视图。
    fn frontier_snapshot(&self) -> Vec<FrontierGoal> {
        Vec::new()
    }

    /// 当前 source diagnostics(只读)
    fn diagnostics(&self) -> ExplorationSourceDiagnostics;
}

/// source arbitration policy
///
/// 方案 §8.2 矩阵:`Goal` 可选;`Pending` 可 fallback(`Prefer*` / `FirstAvailable`);
/// `SourceUnavailable` 视 pending 处理;`Stop` 立即结束。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExplorationArbitrationPolicy {
    /// 优先 wavefront;pending / unavailable 时 fallback 到其它 source
    PreferWavefront,
    /// 优先 TARE;pending / unavailable 时 fallback 到其它 source
    PreferTare,
    /// 在所有 source 中选 score 最高的 `Goal`;无 `Goal` 时返回 `Pending`
    HighestScore,
    /// 顺序扫描,返回第一个 non-pending 的 `Goal`
    FirstAvailable,
}

/// exploration controller:多 source 仲裁 + 统一 lifecycle / diagnostics
///
/// 标准 controller 持有 `Vec<Box<dyn ExplorationGoalSource>>`,以支持
/// heterogeneous source(纯算法 + native bridge adapter)。
pub struct ExplorationController {
    sources: Vec<Box<dyn ExplorationGoalSource>>,
    arbitration: ExplorationArbitrationPolicy,
    active: bool,
    last_stop_reason: Option<ExplorationStopReason>,
    last_selected: Option<String>,
    last_source_diag: Vec<ExplorationSourceDiagnostics>,
}

impl ExplorationController {
    /// 构造 controller
    #[must_use]
    pub fn new(arbitration: ExplorationArbitrationPolicy) -> Self {
        Self {
            sources: Vec::new(),
            arbitration,
            active: false,
            last_stop_reason: None,
            last_selected: None,
            last_source_diag: Vec::new(),
        }
    }

    /// 当前 arbitration policy
    #[must_use]
    pub fn arbitration(&self) -> ExplorationArbitrationPolicy {
        self.arbitration
    }

    /// 是否 active
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// 注册一个具体 source(`'static` 边界使其可被 trait object 持有)
    pub fn add_source<T>(&mut self, source: T)
    where
        T: ExplorationGoalSource + 'static,
    {
        self.sources.push(Box::new(source));
    }

    /// 注册任意满足 trait object 的 source(便于 trait-object-only 调用方)
    pub fn add_source_boxed(&mut self, source: Box<dyn ExplorationGoalSource>) {
        self.sources.push(source);
    }

    /// 当前 source 数量
    #[must_use]
    pub fn source_count(&self) -> usize {
        self.sources.len()
    }

    /// 启动 exploration。
    ///
    /// 行为:按 insertion order 依次 `begin()` 每个 source;若第 `k` 个 source 失败,
    /// 则按 reverse-order 调用前 `k-1` 个 source 的 `end()` 释放状态,清 `active`,
    /// 返回 `SourceError(reason)`。
    ///
    /// `active == true` 时返回 `AlreadyActive`(保持状态)。
    pub fn begin_exploration(&mut self) -> ExploreResult<()> {
        if self.active {
            return Err(ExploreError::AlreadyActive);
        }
        if self.sources.is_empty() {
            return Err(ExploreError::NoSources);
        }

        let mut begun: Vec<usize> = Vec::with_capacity(self.sources.len());
        for (idx, source) in self.sources.iter_mut().enumerate() {
            match source.begin() {
                Ok(()) => begun.push(idx),
                Err(e) => {
                    // rollback in reverse order
                    for &b_idx in begun.iter().rev() {
                        let _ = self.sources[b_idx].end();
                    }
                    self.active = false;
                    return Err(ExploreError::SourceError(format!(
                        "source `{}` begin failed: {}",
                        self.sources[idx].source_name(),
                        e
                    )));
                }
            }
        }
        self.active = true;
        self.last_stop_reason = None;
        Ok(())
    }

    /// 结束 exploration;`active == false` 时是 no-op(返回 `Ok(())`),
    /// 但同时记录 `last_stop_reason = Some(UserCancelled)`,便于 diagnostics。
    pub fn end_exploration(&mut self) -> ExploreResult<()> {
        self.stop_with_reason(ExplorationStopReason::UserCancelled)
    }

    /// 以明确原因结束 exploration，并按 reverse order 释放全部 source。
    pub fn stop_with_reason(&mut self, reason: ExplorationStopReason) -> ExploreResult<()> {
        if self.active {
            for source in self.sources.iter_mut().rev() {
                let _ = source.end();
            }
            self.active = false;
        }
        self.last_stop_reason = Some(reason);
        Ok(())
    }

    /// 强制结束(由上层 / user cancel 触发);与 `end_exploration` 等价但语义更明确
    pub fn cancel(&mut self) -> ExploreResult<()> {
        self.end_exploration()
    }

    /// 关闭 controller 并丢弃所有 source(state)
    pub fn shutdown(&mut self) {
        if self.active {
            for source in self.sources.iter_mut().rev() {
                let _ = source.end();
            }
            self.active = false;
        }
        self.sources.clear();
        self.last_selected = None;
        self.last_source_diag.clear();
    }

    /// 单次 arbitration:返回下一个 `ExplorationPoll`。
    ///
    /// `active == false` 时也会被允许调用(返回 `SourceUnavailable("not_active")`);
    /// 这样上层可以随时 poll diagnostics 而不强制 begin。
    pub fn poll_next_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<ExplorationPoll> {
        if !self.active {
            return Ok(ExplorationPoll::SourceUnavailable(
                "controller not active".to_string(),
            ));
        }

        self.last_source_diag.clear();

        let order = self.arbitration_order();
        let mut best_goal: Option<(f32, usize, FrontierGoal)> = None;
        let mut any_pending = false;
        let mut any_unavailable = false;
        let mut last_unavailable: Option<String> = None;

        for &idx in &order {
            let source_name = self.sources[idx].source_name();
            let poll = match self.sources[idx].poll_goal(state) {
                Ok(p) => p,
                Err(e) => {
                    any_unavailable = true;
                    last_unavailable = Some(format!("{source_name}: {e}"));
                    self.last_source_diag.push(diag_for(
                        &*self.sources[idx],
                        None,
                        Some(format!("{e}")),
                    ));
                    continue;
                }
            };
            match poll {
                ExplorationPoll::Goal(goal) => {
                    // HighestScore 比较;其它 policy 取首个
                    match self.arbitration {
                        ExplorationArbitrationPolicy::HighestScore => {
                            let score = goal.score;
                            let replace = match &best_goal {
                                None => true,
                                Some((best, _, _)) => score > *best,
                            };
                            if replace {
                                best_goal = Some((score, idx, goal.clone()));
                                self.last_selected = Some(source_name.to_string());
                                self.last_source_diag.push(diag_for(
                                    &*self.sources[idx],
                                    Some(goal.id.clone()),
                                    None,
                                ));
                            }
                        }
                        _ => {
                            self.last_selected = Some(source_name.to_string());
                            self.last_source_diag.push(diag_for(
                                &*self.sources[idx],
                                Some(goal.id.clone()),
                                None,
                            ));
                            return Ok(ExplorationPoll::Goal(goal));
                        }
                    }
                }
                ExplorationPoll::Pending => {
                    any_pending = true;
                    self.last_source_diag
                        .push(diag_for(&*self.sources[idx], None, None));
                }
                ExplorationPoll::SourceUnavailable(reason) => {
                    any_unavailable = true;
                    last_unavailable = Some(reason);
                    self.last_source_diag.push(diag_for(
                        &*self.sources[idx],
                        None,
                        last_unavailable.clone(),
                    ));
                }
                ExplorationPoll::Stop(reason) => {
                    // Stop 立即结束,不 fallback
                    self.last_selected = Some(source_name.to_string());
                    self.last_stop_reason = Some(reason);
                    for source in self.sources.iter_mut().rev() {
                        let _ = source.end();
                    }
                    self.active = false;
                    return Ok(ExplorationPoll::Stop(reason));
                }
            }
        }

        if let Some((_, idx, goal)) = best_goal {
            let _ = idx;
            return Ok(ExplorationPoll::Goal(goal));
        }

        if any_unavailable {
            if any_pending {
                // 既有 unavailable 也有 pending;按方案 unavailable 也按 pending 处理
                Ok(ExplorationPoll::Pending)
            } else {
                Ok(ExplorationPoll::SourceUnavailable(
                    last_unavailable.unwrap_or_else(|| "unknown".to_string()),
                ))
            }
        } else if any_pending {
            Ok(ExplorationPoll::Pending)
        } else {
            // 没有任何 source 被询问(空 sources 情况下 begin 已被 NoSources 截断;此处兜底)
            Ok(ExplorationPoll::Pending)
        }
    }

    /// 将 child navigation 反馈广播到 source；各 source 按 goal id 自行过滤。
    pub fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> ExploreResult<()> {
        for source in &mut self.sources {
            source.report_goal_feedback(goal_id, feedback.clone())?;
        }
        Ok(())
    }

    /// 合并所有 source 当前 frontier 快照。
    #[must_use]
    pub fn frontier_snapshot(&self) -> Vec<FrontierGoal> {
        self.sources
            .iter()
            .flat_map(|source| source.frontier_snapshot())
            .collect()
    }

    /// controller diagnostics
    #[must_use]
    pub fn diagnostics(&self) -> ExplorationControllerDiagnostics {
        let sources = if self.last_source_diag.is_empty() {
            self.sources
                .iter()
                .map(|s| {
                    let d = s.diagnostics();
                    ExplorationSourceDiagnostics {
                        source_name: d.source_name,
                        is_active: d.is_active,
                        last_goal_id: d.last_goal_id,
                        last_error: d.last_error,
                    }
                })
                .collect()
        } else {
            self.last_source_diag.clone()
        };
        ExplorationControllerDiagnostics {
            active: self.active,
            selected_source: self.last_selected.clone(),
            last_stop_reason: self.last_stop_reason,
            sources,
        }
    }

    /// 按 arbitration policy 计算 source 顺序。
    ///
    /// - `PreferWavefront` / `PreferTare`:同名 source 不存在时退化为 source insertion order。
    ///   否则 preferred source 优先,其余按 insertion order。
    /// - `HighestScore` / `FirstAvailable`:按 insertion order(全部 source 平等)。
    ///
    /// duplicate source name 时由插入顺序保留 deterministic tie-break。
    pub fn arbitration_order(&self) -> Vec<usize> {
        let mut order: Vec<usize> = (0..self.sources.len()).collect();
        match self.arbitration {
            ExplorationArbitrationPolicy::PreferWavefront
            | ExplorationArbitrationPolicy::PreferTare => {
                let preferred = match self.arbitration {
                    ExplorationArbitrationPolicy::PreferWavefront => "wavefront",
                    ExplorationArbitrationPolicy::PreferTare => "tare",
                    _ => unreachable!(),
                };
                order.sort_by_key(|&idx| {
                    let name = self.sources[idx].source_name();
                    if name.eq_ignore_ascii_case(preferred) {
                        0u8
                    } else {
                        1u8
                    }
                });
                // stable_sort 保留相同 key 内 insertion order
                order.sort_by_key(|&idx| {
                    let name = self.sources[idx].source_name();
                    if name.eq_ignore_ascii_case(preferred) {
                        0u8
                    } else {
                        1u8
                    }
                });
            }
            ExplorationArbitrationPolicy::HighestScore
            | ExplorationArbitrationPolicy::FirstAvailable => {
                // 已按 insertion order
            }
        }
        order
    }
}

/// 内部 helper:把 source-level diagnostics(只读)转 facade 形式的可变拷贝
fn diag_for(
    source: &dyn ExplorationGoalSource,
    last_goal_id: Option<String>,
    last_error: Option<String>,
) -> ExplorationSourceDiagnostics {
    let base = source.diagnostics();
    ExplorationSourceDiagnostics {
        source_name: base.source_name,
        is_active: base.is_active,
        last_goal_id: last_goal_id.or(base.last_goal_id),
        last_error: last_error.or(base.last_error),
    }
}

/// 内部 helper:把 source-side `StopReason` 映射到 facade `ExplorationStopReason`
///
/// source adapter 负责把 wavefront `StopReason` 在自己模块内转为 facade enum,
/// 不在本 crate 持有 wavefront internals。函数保留以便上层消费。
/// wavefront crate 的 `StopReason -> ExplorationStopReason` 映射由
/// `crate::wavefront_adapter` 模块提供。
fn _facade_stop_marker() {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Debug)]
    struct DummySource {
        name: Cow<'static, str>,
        active: bool,
        diag: ExplorationSourceDiagnostics,
    }

    impl DummySource {
        fn new(name: &'static str) -> Self {
            Self {
                name: Cow::Borrowed(name),
                active: false,
                diag: ExplorationSourceDiagnostics {
                    source_name: name.to_string(),
                    is_active: false,
                    last_goal_id: None,
                    last_error: None,
                },
            }
        }
    }

    impl ExplorationGoalSource for DummySource {
        fn source_name(&self) -> Cow<'static, str> {
            self.name.clone()
        }
        fn begin(&mut self) -> ExploreResult<()> {
            self.active = true;
            self.diag.is_active = true;
            Ok(())
        }
        fn end(&mut self) -> ExploreResult<()> {
            self.active = false;
            self.diag.is_active = false;
            Ok(())
        }
        fn poll_goal(
            &mut self,
            _state: &ExplorationStateSnapshot<'_>,
        ) -> ExploreResult<ExplorationPoll> {
            Ok(ExplorationPoll::Pending)
        }
        fn diagnostics(&self) -> ExplorationSourceDiagnostics {
            self.diag.clone()
        }
    }

    #[test]
    fn arbitration_order_prefers_wavefront() {
        let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        ctrl.add_source(DummySource::new("tare"));
        ctrl.add_source(DummySource::new("wavefront"));
        let order = ctrl.arbitration_order();
        assert_eq!(order.first().copied(), Some(1));
    }

    #[test]
    fn arbitration_order_first_available_keeps_insertion_order() {
        let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        ctrl.add_source(DummySource::new("tare"));
        ctrl.add_source(DummySource::new("wavefront"));
        let order = ctrl.arbitration_order();
        assert_eq!(order, vec![0, 1]);
    }

    #[test]
    fn begin_no_sources_returns_error() {
        let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        let err = ctrl.begin_exploration().unwrap_err();
        assert!(matches!(err, ExploreError::NoSources));
    }

    #[test]
    fn begin_already_active_returns_already_active_error() {
        let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        ctrl.add_source(DummySource::new("wavefront"));
        ctrl.begin_exploration().unwrap();
        let err = ctrl.begin_exploration().unwrap_err();
        assert!(matches!(err, ExploreError::AlreadyActive));
    }

    #[test]
    fn end_when_inactive_is_noop() {
        let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        // inactive -> no-op
        ctrl.end_exploration().unwrap();
        assert!(!ctrl.is_active());
        assert_eq!(
            ctrl.diagnostics().last_stop_reason,
            Some(ExplorationStopReason::UserCancelled)
        );
    }

    #[test]
    fn stop_reasons_preserve_success_cancel_and_failure_semantics() {
        assert_eq!(
            ExplorationStopReason::NoInformationGain.terminal_kind(),
            ExplorationTerminalKind::Succeeded
        );
        assert_eq!(
            ExplorationStopReason::UserCancelled.terminal_kind(),
            ExplorationTerminalKind::Cancelled
        );
        assert_eq!(
            ExplorationStopReason::ConsecutiveFailuresExceeded.terminal_kind(),
            ExplorationTerminalKind::Failed
        );
    }

    #[test]
    fn closed_boundary_contains_inside_and_edge_but_not_outside() {
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 4.0, y: 0.0 },
            Point2 { x: 4.0, y: 4.0 },
            Point2 { x: 0.0, y: 4.0 },
        ])
        .unwrap();

        assert!(boundary.contains(Point2 { x: 2.0, y: 2.0 }));
        assert!(boundary.contains(Point2 { x: 4.0, y: 2.0 }));
        assert!(!boundary.contains(Point2 { x: 5.0, y: 2.0 }));
    }

    #[test]
    fn boundary_rejects_self_intersection_and_zero_area() {
        let bow_tie = ExplorationBoundary::from_vertices(vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 2.0, y: 2.0 },
            Point2 { x: 0.0, y: 2.0 },
            Point2 { x: 2.0, y: 0.0 },
        ])
        .unwrap_err();
        assert!(bow_tie.to_string().contains("self-intersect"));

        let collinear = ExplorationBoundary::from_vertices(vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 1.0, y: 0.0 },
            Point2 { x: 2.0, y: 0.0 },
        ])
        .unwrap_err();
        assert!(collinear.to_string().contains("area is zero"));
    }

    #[test]
    fn boundary_segments_still_require_contiguous_closed_order() {
        let error = ExplorationBoundary::from_segments(vec![
            BoundarySegment {
                start: Point2 { x: 0.0, y: 0.0 },
                end: Point2 { x: 2.0, y: 0.0 },
            },
            BoundarySegment {
                start: Point2 { x: 2.0, y: 1.0 },
                end: Point2 { x: 1.0, y: 2.0 },
            },
            BoundarySegment {
                start: Point2 { x: 1.0, y: 2.0 },
                end: Point2 { x: 0.0, y: 0.0 },
            },
        ])
        .unwrap_err();
        assert!(error.to_string().contains("contiguous"));
    }

    struct FeedbackSource {
        feedback: Arc<Mutex<Vec<(String, ExplorationGoalFeedback)>>>,
    }

    impl ExplorationGoalSource for FeedbackSource {
        fn source_name(&self) -> Cow<'static, str> {
            Cow::Borrowed("wavefront")
        }

        fn begin(&mut self) -> ExploreResult<()> {
            Ok(())
        }

        fn end(&mut self) -> ExploreResult<()> {
            Ok(())
        }

        fn poll_goal(
            &mut self,
            _state: &ExplorationStateSnapshot<'_>,
        ) -> ExploreResult<ExplorationPoll> {
            Ok(ExplorationPoll::Pending)
        }

        fn report_goal_feedback(
            &mut self,
            goal_id: &str,
            feedback: ExplorationGoalFeedback,
        ) -> ExploreResult<()> {
            self.feedback
                .lock()
                .unwrap()
                .push((goal_id.to_string(), feedback));
            Ok(())
        }

        fn diagnostics(&self) -> ExplorationSourceDiagnostics {
            ExplorationSourceDiagnostics {
                source_name: "wavefront".to_string(),
                is_active: true,
                last_goal_id: None,
                last_error: None,
            }
        }
    }

    #[test]
    fn controller_routes_child_navigation_feedback_to_sources() {
        let feedback = Arc::new(Mutex::new(Vec::new()));
        let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        ctrl.add_source(FeedbackSource {
            feedback: feedback.clone(),
        });
        ctrl.begin_exploration().unwrap();

        ctrl.report_goal_feedback("child-1", ExplorationGoalFeedback::Reached)
            .unwrap();

        assert_eq!(
            feedback.lock().unwrap().as_slice(),
            &[("child-1".to_string(), ExplorationGoalFeedback::Reached)]
        );
    }
}
