//! Guard-backed, single-owner navigation scheduler core.
//!
//! This crate deliberately stops at the process-neutral scheduling boundary.
//! It owns one mutable motion port, serializes lease renewal and commands, and
//! accepts only lease-correlated paths. Map sequence fields may still be
//! carried by compatibility transports, but they do not participate in motion.

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nuwax_motion_safety_guard::wire::{ErrorCode, Source};
use nuwax_motion_safety_guard::{
    ClientConfig, MotionGuardClient, MotionGuardClientError,
};
use nuwax_nav_astar::local_planner::{LocalPlanner, LocalPlannerConfig, StopMessage};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{GoalKey, Path2D, Path3D, PoseStamped, Twist2};
use thiserror::Error;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;
use tokio::time::{interval_at, Instant, MissedTickBehavior};

pub use nuwax_motion_safety_guard::MeasuredMotion;
/// 保留来源时点的实测运动边界转换。
pub mod motion_feedback;
/// 同帧注册点云原始输入，不是融合地图。
pub mod registered_frame;
pub use registered_frame::{RegisteredFrame, RegisteredFrameDomain, RegisteredFrameIdentity};
mod command_dispatch;
pub use command_dispatch::CommandDispatchBound;
/// Explicit elapsed-time observer seam for bounded recovery qualifications.
#[cfg(feature = "surface-recovery-qualification")]
pub mod qualification_timing;
/// Typed actual execution feedback for deduplicating identical rejected work.
pub mod replan_feedback;
use replan_feedback::{ActualExecutionContext, SurfaceReplanFeedback};
mod planner_worker;
pub mod surface_follower;
pub use nuwax_nav_mls_planner::SurfaceSnapshot;
pub use surface_follower::{SurfaceFollower, SurfaceFollowerConfig};
/// Shared ON/OFF source-change proofs.
pub mod surface_changes;
/// 原生地图输入适配。
pub mod native_map;
/// 三维路线与局部执行共用同一个原生内核拥有者。
pub type RouteFollower = SurfaceFollower;
#[cfg(test)]
mod native_test_helpers;
#[cfg(test)]
mod control_profile;
#[cfg(test)]
mod command_proof_trace;

/// 平台机体参考目标。
#[derive(Debug, Clone, PartialEq)]
pub enum ExecutionGoal {
    /// 完整机体位置和姿态。
    Body(PoseStamped),
}
impl ExecutionGoal {
    /// 读取机体目标。
    pub fn as_body(&self) -> Option<&PoseStamped> { let Self::Body(pose)=self; Some(pose) }
    /// 修改机体目标。
    pub fn as_body_mut(&mut self) -> Option<&mut PoseStamped> { let Self::Body(pose)=self; Some(pose) }
    /// 比较原机体目标的实际几何，忽略传输时刻。
    pub fn same_geometry(&self, other:&Self)->bool {
        replan_feedback::same_pose(self.as_body().unwrap(),other.as_body().unwrap())
    }
    pub(crate) fn validate(&self)->Result<(),RuntimeError> { validate_map_pose(self.as_body().unwrap(),"goal_pose") }
}

/// 局部执行保留路径维度，三维路径不能通过默认适配静默丢弃高度。
#[derive(Debug, Clone, PartialEq)]
pub enum ExecutionPath {
    /// 现有二维路径，配套朝向沿原可选字段传递。
    Planar(Path2D),
    /// 完整机体参考位置与姿态路径。
    Surface(Path3D),
}

// Resend timestamps do not change the route actually consumed by a follower.
// Preserve all consumed coordinates, orientations, frames and dimensionality.
fn same_execution_path(left: &ExecutionPath, right: &ExecutionPath) -> bool {
    match (left, right) {
        (ExecutionPath::Planar(left), ExecutionPath::Planar(right)) =>
            left.frame_id == right.frame_id && left.points == right.points,
        (ExecutionPath::Surface(left), ExecutionPath::Surface(right)) =>
            left.poses.len() == right.poses.len() && left.poses.iter().zip(&right.poses)
                .all(|(left, right)| left.frame_id == right.frame_id && left.pose == right.pose),
        _ => false,
    }
}

/// 当前局部执行使用的不可变地图输入。
#[derive(Clone)]
pub enum ExecutionMap {
    /// 现有二维占据图。
    Planar(CostMap2D),
    /// 由同一MLS拥有方生成的不可变表面快照。
    Surface(Arc<SurfaceSnapshot>),
    /// Immutable surface plus complete differences from its explicit baseline.
    SurfaceUpdate(surface_changes::SurfaceMapUpdate),
}

impl std::fmt::Debug for ExecutionMap {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Planar(map) => formatter.debug_tuple("Planar").field(map).finish(),
            Self::Surface(_) | Self::SurfaceUpdate(_) => formatter.write_str("Surface(..)"),
        }
    }
}

/// Production controller cadence: 20 Hz.
pub const DEFAULT_TICK_INTERVAL: Duration = Duration::from_millis(50);
/// 零表示不按固定接收年龄终止导航，来源失效仍由拥有方显式撤销。
pub const DEFAULT_POSE_TTL: Duration = Duration::ZERO;
/// 地图在同一来源内持续有效，直到更新、清空或来源失效。
pub const DEFAULT_COSTMAP_TTL: Duration = Duration::ZERO;
/// 默认连续等待路径或可执行运动的期限；成功执行运动后重新计时。
pub const DEFAULT_PATH_WAIT_TIMEOUT: Duration = Duration::from_secs(30);
/// 局部运动预测长度，与输入过期门槛独立。
pub const DEFAULT_CONTROL_HORIZON: Duration = Duration::from_millis(250);
const INPUT_CAPACITY: usize = 32;
const URGENT_CAPACITY: usize = 8;

/// 生产导航只配置守卫连接位置。
pub fn production_client_config(socket_path: impl Into<PathBuf>) -> ClientConfig {
    ClientConfig { socket_path: socket_path.into() }
}

/// Complete in-process execution identity correlated from sealed goal and path envelopes.
///
/// This type is deliberately not a wire schema. Goal admission initializes the
/// goal-level fields and leaves the planner/path fields at their neutral values;
/// an accepted path carries the same goal-level fields plus its sealed planner
/// generation and revision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PathExecutionIdentity {
    /// 原目标的两个分配代次。
    pub goal_key: GoalKey,
    /// 原目标会话标识。
    pub session_id: String,
    /// 路径所属规划进程代次。
    pub planner_runtime_generation: i64,
    /// 同一目标内单调路径修订。
    pub path_revision: u64,
}

impl PathExecutionIdentity {
    /// Original Geometry-assigned goal identity.
    pub fn goal_key(&self) -> GoalKey { self.goal_key }

}

/// One navigation session and its request-level motion admission.
#[derive(Debug, Clone, PartialEq)]
pub struct NavSession {
    /// 原任务的平面边界，局部重规划沿用同一范围。
    pub boundary: Vec<[f64; 2]>,
    /// 随封存目标核对的倒车授权与到达容差。
    pub options: nuwax_nav_types_messages::GoalOptions,
    /// Stable goal/session identifier.
    pub session_id: String,
    /// Complete sealed goal identity expected on every admitted path.
    pub execution_identity: PathExecutionIdentity,
    /// Request-level motion admission proof transported with the goal.
    pub motion_generation: u64,
    /// One authoritative typed request; wire callers explicitly choose Body.
    pub goal: ExecutionGoal,
}

/// 路线用途沿原生规划、消息与调度入口保持一致。
pub use nuwax_nav_pct_native::RouteKind;

/// 带完整关联身份与路线用途的执行输入。
#[derive(Debug, Clone, PartialEq)]
pub struct VersionedPath {
    /// 明确区分完整路线与逼近路线，逼近结束不能完成业务目标。
    pub route_kind: RouteKind,
    /// Goal/session identifier copied from the versioned goal envelope.
    pub session_id: String,
    /// Complete identity copied from the sealed goal and path envelopes.
    pub execution_identity: PathExecutionIdentity,
    /// Monotonic path revision within the active navigation session.
    pub path_revision: u64,

    /// Planner output consumed by the local controller.
    pub path: ExecutionPath,
    /// 可选路径朝向；生产入口由已核对身份的目标与路径生成，末项为请求的到达朝向。
    pub path_yaws: Option<Vec<f64>>,
}

/// Scheduler lifecycle. Terminal meaning is carried separately in status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerPhase {
    /// No lease and no active session.
    Idle,
    /// Waiting for the motion guard to grant a NAV lease.
    Acquiring,
    /// 租约已取得，等待首次有效路径或已确认零速后的更新路径。
    AwaitingPath,
    /// 以20赫兹运行控制器并发送速度。
    Controlling,
    /// Cancel accepted; no more twist is issued while fence proof is pending.
    Cancelling,
    /// Fail-closed state. No twist or renewal is issued.
    Failed,
}

/// Verified terminal outcome from the most recent session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalOutcome {
    /// 申请被拒或租约已作废。
    Stopped,
    /// Cancel completed after a valid resource fence.
    Cancelled,
    /// 规划到达、租约已围栏，停车后采样仍满足原目标。
    Succeeded,
    /// Planner failed and the lease was fenced.
    PlannerFailed,
    /// A versioned-path protocol violation was fenced.
    SafetyFailed,
    /// Scheduler shutdown fenced the active lease.
    Shutdown,
}

/// Observable state published by the scheduler task.
#[derive(Debug, Clone, PartialEq)]
pub struct SchedulerStatus {
    /// Current scheduler phase.
    pub phase: SchedulerPhase,
    /// Active session, if any.
    pub session_id: Option<String>,
    /// Latest accepted path revision.
    pub path_revision: u64,
    /// 守卫已接受的速度请求数。
    pub applied_commands: u64,
    /// 最近一次被守卫接受的速度，不代表实测运动。
    pub last_applied_twist: Option<Twist2>,
    /// Whether the scheduler's motion transport is currently usable.
    pub motion_sink_ready: bool,

    /// Whether both planner inputs are within their receipt TTL.
    pub planner_inputs_fresh: bool,
    /// Source timestamp from the most recently consumed map pose.
    pub pose_source_timestamp_ns: i64,
    /// Source timestamp from the most recently consumed occupancy snapshot.
    pub costmap_source_timestamp_ns: i64,
    /// Monotonic receipt age of the latest map pose.
    pub pose_receipt_age_ms: u64,
    /// Monotonic receipt age of the latest occupancy snapshot.
    pub costmap_receipt_age_ms: u64,
    /// Most recent verified terminal outcome.
    pub last_terminal: Option<TerminalOutcome>,
    /// Diagnostic message for operators and tests.
    pub message: String,
    /// 当前目标的局部规划轨迹，仅用于显示。
    pub local_path: Vec<[f64; 3]>,
    /// 沿已接受路线，从当前位姿到末端的剩余距离。
    pub remaining_distance_m: f64,
    /// 当前规划结果明确没有可行路线。
    pub no_feasible_path: bool,
    /// In-process execution evidence; never grants motion or extends a wait.
    pub surface_replan_feedback: Option<SurfaceReplanFeedback>,
}

impl Default for SchedulerStatus {
    fn default() -> Self {
        Self {
            phase: SchedulerPhase::Idle,
            session_id: None,
            path_revision: 0,
            applied_commands: 0,
            last_applied_twist: None,
            motion_sink_ready: false,

            planner_inputs_fresh: false,
            pose_source_timestamp_ns: 0,
            costmap_source_timestamp_ns: 0,
            pose_receipt_age_ms: u64::MAX,
            costmap_receipt_age_ms: u64::MAX,
            last_terminal: None,
            message: "idle".into(),
            local_path: Vec::new(),
            remaining_distance_m: 0.0,
            no_feasible_path: false,
            surface_replan_feedback: None,
        }
    }
}

/// Scheduler configuration independent of transport wiring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SchedulerConfig {
    /// Controller tick interval.
    pub tick_interval: Duration,
    /// 等待路径期限；零表示等待至路径就绪或取消。
    pub path_wait_timeout: Duration,
    /// 位姿接收期限；零表示不以固定接收年龄终止。
    pub pose_ttl: Duration,
    /// 地图接收期限；零表示保持至来源显式失效。
    pub costmap_ttl: Duration,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            tick_interval: DEFAULT_TICK_INTERVAL,
            path_wait_timeout: DEFAULT_PATH_WAIT_TIMEOUT,
            pose_ttl: DEFAULT_POSE_TTL,
            costmap_ttl: DEFAULT_COSTMAP_TTL,
        }
    }
}

/// Typed scheduler errors.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum RuntimeError {
    /// Caller or configuration input is invalid.
    #[error("invalid navigation input `{field}`: {reason}")]
    InvalidInput {
        /// Invalid field.
        field: &'static str,
        /// Validation reason.
        reason: String,
    },
    /// Another session still owns or may own the motion resource.
    #[error("navigation scheduler is busy: {0}")]
    Busy(String),
    /// A stale or cross-session path was rejected without replacing the active path.
    #[error("versioned path rejected: {reason}")]
    PathRejected {
        /// Correlation failure.
        reason: String,
    },
    /// One immutable path revision arrived with different bytes.
    #[error("versioned path mutation detected: {reason}")]
    PathMutation {
        /// Immutability violation.
        reason: String,
    },
    /// Motion port failure. The result may be ambiguous.
    #[error("motion guard operation failed: {0}")]
    Motion(String),
    /// 守卫明确没有授予本目标租约；该目标可以失败并释放尚未执行的责任。
    #[error("{0}")]
    LeaseNotGranted(String),
    /// Planner reported a terminal failure.
    #[error("planner failed: {0}")]
    Planner(String),
    /// A controller input exceeded its receipt TTL or replayed an old source timestamp.
    #[error("stale navigation input: {0}")]
    StaleInput(String),
    /// 地图已明确清空、换域或失去完整性，原目标不能由新输入自动恢复。
    #[error("navigation map source invalidated: {0}")]
    MapSourceInvalidated(String),
    /// The scheduler task is no longer accepting requests.
    #[error("navigation scheduler task is stopped")]
    SchedulerStopped,

}

/// 运动边界的失败原因，保留守卫原错误码。
#[derive(Debug, Error, Clone, PartialEq, Eq)]
#[error("{message}")]
pub struct MotionPortError {
    /// 失败原因。
    pub message: String,
    /// 守卫原错误码；传输失败没有协议错误码。
    pub code: Option<ErrorCode>,
}
impl MotionPortError {
    /// 非协议失败。
    pub fn new(message: impl Into<String>) -> Self { Self { message: message.into(), code: None } }
}
impl From<MotionGuardClientError> for MotionPortError {
    fn from(error: MotionGuardClientError) -> Self {
        let code = match &error { MotionGuardClientError::Remote { code, .. } => Some(*code), _ => None };
        Self { message: error.to_string(), code }
    }
}

/// One planner-consumed occupancy snapshot with immutable source metadata.
#[derive(Debug, Clone)]
pub struct CostmapObservation {
    /// 保留原维度的局部执行地图；三维表面只共享不可变快照。
    pub costmap: ExecutionMap,
    /// Positive source timestamp carried by the occupancy frame.
    pub source_timestamp_ns: i64,
}

/// 本地来源期限检查后的速度发送结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundedTwistOutcome {
    /// 守卫接受了速度请求；不代表实测运动。
    Applied,
    /// 发送前来源期限已到。
    ExpiredNotApplied,
}

/// 同进程地图来源事件；临时缺输入与原目标所属地图失效具有不同生命周期。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapInvalidationKind {
    /// 同域完整输入暂未到达，保持零速并等待原目标恢复。
    TemporaryUnavailable,
    /// 显式清图、换域或完整性失效，结束原目标并取得真实停车证明。
    Invalidated,
}

/// 由单个调度任务拥有的运动连接。
#[async_trait]
pub trait NavMotionPort: Send + 'static {
    /// 来源期限内的真实测速，缺失不代表已停车。
    fn measured_motion(&self) -> Option<MeasuredMotion> { None }
    /// 用目标携带的代次申请导航租约。
    async fn acquire(&mut self, generation: u64) -> Result<u64, MotionPortError>;
    /// 用当前租约发送速度并续租。
    async fn velocity(&mut self, twist: Twist2) -> Result<(), MotionPortError>;
    /// 发送前保留规划器来源期限检查，不把期限写入守卫协议。
    async fn velocity_bounded(&mut self, twist: Twist2, bound: Option<CommandDispatchBound>) -> Result<BoundedTwistOutcome, MotionPortError> {
        if bound.is_some_and(CommandDispatchBound::expired_now) { return Ok(BoundedTwistOutcome::ExpiredNotApplied); }
        self.velocity(twist).await?;
        Ok(BoundedTwistOutcome::Applied)
    }
    /// 目标结束时释放租约。
    async fn release(&mut self) -> Result<(), MotionPortError>;
}
#[async_trait]
impl NavMotionPort for MotionGuardClient {
    fn measured_motion(&self) -> Option<MeasuredMotion> { self.measured_motion() }
    async fn acquire(&mut self, generation: u64) -> Result<u64, MotionPortError> {
        Ok(self.acquire(Source::Nav, generation).await?)
    }
    async fn velocity(&mut self, twist: Twist2) -> Result<(), MotionPortError> {
        self.velocity(twist.linear.x, twist.linear.y, twist.angular_z).await?;
        Ok(())
    }
    async fn release(&mut self) -> Result<(), MotionPortError> { Ok(self.release().await?) }
}

/// Result of one deterministic local-planner step.
#[derive(Debug, Clone, PartialEq)]
pub enum PlannerStep {
    /// Continue controlling with this twist.
    Command(Twist2),
    /// No admitted control result yet; retain route/goal and the existing wait deadline.
    Pending(Vec<String>),
    /// Goal arrival detected.
    Succeeded,
    /// Planner cannot safely continue.
    Failed(String),
    /// 当前路径已失效，先确认零速再有界等待更新的规划修订。
    ReplanRequired(String),
}

#[derive(Debug, Clone, PartialEq)]
struct LocalExecutionCommandIntent {
    identity: Arc<PathExecutionIdentity>,

    payload: LocalExecutionPayload,
    dispatch_bound: Option<CommandDispatchBound>,
    execution_context: Option<Arc<ActualExecutionContext>>,
}

#[derive(Debug, Clone, PartialEq)]
enum LocalExecutionPayload {
    Command(Twist2),
    Pending(Vec<String>),
    Succeeded,
    Failed(String),
    ReplanRequired(String),
}

/// Synchronous deterministic planner boundary.
pub trait NavPlanner: Send + 'static {
    /// 由调度器在启动时传入实际控制周期。
    fn set_control_period(&mut self, _period: Duration) {}
    /// Refresh only actual source/pose/carried-ground observations. This must
    /// neither step a controller nor validate a predicted motion or issue a command.
    /// `applied` is the caller's consumed actuator feedback, never a candidate.
    /// Participating backends preserve actual source/pose/carried ground across
    /// `stop`; clearing control state alone does not change this observation.
    fn refresh_execution_context(&mut self, _applied: Twist2) -> Option<Arc<ActualExecutionContext>> { None }
    /// Context bound to the immediately returned step. Asynchronous adapters
    /// override this with their delivered result slot, never a newer input slot.
    fn take_step_execution_context(&mut self, applied: Twist2) -> Option<Arc<ActualExecutionContext>> {
        self.refresh_execution_context(applied)
    }
    /// Take observation timing metadata belonging only to the immediately returned command.
    /// None means that this result has no finite observation-time dependency.
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {None}
    /// 传递既有导航会话标识用于计算线程日志，不参与运动准入。
    fn set_goal_log_context(&mut self, _goal_id: &str) {}
    /// 接续局部路线时保留已受理任务边界；无局部重规划的后端可沿原全局路径。
    fn configure_boundary(&mut self, _boundary: &[[f64; 2]]) {}
    /// 在获取新目标租约前捕获执行选项；默认后端不推定支持倒车。
    fn configure_goal(&mut self, _goal: &PoseStamped, options: nuwax_nav_types_messages::GoalOptions) -> Result<(), RuntimeError> {
        options.validate().map_err(|reason| RuntimeError::InvalidInput {field:"goal_options",reason})?;
        if options.allow_reverse {return Err(RuntimeError::InvalidInput {field:"allow_reverse",reason:"执行后端未声明倒车能力".into()});}
        Ok(())
    }
    /// 目标配置沿平台机体语义传递。
    fn configure_execution_goal(&mut self, goal:&ExecutionGoal, options:nuwax_nav_types_messages::GoalOptions)
        -> Result<(),RuntimeError> { self.configure_goal(goal.as_body().unwrap(),options) }
    /// 路径保留三维机体参考。
    fn replace_typed_execution_path(&mut self,path:ExecutionPath,yaws:Option<Vec<f64>>,goal:ExecutionGoal)
        ->Result<(),RuntimeError> { self.replace_execution_path(path,yaws,goal.as_body().unwrap().clone()) }
    /// Replace the active plan after version validation.
    fn replace_path(&mut self, path: Path2D);
    /// 替换路径与可选朝向；不使用朝向的兼容后端仍沿原接口运行。
    fn replace_path_with_orientations(&mut self, path: Path2D, _yaws: Option<Vec<f64>>) {
        self.replace_path(path);
    }
    /// 接收经调度器核对的完整路径与原用户目标，默认后端不支持三维降级。
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        _goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        match path {
            ExecutionPath::Planar(path) => {
                self.replace_path_with_orientations(path, yaws);
                Ok(())
            }
            ExecutionPath::Surface(_) => Err(RuntimeError::InvalidInput {
                field: "execution_path",
                reason: "当前局部后端不支持三维路径".into(),
            }),
        }
    }
    /// Update the latest odometry sample.
    fn update_odometry(&mut self, odometry: PoseStamped);
    /// Update the latest planner-consumed occupancy costmap.
    fn update_costmap(&mut self, costmap: CostMap2D);
    /// 接收与路径维度匹配的地图，默认后端不能把表面快照当作二维占据图。
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        match map {
            ExecutionMap::Planar(map) => {
                self.update_costmap(map);
                Ok(())
            }
            ExecutionMap::Surface(_) | ExecutionMap::SurfaceUpdate(_) => Err(RuntimeError::InvalidInput {
                field: "execution_map",
                reason: "当前局部后端不支持三维表面地图".into(),
            }),
        }
    }
    /// 来源被明确清空或更换时撤销局部缓存，不能用旧地图重新恢复运动。
    fn invalidate_execution_map(&mut self) {
        self.stop();
    }
    /// 接收运动端确认已应用的命令值，供下一拍规划接续；候选在回执前不进入该状态。
    fn observe_applied_command(&mut self, _command: Twist2) {}
    /// 独立实测运动快照；保留缺失，与 pose 不假定同一采样。
    fn observe_measured_motion(&mut self, _motion: Option<MeasuredMotion>) {}
    /// 已准入的同帧 world 点云；None 撤销旧原帧，不借用最新定位补齐。
    fn observe_registered_frame(&mut self, _frame: Option<Arc<RegisteredFrame>>) {}
    /// 本次原算法执行周期结束，不是实际几何拒绝；只用于反馈原因分类。
    fn execution_cycle_complete(&self) -> bool { false }
    /// 计算未完成或控制许可过期时的等待原因；命令回执不能覆盖该执行阶段。
    fn execution_wait_reason(&self) -> Option<String> { None }
    /// 新结果未出时，上一条已交付且仍在执行的非零指令；超出其原有许可期返回None。
    fn held_command(&self) -> Option<Twist2> { None }
    /// 只读展示当前局部规划，不改变速度或路径许可。
    fn local_path_for_display(&self) -> Vec<[f64; 3]> { Vec::new() }
    /// Prepare a deferred control result without blocking the scheduler task.
    /// A receiver requests a bounded wait for publication, not command authority.
    /// `continuing_wait` keeps an already deferred tick waiting for matching input
    /// versions; it does not broaden which initial steps may defer.
    /// Synchronous backends return None and retain their immediate step behavior.
    /// Delegating adapters must forward this hook with `step` to preserve scheduling.
    fn prepare_control_step(&mut self, _continuing_wait: bool) -> Option<watch::Receiver<()>> { None }
    /// Produce one controller step without waiting for a background computation.
    fn step(&mut self) -> PlannerStep;
    /// Clear planner state after a terminal fence.
    fn stop(&mut self);
}

/// Production-qualified local execution backend selected at scheduler construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalExecutionBackendKind {
    /// Run the deterministic basic local planner inside the scheduler task.
    BasicInProcess,
    /// 显式配置机体到支撑面的标定后，在同一调度任务内跟随三维路径。
    SurfaceInProcess,
}

struct BasicInProcessLocalBackend(LocalPlanner);

impl BasicInProcessLocalBackend {
    fn production(geometry: nuwax_nav_types_messages::RobotGeometry) -> Self {
        Self(LocalPlanner::with_config(LocalPlannerConfig::default_config(), geometry))
    }

    fn set_navigation_motion_limits(&mut self, physical: nuwax_nav_types_messages::MotionLimits) -> Result<(),RuntimeError> {
        physical.validate().map_err(|reason|RuntimeError::InvalidInput{field:"navigation_capabilities",reason})?;
        // 旧二维后端保持原前向合同；本轮原生 SCAN 不经过这个适配器。
        let mut legacy=physical;
        if legacy.model==nuwax_nav_types_messages::MotionModel::Omnidirectional {
            legacy.model=nuwax_nav_types_messages::MotionModel::Differential;
        }
        legacy.max_reverse_mps=0.;legacy.max_lateral_mps=0.;
        self.0.set_motion_limits(legacy)
            .map_err(|reason|RuntimeError::InvalidInput{field:"navigation_capabilities",reason})
    }

    #[cfg(test)]
    fn kind(&self) -> LocalExecutionBackendKind {
        LocalExecutionBackendKind::BasicInProcess
    }
}

impl NavPlanner for BasicInProcessLocalBackend {
    fn configure_goal(&mut self, goal:&PoseStamped, options:nuwax_nav_types_messages::GoalOptions)->Result<(),RuntimeError> {
        let options=nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options};
        self.0.configure_goal(goal.clone(),options).map_err(|reason| RuntimeError::InvalidInput {field:"goal_options",reason})
    }

    fn replace_path(&mut self, path: Path2D) {
        self.0.start_planning(path);
    }

    fn replace_path_with_orientations(&mut self, path: Path2D, yaws: Option<Vec<f64>>) {
        self.0.start_planning_with_orientations(path, yaws);
    }

    fn update_odometry(&mut self, odometry: PoseStamped) {
        self.0.handle_odom(odometry);
    }

    fn update_costmap(&mut self, costmap: CostMap2D) {
        self.0.update_costmap(costmap);
    }

    fn step(&mut self) -> PlannerStep {
        let (twist, stop) = self.0.step();
        match stop {
            None => PlannerStep::Command(twist),
            Some(StopMessage::Arrived) => PlannerStep::Succeeded,
            Some(StopMessage::ObstacleFound) => PlannerStep::ReplanRequired("局部轨迹受阻，停车等待新路径".into()),
            Some(StopMessage::Error) => PlannerStep::Failed("local planner error".into()),
        }
    }

    fn stop(&mut self) {
        self.0.stop_planning();
    }
}

fn production_local_backend(
    kind: LocalExecutionBackendKind,
) -> Result<BasicInProcessLocalBackend, RuntimeError> {
    match kind {
        LocalExecutionBackendKind::BasicInProcess => {
            let value = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON")
                .map_err(|error| RuntimeError::InvalidInput { field: "robot_geometry", reason: error.to_string() })?;
            let geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(&value)
                .map_err(|reason| RuntimeError::InvalidInput { field: "robot_geometry", reason })?;
            let mut backend = BasicInProcessLocalBackend::production(geometry);
            if let Some(capabilities)=nuwax_nav_types_messages::NavigationCapabilities::from_env().map_err(|reason| RuntimeError::InvalidInput {field:"navigation_capabilities",reason})? {
                backend.set_navigation_motion_limits(capabilities.motion)?;
            }
            Ok(backend)
        }
        LocalExecutionBackendKind::SurfaceInProcess => Err(RuntimeError::InvalidInput {
            field: "robot_geometry",
            reason: "三维后端必须通过显式配置构造".into(),
        }),
    }
}

/// Single-task navigation scheduler.
///
/// The motion port is stored by value and is only accessed through `&mut self`,
/// 申请、速度和释放在同一连接上按序执行。
pub struct NavScheduler<M, P> {
    motion: M,
    planner: P,
    config: SchedulerConfig,
    session: Option<NavSession>,
    lease_active: bool,
    current_path_revision: u64,
    current_route_kind: RouteKind,

    current_path: Option<(ExecutionPath, Option<Vec<f64>>)>,
    current_execution_identity: Option<Arc<PathExecutionIdentity>>,
    path_wait_started_at: Option<Instant>,
    awaiting_session_path: bool,
    latest_pose: Option<PoseStamped>,
    pose_acquired_at: Option<Instant>,
    pose_receipt: Option<PlannerInputReceipt>,
    pose_valid_until: Option<Instant>,
    pose_source_max_age: Option<Duration>,
    obstacle_source_available: bool,
    obstacle_source_bound: Option<CommandDispatchBound>,
    costmap_receipt: Option<PlannerInputReceipt>,
    map_invalidated_at: Option<Instant>,
    status: SchedulerStatus,
    #[cfg(feature = "surface-recovery-qualification")]
    qualification_timing: Option<qualification_timing::TraceSink>,
}

#[derive(Debug, Clone, Copy)]
struct PlannerInputReceipt {
    source_timestamp_ns: i64,
    received_at: Instant,
}

impl<M, P> NavScheduler<M, P>
where
    M: NavMotionPort,
    P: NavPlanner,
{
    /// Construct a scheduler without starting a task.
    pub fn new(motion: M, mut planner: P, config: SchedulerConfig) -> Result<Self, RuntimeError> {
        validate_config(config)?;
        planner.set_control_period(config.tick_interval);
        let status = SchedulerStatus {
            motion_sink_ready: true,
            message: "motion scheduler ready".into(),
            ..SchedulerStatus::default()
        };
        Ok(Self {
            motion,
            planner,
            config,
            session: None,
            lease_active: false,
            current_path_revision: 0,
            current_route_kind: RouteKind::Complete,

            current_path: None,
            current_execution_identity: None,
            path_wait_started_at: None,
            awaiting_session_path: false,
            latest_pose: None,
            pose_acquired_at: None,
            pose_receipt: None,
            pose_valid_until: None,
            pose_source_max_age: None,
            obstacle_source_available: true,
            obstacle_source_bound: None,
            costmap_receipt: None,
            map_invalidated_at: None,
            status,
            #[cfg(feature = "surface-recovery-qualification")]
            qualification_timing: None,
        })
    }

    /// Attach an explicit qualification-only observer; default construction is silent.
    #[cfg(feature = "surface-recovery-qualification")]
    pub fn with_qualification_timing(mut self, sink: qualification_timing::TraceSink) -> Self {
        self.qualification_timing = Some(sink);
        self
    }

    #[cfg(feature = "surface-recovery-qualification")]
    fn qualification_span(&self, phase: &'static str, source: i64)
        -> Option<qualification_timing::SpanGuard> {
        self.qualification_timing.as_ref().map(|sink| sink.span(phase, source))
    }

    #[cfg(feature = "surface-recovery-qualification")]
    fn qualification_snapshot(&self, phase: &'static str, source: i64,
        stale: Option<bool>, stale_cause_bits: u8) {
        let Some(sink) = self.qualification_timing.as_ref().filter(|sink| sink.enabled()) else { return; };
        // These are copies of admitted fields. Do not call refresh_input_status,
        // stale_input_reason, expired_now, planner methods, or any geometry here.
        let now = Instant::now();
        let recent = self.obstacle_source_bound.map(|bound| bound.qualification_timing_fields());
        sink.snapshot_at(phase, source, now.into_std(), qualification_timing::SchedulerSnapshot {
            phase: self.status.phase, lease_active: self.lease_active,
            session_active: self.session.is_some(), planner_inputs_fresh: self.status.planner_inputs_fresh,
            path_revision: self.current_path_revision,
            pose_source_ns: self.pose_receipt.map(|receipt| receipt.source_timestamp_ns),
            map_source_ns: self.costmap_receipt.map(|receipt| receipt.source_timestamp_ns),
            pose_receipt_age_ns: self.pose_receipt.map(|receipt|
                qualification_timing::nanos(now.saturating_duration_since(receipt.received_at))),
            map_receipt_age_ns: self.costmap_receipt.map(|receipt|
                qualification_timing::nanos(now.saturating_duration_since(receipt.received_at))),
            pose_deadline_remaining_ns: self.pose_valid_until.map(|deadline|
                qualification_timing::remaining_ns(deadline.into_std(), now.into_std())),
            recent_available: self.obstacle_source_available,
            recent_bound_present: recent.is_some(),
            recent_anchor_unix_ns: recent.map(|(anchor, _, _)| anchor),
            recent_latest_issue_unix_ns: recent.map(|(_, latest, _)| latest),
            recent_deadline_remaining_ns: recent.and_then(|(_, _, deadline)| deadline)
                .map(|deadline| qualification_timing::remaining_ns(deadline, now.into_std())),
            last_applied: self.status.last_applied_twist.map(|twist|
                [twist.linear.x, twist.linear.y, twist.angular_z]),
            stale, stale_cause_bits,
        });
    }

    async fn run(
        mut self,
        mut input_rx: mpsc::Receiver<InputCommand>,
        mut urgent_rx: mpsc::Receiver<UrgentCommand>,
        status_tx: watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        let mut tick = interval_at(
            Instant::now() + self.config.tick_interval,
            self.config.tick_interval,
        );
        tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut input_open = true;
        let mut urgent_open = true;
        let mut deferred_control: Option<DeferredControl> = None;

        loop {
            // A deferred tick belongs to one accepted path, never the next goal
            // or a replacement path admitted while computation was outstanding.
            if deferred_control.as_ref().is_some_and(|wait|
                self.status.phase != SchedulerPhase::Controlling
                    || !self.current_execution_identity.as_ref()
                        .is_some_and(|identity| Arc::ptr_eq(identity, &wait.identity))) {
                deferred_control = None;
            }
            if !input_open && !urgent_open {
                return self
                    .finish_terminal(&status_tx, TerminalOutcome::Shutdown, SchedulerPhase::Idle)
                    .await;
            }

            let controlling = matches!(
                self.status.phase,
                SchedulerPhase::AwaitingPath | SchedulerPhase::Controlling
            );
            tokio::select! {
                biased;

                command = urgent_rx.recv(), if urgent_open => {
                    match command {
                        Some(command) => {
                            if let Some(result) = self.handle_urgent(&status_tx, command).await {
                                return result;
                            }
                        }
                        None => urgent_open = false,
                    }
                }

                event = async {
                    if let Some(wait) = deferred_control.as_mut() {
                        match wait.queued_inputs {
                            Some(0) => SchedulerEvent::ControlReady,
                            Some(_) => SchedulerEvent::Input(input_rx.recv().await),
                            None => tokio::select! {
                                biased;
                                finish = wait.ready() => SchedulerEvent::Publication(finish),
                                _ = tick.tick() => SchedulerEvent::Heartbeat,
                                command = input_rx.recv(), if input_open => SchedulerEvent::Input(command),
                            },
                        }
                    } else {
                        // 普通输入与控制公平轮换；取消和续租仍由外层优先选择。
                        tokio::select! {
                            command=input_rx.recv(), if input_open=>SchedulerEvent::Input(command),
                            _=tick.tick(), if controlling=>SchedulerEvent::Tick,
                            else=>std::future::pending().await,
                        }
                    }
                } => match event {
                    SchedulerEvent::Input(command)=> {
                        if let Some(remaining) = deferred_control.as_mut().and_then(|wait| wait.queued_inputs.as_mut()) {
                            *remaining = remaining.saturating_sub(1);
                        }
                        match command {
                            Some(command)=>self.handle_input(&status_tx,command).await,
                            None=>input_open=false,
                        }
                    }
                    SchedulerEvent::Publication(finish_wait) => {
                        if let Some(wait) = deferred_control.as_mut() {
                            // Admit the finite prefix already queued at readiness.
                            // Versions advance only in handle_input, so consuming a
                            // publication first could otherwise dispatch old sources.
                            // New arrivals do not extend this prefix or its deadline.
                            wait.queued_inputs = Some(input_rx.len());
                            wait.finish_wait = finish_wait;
                        }
                    }
                    SchedulerEvent::Heartbeat => {
                        let command = if self.stale_input_reason(Instant::now()).is_some() { Twist2::ZERO }
                            else { self.planner.held_command().unwrap_or(Twist2::ZERO) };
                        if let Err(error) = self.send_controller_twist_with_bound(&status_tx, command, None, false).await {
                            if self.session.is_none() { continue; }
                            return Err(error);
                        }
                    }
                    SchedulerEvent::Tick | SchedulerEvent::ControlReady => {
                        self.planner.observe_measured_motion(self.motion.measured_motion());
                        let previous = deferred_control.take();
                        let deadline = previous.as_ref().map_or_else(
                            || Instant::now() + self.config.tick_interval, |wait| wait.deadline);
                        let finish_wait = previous.as_ref().is_some_and(|wait| wait.finish_wait)
                            || Instant::now() >= deadline;
                        if !finish_wait && self.status.phase == SchedulerPhase::Controlling {
                            if let Some(identity) = self.current_execution_identity.clone() {
                                let publication = {
                                    #[cfg(feature = "surface-recovery-qualification")]
                                    let _qualification_span = self.qualification_span(
                                        "scheduler.prepare_control_step", self.status.pose_source_timestamp_ns);
                                    self.planner.prepare_control_step(previous.is_some())
                                };
                                if let Some(publication) = publication {
                                    deferred_control = Some(DeferredControl {
                                        publication, deadline, identity, queued_inputs: None, finish_wait: false,
                                    });
                                    continue;
                                }
                            }
                        }
                    if let Err(error) = self.session_tick(&status_tx).await {
                        if !self.lease_active && self.session.is_none() { continue; }
                        self.status.motion_sink_ready = false;
                        self.publish_status(&status_tx);
                        return Err(error);
                    }
                    }
                }
            }
        }
    }

    async fn handle_urgent(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        command: UrgentCommand,
    ) -> Option<Result<(), RuntimeError>> {
        match command {
            UrgentCommand::Cancel { session_id, reply } => {
                #[cfg(feature = "surface-recovery-qualification")]
                if let Some(sink) = &self.qualification_timing {
                    sink.mark("scheduler.cancel.handle.enter", 0);
                }
                let result = self.cancel(status_tx, &session_id).await;
                let _ = reply.send(result);
                None
            }
            UrgentCommand::InvalidateMap {
                reason,
                invalidated_at,
                kind,
                reply,
            } => {
                self.map_invalidated_at = Some(
                    self.map_invalidated_at
                        .map_or(invalidated_at, |previous| previous.max(invalidated_at)),
                );
                self.costmap_receipt = None;
                if kind == MapInvalidationKind::Invalidated {
                    self.planner.invalidate_execution_map();
                }
                self.refresh_input_status(Instant::now());
                let result = if self.lease_active || self.session.is_some() {
                    if kind == MapInvalidationKind::TemporaryUnavailable {
                        self.wait_for_fresh_input(status_tx, format!("地图来源暂不可用: {reason}")).await
                    } else {
                        let error = self.fail_closed(status_tx, RuntimeError::MapSourceInvalidated(reason)).await;
                        if self.lease_active { Err(error) } else { Ok(()) }
                    }
                } else {
                    self.status.message = format!("地图来源失效，等待新输入: {reason}");
                    self.publish_status(status_tx);
                    Ok(())
                };
                let _ = reply.send(result);
                None
            }
            UrgentCommand::Shutdown { reply } => {
                let result = self
                    .finish_terminal(status_tx, TerminalOutcome::Shutdown, SchedulerPhase::Idle)
                    .await;
                let _ = reply.send(result.clone());
                Some(result)
            }
        }
    }

    async fn handle_input(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        command: InputCommand,
    ) {
        #[cfg(feature = "surface-recovery-qualification")]
        let qualification_input = self.qualification_timing.as_ref().map(|sink| {
            let (kind, source) = match &command.kind {
                InputKind::StartSession(_) => ("scheduler.input.start_session", 0),
                InputKind::SubmitPath(_) => ("scheduler.input.path", 0),
                InputKind::UpdateOdometry(pose, _, _) | InputKind::UpdateSourceOdometry(pose, _, _) => ("scheduler.input.pose", pose.stamp.nanos),
                InputKind::UpdateCostmap(map, _) => ("scheduler.input.map", map.source_timestamp_ns),
                // Runtime owns no recent source timestamp; record its actual incoming bound.
                InputKind::UpdateObstacleSource(_, _, _, _) => ("scheduler.input.recent", 0),
                InputKind::PlanningDiagnostic(_, _, _, _, _) => ("scheduler.input.planning_diagnostic", 0),
                InputKind::CapturePlanningActual(_, _) => ("scheduler.input.capture_actual", 0),
            };
            sink.mark("scheduler.input.dequeue", source);
            match &command.kind {
                InputKind::SubmitPath(path) => sink.count("scheduler.input.path_revision", source, path.path_revision),
                InputKind::UpdateObstacleSource(_, available, bound, _) => {
                    sink.count("scheduler.input.recent_available", 0, u64::from(*available));
                    sink.bound("scheduler.input.recent_bound", 0, *bound);
                }
                _ => {}
            }
            (sink.clone(), source, sink.span(kind, source))
        });
        let result = match command.kind {
            InputKind::StartSession(session) => self.start_session(status_tx, session).await,
            InputKind::SubmitPath(path) => self.submit_path(status_tx, path).await,
            InputKind::UpdateOdometry(odometry, valid_until, acquired_at) => {
                self.observe_odometry_at(status_tx, odometry, valid_until, acquired_at)
            }
            InputKind::UpdateSourceOdometry(odometry, acquired_at, max_age) => {
                self.observe_odometry_qualified_at(status_tx, odometry, None, acquired_at, Some(max_age))
            }
            InputKind::UpdateCostmap(observation, received_at) => {
                self.observe_costmap(status_tx, observation, received_at)
            }
            InputKind::UpdateObstacleSource(map, available, bound, registered) => (|| {
                if let Some(frame) = registered { self.planner.observe_registered_frame(frame); }
                if let Some(map)=map {
                    self.planner.update_execution_map(map.clone())?;
                }
                self.obstacle_source_available=available;
                self.obstacle_source_bound=bound;
                self.refresh_input_status(Instant::now());
                self.publish_status(status_tx);
                Ok(())
            })(),
            InputKind::CapturePlanningActual(identity, reply) => {
                let matching = self.session.as_ref().is_some_and(|session|
                    session.execution_identity.session_id == identity.session_id && session.execution_identity.goal_key == identity.goal_key);
                let context = if matching
                    && self.stale_input_reason(Instant::now()).is_none() {
                    // Worker reads only its same-version publication, including the
                    // lifecycle-only Initial commit when there is no path yet.
                    self.planner.refresh_execution_context(self.status.last_applied_twist.unwrap_or(Twist2::ZERO))
                        .filter(|context| self.latest_pose.as_ref().is_some_and(|pose|
                            replan_feedback::same_pose(pose, &context.odometry)))

                } else { None };
                let _ = reply.send(context);
                Ok(())
            }
            InputKind::PlanningDiagnostic(goal_id, key, message, failed, no_feasible_path) => {
                if matches!(self.status.phase,SchedulerPhase::AwaitingPath | SchedulerPhase::Controlling)
                    && self.session.as_ref().is_some_and(|session|
                        session.session_id == goal_id && session.execution_identity.goal_key() == key) {
                    self.status.no_feasible_path = no_feasible_path;
                    if failed {
                        self.status.message=message.clone();self.publish_status(status_tx);
                        let result=self.finish_terminal(status_tx,TerminalOutcome::PlannerFailed,SchedulerPhase::Failed).await;
                        if result.is_ok() {self.status.message=message;self.publish_status(status_tx);}
                        result
                    } else if self.status.phase == SchedulerPhase::AwaitingPath {
                        self.status.message = message; self.publish_status(status_tx);Ok(())
                    } else {Ok(())}
                } else {Ok(())}
            }
        };
        #[cfg(feature = "surface-recovery-qualification")]
        let qualification_ok = result.is_ok();
        let _ = command.reply.send(result);
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some((sink, source, _span)) = qualification_input {
            sink.mark(if qualification_ok { "scheduler.input.ack.ok" }
                else { "scheduler.input.ack.error" }, source);
        }
    }

    #[cfg(test)]
    fn observe_odometry(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        odometry: PoseStamped,
        valid_until: Option<Instant>,
    ) -> Result<(), RuntimeError> {
        self.observe_odometry_at(status_tx, odometry, valid_until, Instant::now())
    }

    fn observe_odometry_at(
        &mut self, status_tx: &watch::Sender<SchedulerStatus>, odometry: PoseStamped,
        valid_until: Option<Instant>, acquired_at: Instant,
    ) -> Result<(), RuntimeError> {
        self.observe_odometry_qualified_at(status_tx, odometry, valid_until, acquired_at, None)
    }

    fn observe_odometry_qualified_at(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        odometry: PoseStamped,
        valid_until: Option<Instant>,
        acquired_at: Instant,
        source_max_age: Option<Duration>,
    ) -> Result<(), RuntimeError> {
        if acquired_at > Instant::now() {
            return Err(RuntimeError::InvalidInput {
                field: "odometry.acquired_at", reason: "位姿采集时刻或来源期限无效".into(),
            });
        }
        if odometry.frame_id != nuwax_nav_types_messages::NavFrameId::Map {
            return Err(RuntimeError::InvalidInput {
                field: "odometry.frame_id",
                reason: format!("expected map, got {}", odometry.frame_id.as_str()),
            });
        }
        if valid_until.is_some_and(|deadline| deadline <= Instant::now()) {
            return Err(RuntimeError::StaleInput(
                "pose source deadline already elapsed before admission".into(),
            ));
        }
        let source_timestamp_ns = odometry.stamp.nanos;
        if let Some(max_age) = source_max_age {
            check_pose_data_age(source_timestamp_ns, max_age)?;
        }
        validate_source_timestamp(
            "odometry.source_timestamp_ns",
            source_timestamp_ns,
            self.pose_receipt,
        )?;
        self.latest_pose = Some(odometry.clone());
        self.pose_acquired_at = Some(acquired_at);
        self.planner.update_odometry(odometry);
        self.pose_receipt = Some(PlannerInputReceipt {
            source_timestamp_ns,
            received_at: Instant::now(),
        });
        self.pose_valid_until = valid_until;
        self.pose_source_max_age = source_max_age;
        self.refresh_input_status(Instant::now());
        if self.status.last_terminal.is_none() && self.session.is_none() {
            self.status.message = "fresh map pose consumed".into();
        }
        self.publish_status(status_tx);
        Ok(())
    }

    fn observe_costmap(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        observation: CostmapObservation,
        received_at: Instant,
    ) -> Result<(), RuntimeError> {
        if received_at > Instant::now() {
            return Err(RuntimeError::InvalidInput {
                field: "costmap.received_at",
                reason: "地图接收时刻不能位于未来".into(),
            });
        }
        if self
            .map_invalidated_at
            .is_some_and(|invalidated_at| received_at < invalidated_at)
        {
            return Err(RuntimeError::StaleInput(
                "地图在来源失效前已经接收，不能迟到恢复旧地图".into(),
            ));
        }
        // A verified surface mask/statistics update can keep the same source
        // stamp. Deliver its geometry, but never renew the original receipt.
        let unchanged_surface_source=matches!(&observation.costmap,ExecutionMap::SurfaceUpdate(_))
            && self.costmap_receipt.is_some_and(|old|old.source_timestamp_ns==observation.source_timestamp_ns);
        if !unchanged_surface_source {
            validate_source_timestamp("costmap.source_timestamp_ns",observation.source_timestamp_ns,self.costmap_receipt)?;
        }
        self.planner.update_execution_map(observation.costmap.clone())?;
        if !unchanged_surface_source {
            self.costmap_receipt = Some(PlannerInputReceipt {
                source_timestamp_ns: observation.source_timestamp_ns, received_at,
            });
        }
        self.refresh_input_status(Instant::now());
        if self.status.last_terminal.is_none() && self.session.is_none() {
            self.status.message = "fresh occupancy costmap consumed".into();
        }
        self.publish_status(status_tx);
        Ok(())
    }

    fn refresh_input_status(&mut self, now: Instant) {
        self.status.pose_source_timestamp_ns = self
            .pose_receipt
            .map_or(0, |receipt| receipt.source_timestamp_ns);
        self.status.costmap_source_timestamp_ns = self
            .costmap_receipt
            .map_or(0, |receipt| receipt.source_timestamp_ns);
        self.status.pose_receipt_age_ms = receipt_age_ms(self.pose_receipt, now);
        self.status.costmap_receipt_age_ms = receipt_age_ms(self.costmap_receipt, now);
        self.status.planner_inputs_fresh = self
            .pose_receipt
            .is_some_and(|receipt| self.pose_source_max_age.map_or_else(
                || self.config.pose_ttl.is_zero() || now.duration_since(receipt.received_at) <= self.config.pose_ttl,
                |maximum| nuwax_nav_types_messages::source_time::input_is_alive(
                    now.saturating_duration_since(receipt.received_at).as_nanos() as u64)
                    && check_pose_data_age(receipt.source_timestamp_ns, maximum).is_ok()))
            && self.pose_valid_until.is_none_or(|deadline| now < deadline)
            && self.obstacle_source_available
            && self.obstacle_source_bound.is_none_or(|bound|!bound.expired_now())
            && self.costmap_receipt.is_some_and(|receipt| {
                self.config.costmap_ttl.is_zero() || now.duration_since(receipt.received_at) <= self.config.costmap_ttl
            });
    }

    fn stale_input_reason(&mut self, now: Instant) -> Option<String> {
        self.refresh_input_status(now);
        if !self.obstacle_source_available {return Some("recent obstacle source revoked".into());}
        if self.obstacle_source_bound.is_some_and(CommandDispatchBound::expired_now) {
            return Some("recent obstacle source deadline elapsed".into());
        }
        let pose_reason = if let Some(maximum) = self.pose_source_max_age {
            self.pose_receipt.and_then(|receipt| {
                if !nuwax_nav_types_messages::source_time::input_is_alive(
                    now.saturating_duration_since(receipt.received_at).as_nanos() as u64) {
                    Some("pose input liveness deadline elapsed".into())
                } else { check_pose_data_age(receipt.source_timestamp_ns, maximum).err().map(|e| e.to_string()) }
            })
        } else {
            stale_receipt_reason("pose", self.pose_receipt, now, self.config.pose_ttl).or_else(
                || {
                    self.pose_valid_until
                        .filter(|deadline| now >= *deadline)
                        .map(|_| {
                            format!(
                                "pose source deadline elapsed (source_timestamp_ns={})",
                                self.status.pose_source_timestamp_ns
                            )
                        })
                },
            )
        };
        let costmap_reason = stale_receipt_reason(
            "costmap",
            self.costmap_receipt,
            now,
            self.config.costmap_ttl,
        );
        match (pose_reason, costmap_reason) {
            (None, None) => None,
            (Some(reason), None) | (None, Some(reason)) => Some(reason),
            (Some(pose), Some(costmap)) => Some(format!("{pose}; {costmap}")),
        }
    }

    async fn start_session(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        session: NavSession,
    ) -> Result<(), RuntimeError> {
        validate_session(&session)?;
        let replacing = self.session.is_some();
        if let Err(error) = self.motion.acquire(session.motion_generation).await {
            if error.code.is_none() {
                self.status.motion_sink_ready=false;
                return Err(self.fail_closed(status_tx,RuntimeError::Motion(error.message)).await);
            }
            self.status.message = error.message.clone();
            self.finish_terminal(status_tx, TerminalOutcome::Stopped, SchedulerPhase::Idle).await?;
            return Err(RuntimeError::LeaseNotGranted(error.message));
        }
        self.lease_active = true;
        if !replacing {
            self.planner.stop();
            if let Err(error) = self.planner.configure_execution_goal(&session.goal, session.options) {
                return Err(self.fail_closed(status_tx, error).await);
            }
            self.current_path = None;
            self.current_execution_identity = None;
            self.current_route_kind = RouteKind::Complete;
        }
        self.planner.set_goal_log_context(&session.session_id);
        self.planner.configure_boundary(&session.boundary);
        self.planner.observe_measured_motion(self.motion.measured_motion());
        self.status.no_feasible_path = false;
        self.status.applied_commands = 0;
        self.status.session_id = Some(session.session_id.clone());
        self.session = Some(session);
        self.awaiting_session_path = true;
        self.current_path_revision = 0;
        self.path_wait_started_at = Some(Instant::now());
        self.status.phase = if self.current_execution_identity.is_some() {
            SchedulerPhase::Controlling
        } else {
            SchedulerPhase::AwaitingPath
        };
        self.status.path_revision = 0;
        self.status.motion_sink_ready = true;
        self.status.last_terminal = None;
        self.status.surface_replan_feedback = None;
        self.status.message = "导航租约已接管，等待新路径".into();
        self.publish_status(status_tx);
        Ok(())
    }

    async fn submit_path(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        path: VersionedPath,
    ) -> Result<(), RuntimeError> {
        let Some(session) = self.session.as_ref() else {
            return Err(path_rejected("no active navigation session"));
        };
        if path.session_id != session.session_id {
            return Err(path_rejected("path does not belong to the current navigation session"));
        }
        if !self.lease_active
            || !matches!(
                self.status.phase,
                SchedulerPhase::AwaitingPath | SchedulerPhase::Controlling
            )
        {
            return Err(path_rejected("active session is not accepting paths"));
        }
        if path.path_revision < self.current_path_revision {
            return Err(path_rejected("path revision is stale"));
        }
        if path.path_revision == self.current_path_revision && self.current_path_revision != 0 {
            let immutable = self.current_route_kind == path.route_kind && self.current_path.as_ref().is_some_and(|(current, yaws)| {
                    same_execution_path(current, &path.path) && yaws == &path.path_yaws
                })
                && self
                    .current_execution_identity
                    .as_ref()
                    .is_some_and(|identity| identity.as_ref() == &path.execution_identity);
            if immutable {
                if self.status.phase == SchedulerPhase::AwaitingPath {
                    return Err(path_rejected(
                        "失效路径需要更高规划修订，不能用同修订重放恢复",
                    ));
                }
                return Ok(());
            }
            let error = RuntimeError::PathMutation {
                reason: format!(
                    "session={} path_revision={} changed identity or payload",
                    path.session_id, path.path_revision
                ),
            };
            self.status.message = format!("同修订路径变化已丢弃：{error}");
            self.publish_status(status_tx);
            return Err(error);
        }

        let VersionedPath {
            path: planner_path,
            route_kind,
            path_yaws,
            execution_identity,
            path_revision,

            ..
        } = path;
        if self.awaiting_session_path {
            self.planner.configure_execution_goal(&session.goal, session.options)?;
        }
        self.planner.replace_typed_execution_path(
            planner_path.clone(),
            path_yaws.clone(),
            session.goal.clone(),
        )?;
        self.awaiting_session_path = false;
        self.current_path_revision = path_revision;
        self.current_route_kind = route_kind;

        self.current_path = Some((planner_path, path_yaws));
        self.current_execution_identity = Some(Arc::new(execution_identity));
        self.status.surface_replan_feedback = None;
        // 等待预算在真正执行有效控制步骤后清除，不能靠反复投递无效新路径续期。
        self.status.phase = SchedulerPhase::Controlling;
        self.status.path_revision = self.current_path_revision;
        self.status.no_feasible_path = false;
        self.status.message = "versioned path accepted".into();
        self.publish_status(status_tx);
        Ok(())
    }

    async fn cancel(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        session_id: &str,
    ) -> Result<(), RuntimeError> {
        let Some(session) = self.session.as_ref() else {
            return Ok(());
        };
        if !session_id.is_empty() && session_id.trim() != session.session_id {
            return Err(RuntimeError::Busy(format!(
                "cancel session {} does not match active {}",
                session_id.trim(),
                session.session_id
            )));
        }
        self.status.phase = SchedulerPhase::Cancelling;
        self.status.surface_replan_feedback = None;
        self.status.message = "取消目标，释放租约".into();
        self.publish_status(status_tx);
        self.finish_terminal(status_tx, TerminalOutcome::Cancelled, SchedulerPhase::Idle)
            .await
    }

    async fn control_tick(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.control_tick", self.status.pose_source_timestamp_ns);
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.control_tick.enter", self.status.pose_source_timestamp_ns, None, 0);
        if let Some(reason) = self.stale_input_reason(Instant::now()) {
            #[cfg(feature = "surface-recovery-qualification")]
            if self.qualification_timing.as_ref().is_some_and(|sink| sink.enabled()) {
                // Classify only the existing result. Never evaluate staleness again.
                let mut bits = 0;
                if reason.contains("pose") { bits |= 1; }
                if reason.contains("costmap") { bits |= 2; }
                if reason.contains("recent") { bits |= 4; }
                if bits == 0 { bits = 8; }
                self.qualification_snapshot("scheduler.control_tick.stale", self.status.pose_source_timestamp_ns, Some(true), bits);
            }
            return self.wait_for_fresh_input(status_tx, reason).await;
        }
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.control_tick.fresh", self.status.pose_source_timestamp_ns, Some(false), 0);
        let mut step = {
            #[cfg(feature = "surface-recovery-qualification")]
            let _qualification_span = self.qualification_span("scheduler.planner.step", self.status.pose_source_timestamp_ns);
            self.planner.step()
        };
        if self.current_route_kind == RouteKind::Approach {
            let near_endpoint = match (&self.current_path,&self.latest_pose) {
                (Some((ExecutionPath::Surface(path),_)),Some(pose)) => path.poses.last()
                    .is_some_and(|end|pose.pose.position.distance(end.pose.position)<=0.15),
                _ => false,
            };
            if matches!(&step,PlannerStep::Succeeded) || near_endpoint
                && matches!(&step,PlannerStep::Command(_) | PlannerStep::Pending(_)) {
                step=PlannerStep::ReplanRequired("逼近路线已到近端，原目标仍活动，等待新观测地面与重规划".into());
            }
        }
        let intent = self.wrap_local_intent(step)?;
        // This evidence belongs to the exact delivered step and accepted path.
        // It is not reconstructed from the scheduler's latest map/pose inputs.
        self.status.surface_replan_feedback = Some(SurfaceReplanFeedback {
            path_identity: Arc::clone(&intent.identity),
            rejected_context: None,
            current_context: intent.execution_context.clone(),
            zero_confirmed_rejection: false,
            rejection_reason: None,
        });
        match intent.payload {
            LocalExecutionPayload::Command(twist) => {
                let wait_reason = self.planner.execution_wait_reason();
                if !self.apply_controller_twist_with_bound(status_tx, twist, intent.dispatch_bound).await? {
                    self.path_wait_started_at.get_or_insert_with(Instant::now);
                    return self.check_path_wait_timeout(status_tx).await;
                }
                if twist == Twist2::ZERO {
                    // 零速已确认不等于规划恢复；同一轮持续等待不能被每拍归零续期。
                    // 正常制动后会继续运动或到达；缺少原因也不能无限清除等待期限。
                    self.path_wait_started_at.get_or_insert_with(Instant::now);
                    self.status.message = wait_reason.filter(|reason| !reason.trim().is_empty())
                        .unwrap_or_else(|| "控制器持续零速，等待可执行运动".into());
                    self.publish_status(status_tx);
                    return self.check_path_wait_timeout(status_tx).await;
                }
                if self.awaiting_session_path {
                    self.check_path_wait_timeout(status_tx).await
                } else {
                    self.path_wait_started_at = None;
                    Ok(())
                }
            }
            LocalExecutionPayload::Pending(failures) => {
                // 只是下一结果未出时，上一指令在原许可期内继续执行；
                // 足式机器人一两拍零速停不下来，只会打断步态。
                if let Some(held) = self.planner.held_command() {
                    self.apply_controller_twist(status_tx, held).await?;
                    return Ok(());
                }
                self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
                self.path_wait_started_at.get_or_insert_with(Instant::now);
                // Rebuild from this tick's evidence; never append to the previous status.
                self.status.message = self.planner.execution_wait_reason()
                    .filter(|reason| !reason.trim().is_empty())
                    .unwrap_or_else(|| "局部轨迹待定，保持零速等待".into());
                if !failures.is_empty() {
                    self.status.message = format!("{}: {failures:?}", self.status.message);
                }
                self.publish_status(status_tx);
                self.check_path_wait_timeout(status_tx).await
            }
            LocalExecutionPayload::ReplanRequired(reason) => {
                self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
                // A same-valued ZERO receipt leaves the worker's exact actual
                // input versions current. Preserve that snapshot before stop
                // retires control epoch; stop does not change source/pose/ground.
                // A changed speed/input still yields None and normal planning.
                let stopped_context=self.planner.refresh_execution_context(Twist2::ZERO);
                self.planner.stop();
                self.path_wait_started_at.get_or_insert_with(Instant::now);
                self.status.phase = SchedulerPhase::AwaitingPath;
                // Never relabel the rejection's pre-braking context as stopped.
                // `stopped_context` exists only if ZERO was already consumed
                // with these exact actual source/pose/applied/boundary versions.
                self.status.surface_replan_feedback = Some(SurfaceReplanFeedback {
                    path_identity: Arc::clone(&intent.identity),
                    rejected_context: intent.execution_context,
                    current_context: stopped_context,
                    zero_confirmed_rejection: true,
                    rejection_reason: Some(reason.clone()),
                });
                self.status.message = format!("路径失效，零速已执行，等待更新路径: {reason}");
                self.publish_status(status_tx);
                Ok(())
            }
            LocalExecutionPayload::Succeeded if self.awaiting_session_path => {
                self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
                self.check_path_wait_timeout(status_tx).await
            }
            LocalExecutionPayload::Succeeded => {
                self.finish_terminal(status_tx, TerminalOutcome::Succeeded, SchedulerPhase::Idle)
                    .await
            }
            LocalExecutionPayload::Failed(reason) => {
                let qualified_cause = format!("PlannerFailed cause=LocalPlanner: {reason}");
                self.status.message = qualified_cause.clone();
                self.publish_status(status_tx);
                self.finish_terminal(
                    status_tx,
                    TerminalOutcome::PlannerFailed,
                    SchedulerPhase::Failed,
                )
                .await?;
                self.status.message = qualified_cause;
                self.publish_status(status_tx);
                Ok(())
            }
        }
    }

    /// 运动和等待重规划的零速共用同一提交与回执校验路径。
    async fn apply_controller_twist(
        &mut self, status_tx:&watch::Sender<SchedulerStatus>, twist:Twist2,
    )->Result<(),RuntimeError> {
        self.apply_controller_twist_with_bound(status_tx,twist,None).await.map(|_|())
    }

    // False means the selected command expired and an ordinary zero was confirmed.
    async fn apply_controller_twist_with_bound(
        &mut self, status_tx:&watch::Sender<SchedulerStatus>, twist:Twist2,
        bound:Option<CommandDispatchBound>,
    )->Result<bool,RuntimeError> {
        self.send_controller_twist_with_bound(status_tx,twist,bound,true).await
    }

    async fn send_controller_twist_with_bound(
        &mut self, status_tx:&watch::Sender<SchedulerStatus>, twist:Twist2,
        mut bound:Option<CommandDispatchBound>, publish_message:bool,
    )->Result<bool,RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.apply_controller_twist", self.status.pose_source_timestamp_ns);
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some(sink) = &self.qualification_timing {
            sink.motion("scheduler.apply.requested", self.status.pose_source_timestamp_ns,
                [twist.linear.x, twist.linear.y, twist.angular_z]);
        }
        if twist!=Twist2::ZERO {
            if let Some(source)=self.obstacle_source_bound {
                bound=Some(bound.map_or(source,|candidate|candidate.earliest(source)));
            }
        }
        let mut applied_twist=twist;
        let mut expired=false;
        loop {
            #[cfg(feature = "surface-recovery-qualification")]
            self.qualification_snapshot("scheduler.motion.submit.enter", self.status.pose_source_timestamp_ns, None, 0);
            let outcome = {
                #[cfg(feature = "surface-recovery-qualification")]
                let _qualification_span = self.qualification_span("scheduler.motion.submit", self.status.pose_source_timestamp_ns);
                #[cfg(feature = "surface-recovery-qualification")]
                if let Some(sink) = &self.qualification_timing {
                    sink.motion("scheduler.motion.submit.command", self.status.pose_source_timestamp_ns,
                        [applied_twist.linear.x, applied_twist.linear.y, applied_twist.angular_z]);
                    sink.bound("scheduler.motion.submit.bound", self.status.pose_source_timestamp_ns, bound);
                }
                self.motion.velocity_bounded(applied_twist,bound).await
            };
            #[cfg(feature = "surface-recovery-qualification")]
            if let Some(sink) = &self.qualification_timing {
                sink.mark(match &outcome {
                    Ok(BoundedTwistOutcome::Applied) => "scheduler.motion.submit.return.applied",
                    Ok(BoundedTwistOutcome::ExpiredNotApplied) => "scheduler.motion.submit.return.expired",
                    Err(_) => "scheduler.motion.submit.return.error",
                }, self.status.pose_source_timestamp_ns);
            }
            match outcome {
                Ok(BoundedTwistOutcome::Applied)=>{},
                Ok(BoundedTwistOutcome::ExpiredNotApplied) if bound.is_some()=>{
                    applied_twist=Twist2::ZERO;bound=None;expired=true;continue;
                },
                Ok(BoundedTwistOutcome::ExpiredNotApplied)=>{
                    return Err(self.fail_closed(status_tx,RuntimeError::Motion("unbounded zero submission reported observation expiry".into())).await);
                },
                Err(error)=>return Err(self.motion_failed(status_tx,error).await),
            };
            self.status.applied_commands=self.status.applied_commands.saturating_add(1);
            self.status.last_applied_twist=Some(applied_twist);
            self.planner.observe_measured_motion(self.motion.measured_motion());
            self.planner.observe_applied_command(applied_twist);
            if let Some(feedback)=&mut self.status.surface_replan_feedback {
                if feedback.current_context.as_ref().is_some_and(|context|
                    [context.applied.linear.x,context.applied.linear.y,context.applied.angular_z].map(f64::to_bits)
                        != [applied_twist.linear.x,applied_twist.linear.y,applied_twist.angular_z].map(f64::to_bits)) {
                    feedback.current_context=None;
                }
            }
            if publish_message { self.status.message=if expired {"执行来源凭据在下发前已过期，零速已执行，等待重新计算".into()}else{"速度请求已接受".into()}; }
            self.publish_status(status_tx);
            return Ok(!expired);
        }
    }

    fn wrap_local_intent(
        &mut self,
        step: PlannerStep,
    ) -> Result<LocalExecutionCommandIntent, RuntimeError> {
        let identity = Arc::clone(self.current_execution_identity.as_ref().ok_or_else(|| {
            RuntimeError::Planner("local planner produced an intent without an active path".into())
        })?);
        // An internal planner initialization ZERO is not an APPLIED receipt.
        let execution_context=self.status.last_applied_twist
            .and_then(|applied|self.planner.take_step_execution_context(applied));
        let candidate_bound=self.planner.take_command_dispatch_bound();
        let dispatch_bound=if matches!(&step,PlannerStep::Command(v) if *v!=Twist2::ZERO) {candidate_bound}else{None};
        let payload = match step {
            PlannerStep::Command(twist) => LocalExecutionPayload::Command(twist),
            PlannerStep::Pending(failures) => LocalExecutionPayload::Pending(failures),
            PlannerStep::Succeeded => LocalExecutionPayload::Succeeded,
            PlannerStep::Failed(reason) => LocalExecutionPayload::Failed(reason),
            PlannerStep::ReplanRequired(reason) => LocalExecutionPayload::ReplanRequired(reason),
        };
        Ok(LocalExecutionCommandIntent {
            identity,
            payload,
            dispatch_bound,
            execution_context,
        })
    }

    async fn session_tick(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        if self.status.phase == SchedulerPhase::Controlling {
            return self.control_tick(status_tx).await;
        }
        if self.status.phase != SchedulerPhase::AwaitingPath {
            return Ok(());
        }
        if self.current_path_revision > 0 {
            if let Some(reason) = self.stale_input_reason(Instant::now()) {
                return self.wait_for_fresh_input(status_tx, reason).await;
            }
        }
        self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
        self.check_path_wait_timeout(status_tx).await?;
        if self.status.phase == SchedulerPhase::AwaitingPath {
            // Preserve the original stale/deadline checks and their ordering.
            // Worker getter is a short snapshot read, never a controller step.
            let context=self.status.last_applied_twist
                .and_then(|applied|self.planner.refresh_execution_context(applied));
            let mut changed=false;
            if let Some(feedback)=&mut self.status.surface_replan_feedback {
                if feedback.zero_confirmed_rejection
                    && self.current_execution_identity.as_ref().is_some_and(|identity|
                        identity.as_ref()==feedback.path_identity.as_ref())
                    && feedback.current_context != context {
                    feedback.current_context=context;
                    changed=true;
                }
            }
            if changed {self.publish_status(status_tx);}
        }
        Ok(())
    }

    async fn wait_for_fresh_input(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        reason: String,
    ) -> Result<(), RuntimeError> {
        // 重复过期、零速回执和新路径都不能重置原等待起点。
        self.path_wait_started_at.get_or_insert_with(Instant::now);
        if self.lease_active {
            self.apply_controller_twist(status_tx, Twist2::ZERO).await?;
        }
        self.status.message = format!("等待新鲜导航输入: {reason}");
        self.publish_status(status_tx);
        self.check_path_wait_timeout(status_tx).await
    }

    async fn check_path_wait_timeout(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        let Some(started_at) = self.path_wait_started_at else {
            let error = RuntimeError::Planner("awaiting path without a deadline".into());
            return Err(self.fail_closed(status_tx, error).await);
        };
        if self.config.path_wait_timeout.is_zero()
            || Instant::now().duration_since(started_at) < self.config.path_wait_timeout {
            return Ok(());
        }
        let timeout_message = format!(
            "planner path timeout after {}ms; fencing",
            self.config.path_wait_timeout.as_millis(),
        );
        self.status.message = if self.status.message.trim().is_empty() {
            timeout_message
        } else {
            format!("{timeout_message}; {}", self.status.message)
        };
        self.publish_status(status_tx);
        self.finish_terminal(
            status_tx,
            TerminalOutcome::PlannerFailed,
            SchedulerPhase::Failed,
        )
        .await
    }

    async fn motion_failed(&mut self, status_tx: &watch::Sender<SchedulerStatus>, error: MotionPortError) -> RuntimeError {
        if error.code == Some(ErrorCode::LeaseRevoked) {
            self.lease_active = false;
            self.status.message = error.message.clone();
            let _ = self.finish_terminal(status_tx, TerminalOutcome::Stopped, SchedulerPhase::Idle).await;
            RuntimeError::Motion(error.message)
        } else { self.fail_closed(status_tx, RuntimeError::Motion(error.message)).await }
    }

    async fn fail_closed(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        original: RuntimeError,
    ) -> RuntimeError {
        let cause_kind = match &original {
            RuntimeError::InvalidInput { .. } => "InvalidInput",
            RuntimeError::Busy(_) => "Busy",
            RuntimeError::PathRejected { .. } => "PathRejected",
            RuntimeError::PathMutation { .. } => "PathMutation",
            RuntimeError::Motion(_) => "Motion",
            RuntimeError::LeaseNotGranted(_) => "LeaseNotGranted",
            RuntimeError::Planner(_) => "Planner",
            RuntimeError::StaleInput(_) => "StaleInput",
            RuntimeError::MapSourceInvalidated(_) => "MapSourceInvalidated",
            RuntimeError::SchedulerStopped => "SchedulerStopped",
        };
        let qualified_cause = format!("SafetyFailed cause={cause_kind}: {original}");
        self.status.message = qualified_cause.clone();
        match self
            .finish_terminal(
                status_tx,
                TerminalOutcome::SafetyFailed,
                SchedulerPhase::Failed,
            )
            .await
        {
            Ok(()) => {
                self.status.message = qualified_cause;
                self.publish_status(status_tx);
                original
            }
            // finish_terminal 已将首因和停车失败一并发布，不能在返回时重复拼接。
            Err(fence_error) => RuntimeError::Motion(format!(
                "{original}; explicit fence also failed: {fence_error}"
            )),
        }
    }

    async fn finish_terminal(
        &mut self,
        status_tx: &watch::Sender<SchedulerStatus>,
        terminal: TerminalOutcome,
        phase_after: SchedulerPhase,
    ) -> Result<(), RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let _qualification_span = self.qualification_span("scheduler.finish_terminal", self.status.pose_source_timestamp_ns);
        #[cfg(feature = "surface-recovery-qualification")]
        self.qualification_snapshot("scheduler.finish_terminal.enter", self.status.pose_source_timestamp_ns, None, 0);
        // 上层可能只消费首次终态；先保留真实原因，不能等围栏完成后再补发。
        let failure_reason = matches!(terminal, TerminalOutcome::PlannerFailed | TerminalOutcome::SafetyFailed)
            .then(|| self.status.message.clone());
        if terminal == TerminalOutcome::Shutdown && !self.lease_active && self.session.is_none() && self.status.last_terminal.is_some() {
            self.status.motion_sink_ready=false;
            self.publish_status(status_tx);
            return Ok(());
        }
        self.status.surface_replan_feedback = None;
        self.current_execution_identity = None;

        {
            #[cfg(feature = "surface-recovery-qualification")]
            let _qualification_span = self.qualification_span("scheduler.finish_terminal.planner_stop", self.status.pose_source_timestamp_ns);
            self.planner.stop();
        }
        if self.lease_active {
            if let Err(error) = self.motion.release().await {
                self.fail_status(status_tx, &error.message);
                return Err(RuntimeError::Motion(error.message));
            }
            self.lease_active = false;
            self.planner.observe_applied_command(Twist2::ZERO);
            self.status.last_applied_twist = Some(Twist2::ZERO);
        }

        {
            #[cfg(feature = "surface-recovery-qualification")]
            let _qualification_span = self.qualification_span("scheduler.finish_terminal.planner_stop", self.status.pose_source_timestamp_ns);
            self.planner.stop();
        }
        self.session = None;
        self.current_path_revision = 0;

        self.current_path = None;
        self.current_execution_identity = None;

        self.path_wait_started_at = None;
        self.status.phase = phase_after;
        self.status.session_id = None;
        self.status.path_revision = 0;
        self.status.last_terminal = Some(terminal);
        self.status.message = if terminal == TerminalOutcome::Stopped {
            "已被停止".into()
        } else { format!("{terminal:?}，租约已释放") };
        if let Some(reason) = failure_reason {
            self.status.message = format!("{reason}; {}", self.status.message);
        }
        if terminal == TerminalOutcome::Shutdown {
            self.status.motion_sink_ready = false;
        }
        self.publish_status(status_tx);
        Ok(())
    }

    fn fail_status(&mut self, status_tx: &watch::Sender<SchedulerStatus>, reason: &str) {
        self.status.surface_replan_feedback = None;
        self.status.phase = SchedulerPhase::Failed;
        self.status.motion_sink_ready = false;
        self.status.last_terminal = None;
        self.status.message = reason.to_string();
        self.publish_status(status_tx);
    }

    fn publish_status(&self, status_tx: &watch::Sender<SchedulerStatus>) {
        let mut status = self.status.clone();
        if self.current_route_kind == RouteKind::Approach && status.last_terminal.is_none()
            && matches!(status.phase,SchedulerPhase::Controlling | SchedulerPhase::AwaitingPath) {
            status.message=format!("正在逐步逼近目标；{}",status.message);
        }
        if status.phase == SchedulerPhase::Controlling && status.last_terminal.is_none() {
            status.local_path = self.planner.local_path_for_display();
        } else {
            status.local_path.clear();
        }
        status.remaining_distance_m = match (&self.current_path, &self.latest_pose) {
            (Some((path, _)), Some(pose)) => {
                let points: Vec<[f64; 3]> = match path {
                    ExecutionPath::Planar(path) => path.points.iter().map(|p| [p.x, p.y, 0.0]).collect(),
                    ExecutionPath::Surface(path) => path.poses.iter().map(|p| [p.pose.position.x, p.pose.position.y, p.pose.position.z]).collect(),
                };
                let z = if matches!(path, ExecutionPath::Surface(_)) { pose.pose.position.z } else { 0.0 };
                remaining_route_distance(&points, [pose.pose.position.x, pose.pose.position.y, z])
            }
            _ => 0.0,
        };
        status_tx.send_replace(status);
    }
}

/// 从路线中离当前位姿最近的投影点开始累计，保留弯折与三维高度。
fn remaining_route_distance(points: &[[f64; 3]], position: [f64; 3]) -> f64 {
    let distance = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).map(|(a, b)| (a-b).powi(2)).sum::<f64>().sqrt();
    let mut best = f64::INFINITY;
    let mut remaining = 0.0;
    let mut tail = 0.0;
    for segment in points.windows(2).rev() {
        let start = segment[0];
        let end = segment[1];
        let delta = [end[0]-start[0], end[1]-start[1], end[2]-start[2]];
        let length = distance(start, end);
        let fraction = if length > 0.0 {
            ((0..3).map(|i| (position[i]-start[i])*delta[i]).sum::<f64>() / length.powi(2)).clamp(0.0, 1.0)
        } else { 0.0 };
        let projected = [start[0]+fraction*delta[0], start[1]+fraction*delta[1], start[2]+fraction*delta[2]];
        let deviation = distance(position, projected);
        if deviation < best {
            best = deviation;
            remaining = tail + length*(1.0-fraction);
        }
        tail += length;
    }
    remaining
}

/// Cloneable command handle for a spawned scheduler.
#[derive(Clone)]
pub struct NavSchedulerHandle {
    input_tx: mpsc::Sender<InputCommand>,
    urgent_tx: mpsc::Sender<UrgentCommand>,
    status_rx: watch::Receiver<SchedulerStatus>,
    #[cfg(feature = "surface-recovery-qualification")]
    qualification_timing: Option<qualification_timing::TraceSink>,
}

impl NavSchedulerHandle {
    /// Observer explicitly attached before spawn; None is the silent default.
    #[cfg(feature = "surface-recovery-qualification")]
    pub fn qualification_timing(&self) -> Option<&qualification_timing::TraceSink> {
        self.qualification_timing.as_ref()
    }

    /// Acquire a lease for a new session.
    pub async fn start_session(&self, session: NavSession) -> Result<(), RuntimeError> {
        self.send_input(InputKind::StartSession(session)).await
    }

    /// Read an existing paired actual publication through the original input queue.
    /// None means not ready; this does not create an epoch or execute geometry.
    pub async fn capture_planning_actual(&self, goal_identity: PathExecutionIdentity)
        -> Result<Option<Arc<ActualExecutionContext>>, RuntimeError> {
        let (reply, result) = oneshot::channel();
        self.send_input(InputKind::CapturePlanningActual(goal_identity, reply)).await?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)
    }

    /// Submit a correlated, immutable path revision.
    pub async fn submit_path(&self, path: VersionedPath) -> Result<(), RuntimeError> {
        self.send_input(InputKind::SubmitPath(path)).await
    }

    /// 同进程当前观察入口；调用时刻保留至队列消费，传输来源须用采集时刻入口。
    pub async fn update_odometry(&self, odometry: PoseStamped) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry, None, Instant::now()))
            .await
    }

    /// 保留来源期限的同进程当前观察入口；传输来源须另提供实际采集时刻。
    pub async fn update_odometry_with_deadline(
        &self,
        odometry: PoseStamped,
        valid_until: Instant,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry, Some(valid_until), Instant::now()))
            .await
    }

    /// 生产位姿保留实际采集单调时刻，停车复核不能把队列消费当新采样。
    pub async fn update_odometry_with_acquisition(
        &self, odometry: PoseStamped, acquired_at: Instant, valid_until: Instant,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateOdometry(odometry, Some(valid_until), acquired_at)).await
    }

    /// 生产位姿按来源钟动态判龄；采集单调时刻保留真实墙钟年龄，不能续期执行许可。
    pub async fn update_odometry_with_source_age(
        &self, odometry: PoseStamped, acquired_at: Instant, max_source_age: Duration,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateSourceOdometry(odometry, acquired_at, max_source_age)).await
    }

    /// Update the planner costmap with its real revision and source timestamp.
    pub async fn update_costmap(
        &self,
        observation: CostmapObservation,
    ) -> Result<(), RuntimeError> {
        self.update_costmap_received(observation, Instant::now())
            .await
    }

    /// Refresh only current obstacle evidence and its admitted source deadline.
    /// Persistent terrain receipts and planning geometry versions are unchanged.
    pub async fn update_obstacle_source(&self,map:Option<ExecutionMap>,available:bool,
        bound:Option<CommandDispatchBound>)->Result<(),RuntimeError> {
        self.send_input(InputKind::UpdateObstacleSource(map,available,bound,(!available).then_some(None))).await
    }

    /// 原子交付现有障碍来源资格与同帧原始注册输入，不增加另一个时钟。
    pub async fn update_registered_obstacle_source(&self, map:Option<ExecutionMap>, available:bool,
        bound:Option<CommandDispatchBound>, frame:Option<Arc<RegisteredFrame>>)->Result<(),RuntimeError> {
        self.send_input(InputKind::UpdateObstacleSource(map,available,bound,Some(frame))).await
    }

    /// 保留实际地图接收时刻，表面构建完成或队列等待不能延长输入有效期。
    pub async fn update_costmap_received(
        &self,
        observation: CostmapObservation,
        received_at: Instant,
    ) -> Result<(), RuntimeError> {
        self.send_input(InputKind::UpdateCostmap(observation, received_at))
            .await
    }

    /// Cancel the active session and wait for a verified fence.
    pub async fn cancel(&self, session_id: &str) -> Result<(), RuntimeError> {
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::Cancel {
                session_id: session_id.trim().to_string(),
                reply,
            })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some(sink) = &self.qualification_timing {
            sink.mark("scheduler.cancel.send.accepted", 0);
        }
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
    }

    /// 明确撤销地图来源；活跃会话等待失败停车围栏，空闲状态不申请运动租约。
    pub async fn invalidate_map(&self, reason: impl Into<String>) -> Result<(), RuntimeError> {
        self.invalidate_map_received(reason, Instant::now(), MapInvalidationKind::Invalidated).await
    }

    /// 使用来源失效事件的原接收时刻，不把排队处理时刻当作来源水位。
    pub async fn invalidate_map_received(
        &self,
        reason: impl Into<String>,
        received_at: Instant,
        kind: MapInvalidationKind,
    ) -> Result<(), RuntimeError> {
        if received_at > Instant::now() {
            return Err(RuntimeError::InvalidInput {
                field: "map.invalidated_at",
                reason: "地图失效事件的接收时刻不能位于未来".into(),
            });
        }
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::InvalidateMap {
                reason: reason.into(),
                invalidated_at: received_at,
                kind,
                reply,
            })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
    }

    /// Stop the scheduler and fence any active lease.
    pub async fn shutdown(&self) -> Result<(), RuntimeError> {
        let (reply, result) = oneshot::channel();
        self.urgent_tx
            .send(UrgentCommand::Shutdown { reply })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        result.await.map_err(|_| RuntimeError::SchedulerStopped)?
    }

    /// 全局拥有方确认无法继续逼近时沿当前会话的原停车围栏结束。
    pub async fn report_planning_failure(&self,goal_id:String,key:GoalKey,message:String)->Result<(),RuntimeError> {
        self.send_input(InputKind::PlanningDiagnostic(goal_id,key,message,true,false)).await
    }
    /// 规划诊断按原目标身份关联，仅更新等待路径阶段，不能覆盖执行或终态。
    pub async fn report_planning_diagnostic(&self,goal_id:String,key:GoalKey,message:String,no_feasible_path:bool)->Result<(),RuntimeError> {
        self.send_input(InputKind::PlanningDiagnostic(goal_id,key,message,false,no_feasible_path)).await
    }

    /// Read the latest scheduler status.
    #[must_use]
    pub fn status(&self) -> SchedulerStatus {
        self.status_rx.borrow().clone()
    }

    /// Subscribe to scheduler status changes.
    #[must_use]
    pub fn subscribe_status(&self) -> watch::Receiver<SchedulerStatus> {
        self.status_rx.clone()
    }

    async fn send_input(&self, kind: InputKind) -> Result<(), RuntimeError> {
        #[cfg(feature = "surface-recovery-qualification")]
        let qualification_input = self.qualification_timing.as_ref().map(|sink| {
            let (phase, source) = match &kind {
                InputKind::StartSession(_) => ("scheduler.send.start_session", 0),
                InputKind::SubmitPath(_) => ("scheduler.send.path", 0),
                InputKind::UpdateOdometry(pose, _, _) | InputKind::UpdateSourceOdometry(pose, _, _) => ("scheduler.send.pose", pose.stamp.nanos),
                InputKind::UpdateCostmap(map, _) => ("scheduler.send.map", map.source_timestamp_ns),
                InputKind::UpdateObstacleSource(_, _, _, _) => ("scheduler.send.recent", 0),
                InputKind::PlanningDiagnostic(_, _, _, _, _) => ("scheduler.send.planning_diagnostic", 0),
                InputKind::CapturePlanningActual(_, _) => ("scheduler.send.capture_actual", 0),
            };
            if let InputKind::UpdateObstacleSource(_, _, bound, _) = &kind {
                sink.bound("scheduler.input.send.recent_bound", 0, *bound);
            }
            sink.mark("scheduler.input.send.begin", source);
            (sink.clone(), source, sink.span(phase, source))
        });
        let (reply, result) = oneshot::channel();
        self.input_tx
            .send(InputCommand { kind, reply })
            .await
            .map_err(|_| RuntimeError::SchedulerStopped)?;
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some((sink, source, _span)) = &qualification_input {
            // Successful send means queued; the actor may already have dequeued it.
            sink.mark("scheduler.input.send.accepted", *source);
        }
        let reply = result.await.map_err(|_| RuntimeError::SchedulerStopped);
        #[cfg(feature = "surface-recovery-qualification")]
        if let Some((sink, source, _span)) = &qualification_input {
            sink.mark(match &reply {
                Ok(Ok(())) => "scheduler.input.reply.ok",
                Ok(Err(_)) => "scheduler.input.reply.error",
                Err(_) => "scheduler.input.reply.closed",
            }, *source);
        }
        reply?
    }
}

/// Spawn one scheduler task and return its command handle and join handle.
#[must_use]
pub fn spawn_scheduler<M, P>(
    scheduler: NavScheduler<M, P>,
) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>)
where
    M: NavMotionPort,
    P: NavPlanner,
{
    #[cfg(feature = "surface-recovery-qualification")]
    let qualification_timing = scheduler.qualification_timing.clone();
    let (input_tx, input_rx) = mpsc::channel(INPUT_CAPACITY);
    let (urgent_tx, urgent_rx) = mpsc::channel(URGENT_CAPACITY);
    let (status_tx, status_rx) = watch::channel(scheduler.status.clone());
    let handle = NavSchedulerHandle {
        input_tx,
        urgent_tx,
        status_rx,
        #[cfg(feature = "surface-recovery-qualification")]
        qualification_timing,
    };
    let task = tokio::spawn(scheduler.run(input_rx, urgent_rx, status_tx));
    (handle, task)
}

/// Connect the production private-UDS guard client and spawn the qualified
/// LocalPlanner scheduler. A successful transport connection makes the motion
/// sink ready; individual sessions still require an admitted guard proof.
pub async fn spawn_production_scheduler(
    socket_path: impl Into<PathBuf>,
    config: SchedulerConfig,
) -> Result<(NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>), RuntimeError> {
    spawn_production_scheduler_with_backend(
        socket_path,
        config,
        LocalExecutionBackendKind::BasicInProcess,
    )
    .await
}

/// Connect the production guard and freeze one local execution backend into
/// the scheduler-owned task for its complete lifetime.
pub async fn spawn_production_scheduler_with_backend(
    socket_path: impl Into<PathBuf>,
    config: SchedulerConfig,
    backend_kind: LocalExecutionBackendKind,
) -> Result<(NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>), RuntimeError> {
    validate_config(config)?;
    let backend = production_local_backend(backend_kind)?;
    let motion = MotionGuardClient::connect_with_config(production_client_config(socket_path))
        .await
        .map_err(|error| RuntimeError::Motion(error.to_string()))?;
    let scheduler = NavScheduler::new(motion, backend, config)?;
    Ok(spawn_scheduler(scheduler))
}

/// 显式构造三维局部后端，标定非法时先拒绝，不能为失败配置连接运动端。
pub async fn spawn_production_surface_scheduler(
    socket_path: impl Into<PathBuf>,
    config: SchedulerConfig,
    mut surface_config: SurfaceFollowerConfig,
) -> Result<(NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>), RuntimeError> {
    validate_config(config)?;
    // 先校验标定，再连接原守卫查询真实设备限制，不创建第二个设备执行器。
    let _ = SurfaceFollower::new(surface_config.clone())?;
    let mut motion = MotionGuardClient::connect_with_config(production_client_config(socket_path))
        .await
        .map_err(|error| RuntimeError::Motion(error.to_string()))?;
    let readiness=motion.status().await.map_err(|error|RuntimeError::Motion(error.to_string()))?;
    surface_config.command_limits=readiness.command_limits;
    let backend = production_surface_backend(surface_config)?;
    Ok(spawn_scheduler(NavScheduler::new(motion, backend, config)?))
}

fn production_surface_backend(config: SurfaceFollowerConfig) -> Result<planner_worker::PlannerWorker, RuntimeError> {
    planner_worker::PlannerWorker::new(RouteFollower::new(config)?)
}

/// Construct the unchanged production backend for opt-in cross-crate qualification.
#[cfg(feature = "surface-recovery-qualification")]
pub fn qualification_surface_backend(
    config: SurfaceFollowerConfig,
) -> Result<impl NavPlanner, RuntimeError> {
    production_surface_backend(config)
}

/// 静态地图资格测试仍使用真实异步执行器，显式声明其输入并非注册扫描原帧。
#[cfg(feature = "surface-recovery-qualification")]
pub fn qualification_static_surface_backend(
    config: SurfaceFollowerConfig,
) -> Result<impl NavPlanner, RuntimeError> {
    let mut follower = SurfaceFollower::new(config)?;
    follower.enable_author_static_map_fixture(true);
    planner_worker::PlannerWorker::new(follower)
}

struct DeferredControl {
    publication: watch::Receiver<()>,
    deadline: Instant,
    identity: Arc<PathExecutionIdentity>,
    queued_inputs: Option<usize>,
    finish_wait: bool,
}

impl DeferredControl {
    // True ends the wait even if the worker still has no current result. Channel
    // closure must resolve once, not repeatedly subscribe to a closed notifier.
    async fn ready(&mut self) -> bool {
        tokio::select! {
            biased;
            _ = tokio::time::sleep_until(self.deadline) => true,
            changed = self.publication.changed() => changed.is_err(),
        }
    }
}

enum SchedulerEvent {
    Heartbeat, Input(Option<InputCommand>), Tick, Publication(bool), ControlReady }

fn check_pose_data_age(source_ns: i64, maximum: Duration) -> Result<(), RuntimeError> {
    let source = u64::try_from(source_ns).ok().filter(|v| *v > 0)
        .ok_or_else(|| RuntimeError::StaleInput("pose source timestamp invalid".into()))?;
    let age = nuwax_nav_types_messages::source_time::data_age_ns_at(source,
        command_dispatch::unix_ns().unwrap_or(0)).map_err(RuntimeError::StaleInput)?;
    if u128::from(age) >= maximum.as_nanos() {
        return Err(RuntimeError::StaleInput(format!(
            "pose source deadline elapsed (source_timestamp_ns={source_ns}, source_age_ms={})", age / 1_000_000)));
    }
    Ok(())
}

struct InputCommand {
    kind: InputKind,
    reply: oneshot::Sender<Result<(), RuntimeError>>,
}

enum InputKind {
    StartSession(NavSession),
    SubmitPath(VersionedPath),
    UpdateOdometry(PoseStamped, Option<Instant>, Instant),
    UpdateSourceOdometry(PoseStamped, Instant, Duration),
    UpdateCostmap(CostmapObservation, Instant),
    UpdateObstacleSource(Option<ExecutionMap>,bool,Option<CommandDispatchBound>,Option<Option<Arc<RegisteredFrame>>>),
    PlanningDiagnostic(String, GoalKey, String, bool, bool),
    CapturePlanningActual(PathExecutionIdentity, oneshot::Sender<Option<Arc<ActualExecutionContext>>>),
}

enum UrgentCommand {
    Cancel {
        session_id: String,
        reply: oneshot::Sender<Result<(), RuntimeError>>,
    },
    InvalidateMap {
        reason: String,
        invalidated_at: Instant,
        kind: MapInvalidationKind,
        reply: oneshot::Sender<Result<(), RuntimeError>>,
    },
    Shutdown {
        reply: oneshot::Sender<Result<(), RuntimeError>>,
    },
}

fn validate_config(config: SchedulerConfig) -> Result<(), RuntimeError> {
    if config.tick_interval.is_zero() {
        return Err(RuntimeError::InvalidInput {
            field: "tick_interval",
            reason: "must be positive".into(),
        });
    }
    Ok(())
}

fn validate_source_timestamp(
    field: &'static str,
    source_timestamp_ns: i64,
    previous: Option<PlannerInputReceipt>,
) -> Result<(), RuntimeError> {
    if source_timestamp_ns <= 0 {
        return Err(RuntimeError::InvalidInput {
            field,
            reason: "must be positive".into(),
        });
    }
    if previous.is_some_and(|receipt| source_timestamp_ns <= receipt.source_timestamp_ns) {
        return Err(RuntimeError::StaleInput(format!(
            "{field}={source_timestamp_ns} did not advance beyond {}",
            previous.map_or(0, |receipt| receipt.source_timestamp_ns),
        )));
    }
    Ok(())
}

fn receipt_age_ms(receipt: Option<PlannerInputReceipt>, now: Instant) -> u64 {
    receipt.map_or(u64::MAX, |receipt| {
        now.duration_since(receipt.received_at)
            .as_millis()
            .min(u128::from(u64::MAX)) as u64
    })
}

fn stale_receipt_reason(
    label: &str,
    receipt: Option<PlannerInputReceipt>,
    now: Instant,
    ttl: Duration,
) -> Option<String> {
    let Some(receipt) = receipt else {
        return Some(format!("{label} has no consumed observation"));
    };
    let age = now.duration_since(receipt.received_at);
    (!ttl.is_zero() && age > ttl).then(|| {
        format!(
            "{label} receipt age={}ms exceeds TTL={}ms (source_timestamp_ns={})",
            age.as_millis(),
            ttl.as_millis(),
            receipt.source_timestamp_ns,
        )
    })
}

fn validate_session(session: &NavSession) -> Result<(), RuntimeError> {
    session.options.validate().map_err(|reason| RuntimeError::InvalidInput {field:"goal_options",reason})?;
    session.goal.validate()
}

fn validate_map_pose(pose: &PoseStamped, field: &'static str) -> Result<(), RuntimeError> {
    if pose.frame_id != nuwax_nav_types_messages::NavFrameId::Map
        || ![
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ]
        .into_iter()
        .all(f64::is_finite)
        || !pose.pose.orientation.yaw().is_finite()
    {
        return Err(RuntimeError::InvalidInput {
            field,
            reason: "位姿必须在map坐标系具有有限位置和有效完整四元数".into(),
        });
    }
    Ok(())
}

fn path_rejected(reason: impl Into<String>) -> RuntimeError {
    RuntimeError::PathRejected {
        reason: reason.into(),
    }
}

#[cfg(test)]
mod tests {
    include!("scheduler_delivery_gap_tests.rs");
    include!("position_zero_runtime_tests.rs");
    include!("yaw_zero_contract_tests.rs");
    include!("recent_source_health_tests.rs");
    include!("registered_frame_stream_tests.rs");
    use std::sync::{Arc, Mutex};
    use std::sync::atomic::Ordering;

    use nuwax_nav_astar::navigation_map::CostMap2D;
    use nuwax_nav_types_messages::{NavFrameId, Point2, Pose3, Timestamp};

    use super::*;

    #[test]
    fn production_surface_worker_accepts_legacy_reverse_authorization_as_an_upper_bound() {
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.basic_kinematics = Some(nuwax_nav_types_messages::MotionKinematics {
            model:nuwax_nav_types_messages::MotionModel::Omnidirectional,
            max_forward_mps:0.35,max_reverse_mps:0.35,max_lateral_mps:0.35,
            max_angular_rps:0.8,minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        });
        assert!(config.motion_limits.is_none());
        let mut backend = production_surface_backend(config).unwrap();
        let goal = PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,0),pose:Pose3::default()};
        backend.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions {allow_reverse:true,..Default::default()}).unwrap();
    }

    fn physical_omni_limits() -> nuwax_nav_types_messages::MotionLimits {
        nuwax_nav_types_messages::MotionLimits {
            model:nuwax_nav_types_messages::MotionModel::Omnidirectional,
            max_forward_mps:1.0,max_reverse_mps:1.0,max_lateral_mps:1.0,max_angular_rps:0.8,
            linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,minimum_turning_radius_m:0.0,
            can_rotate_in_place:true,
        }
    }

    #[test]
    fn production_basic_old_oriented_paths_cannot_emit_sideways_or_reverse() {
        for (x,y) in [(2.0,3.0),(1.0,2.0)] {
            let mut backend=BasicInProcessLocalBackend::production(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() });
            backend.set_navigation_motion_limits(physical_omni_limits()).unwrap();
            backend.update_costmap(CostMap2D::new_all_free("map",80,80,0.1));
            let mut pose=nuwax_nav_astar::helpers::make_pose_stamped("map",2.0,2.0,0.0);
            let goal=nuwax_nav_astar::helpers::make_pose_stamped("map",x,y,0.0);
            backend.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions {
                allow_reverse:true,position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,
            }).unwrap();
            backend.replace_path_with_orientations(Path2D::stamped(NavFrameId::Map,Timestamp::default(),
                vec![Point2::new(2.0,2.0),Point2::new(x,y)]),Some(vec![0.0,0.0]));
            for tick in 0..20 {
                pose.stamp=Timestamp::from_unix(1+tick/10,(tick%10)as u32*100_000_000);
                backend.update_odometry(pose.clone());
                match backend.step() {
                    PlannerStep::Command(command)=>{
                        assert!(command.linear.x>=0.0,"生产二维后端不能按旧授权倒退：{command:?}");
                        assert_eq!(command.linear.y,0.0,"生产二维后端不能沿旧全向路径横移：{command:?}");
                    },
                    PlannerStep::ReplanRequired(_)=>break,
                    other=>panic!("旧全向路径应保持前向或请求新路径：{other:?}"),
                }
            }
        }
    }

    #[test]
    fn production_basic_forward_paths_turn_and_reach_side_and_rear_goals() {
        for heading in [std::f64::consts::FRAC_PI_2,std::f64::consts::PI] {
            let mut backend=BasicInProcessLocalBackend::production(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() });
            let physical=physical_omni_limits();
            backend.set_navigation_motion_limits(physical.clone()).unwrap();
            let grid=CostMap2D::new_all_free("map",80,80,0.1);
            backend.update_costmap(grid.clone());
            let mut pose=nuwax_nav_astar::helpers::make_pose_stamped("map",2.0,2.0,0.0);
            let goal=nuwax_nav_astar::helpers::make_pose_stamped("map",2.0+heading.cos(),2.0+heading.sin(),heading);
            backend.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions {
                allow_reverse:true,position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,
            }).unwrap();
            backend.replace_path_with_orientations(Path2D::stamped(NavFrameId::Map,Timestamp::default(),
                vec![Point2::new(2.0,2.0),Point2::new(2.0,2.0),goal.pose.position.to_2d()]),
                Some(vec![0.0,heading,heading]));
            let dt=1.0/LocalPlannerConfig::default_config().control_frequency;
            let mut applied=Twist2::ZERO;let mut reached=false;let mut done=false;
            let mut first_motion=None;
            for tick in 0..500 {
                pose.stamp=Timestamp::from_unix(1,0);pose.stamp.nanos+=(tick as f64*dt*1e9)as i64;
                backend.update_odometry(pose.clone());
                if tick%10==0 {backend.update_costmap(grid.clone());}
                let command=if reached {Twist2::ZERO} else {
                    match backend.step() {
                        PlannerStep::Command(command)=>command,
                        PlannerStep::Succeeded=>{reached=true;Twist2::ZERO},
                        other=>panic!("二维前向路径必须连续闭环：方向{heading}，拍{tick}，{other:?}"),
                    }
                };
                assert!(command.linear.x>=0.0,"生产二维命令不能倒退：{command:?}");
                assert_eq!(command.linear.y,0.0,"生产二维命令不能横移：{command:?}");
                if first_motion.is_none() && command.linear.x.hypot(command.linear.y)+command.angular_z.abs()>1e-9 {
                    first_motion=Some(command);assert_eq!(command.linear.x,0.0,"侧后方目标必须先原地转向");
                }
                let next=Twist2 {
                    linear:Point2::new(command.linear.x.clamp(applied.linear.x-physical.linear_acceleration_mps2*dt,
                        applied.linear.x+physical.linear_acceleration_mps2*dt),0.0),
                    angular_z:command.angular_z.clamp(applied.angular_z-physical.angular_acceleration_rps2*dt,
                        applied.angular_z+physical.angular_acceleration_rps2*dt),
                };
                let v=(applied.linear.x+next.linear.x)*0.5;let w=(applied.angular_z+next.angular_z)*0.5;
                let yaw=pose.pose.orientation.yaw();let end_yaw=yaw+w*dt;
                if w.abs()<1e-12 {pose.pose.position.x+=v*yaw.cos()*dt;pose.pose.position.y+=v*yaw.sin()*dt;}
                else {pose.pose.position.x+=v*(end_yaw.sin()-yaw.sin())/w;pose.pose.position.y+=v*(yaw.cos()-end_yaw.cos())/w;}
                pose.pose.orientation=nuwax_nav_types_messages::Quaternion::from_yaw(end_yaw);
                applied=next;backend.observe_applied_command(applied);
                assert!(applied.linear.x>=0.0);assert_eq!(applied.linear.y,0.0);
                if reached && applied.linear.x<1e-12 && applied.angular_z.abs()<1e-12 {
                    done=true;eprintln!("生产二维前向闭环：方向{heading:.3}，{}秒，零横移/倒退",(tick+1)as f64*dt);break;
                }
            }
            assert!(done && first_motion.is_some());
            assert!(pose.pose.position.distance(goal.pose.position)<0.12);
            assert_eq!(physical.max_lateral_mps,1.0,"物理手动运动声明不改写");
        }
    }

    #[derive(Debug, Default)]
    pub(crate) struct FakeMotionState {
        calls: Vec<String>,
        fail_twist: bool,
        rejected_generation: Option<u64>,
        // Enabled only by phase-sensitive protocol tests; ordinary fixtures do not allocate it.
        twist_probe: Option<Vec<(usize, String, Twist2)>>,
    }

    #[derive(Clone)]
    pub(crate) struct FakeMotion(pub(crate) Arc<Mutex<FakeMotionState>>);

    #[async_trait]
    impl NavMotionPort for FakeMotion {
        async fn acquire(&mut self, generation: u64) -> Result<u64, MotionPortError> {
            let mut state = self.0.lock().unwrap();
            state.calls.push("acquire:".into());
            if state.rejected_generation == Some(generation) {
                return Err(MotionPortError { message: "代次已过期".into(), code: Some(ErrorCode::StaleGeneration) });
            }
            Ok(11)
        }
        async fn velocity(&mut self, twist: Twist2) -> Result<(), MotionPortError> {
            let mut state = self.0.lock().unwrap();
            let index = state.calls.len();
            if let Some(probe) = &mut state.twist_probe { probe.push((index, String::new(), twist)); }
            state.calls.push("twist:".into());
            if state.fail_twist { return Err(MotionPortError::new("注入速度发送失败")); }
            Ok(())
        }
        async fn release(&mut self) -> Result<(), MotionPortError> {
            self.0.lock().unwrap().calls.push("release:".into());
            Ok(())
        }
}

    #[derive(Clone)]
    struct FakePlanner {
        state: Arc<Mutex<FakePlannerState>>,
    }

    #[derive(Debug)]
    struct FakePlannerState {
        step: PlannerStep,
        wait_reason: Option<String>,
        costmap_updates: usize,
        odometry_updates: usize,
        stops: usize,
    }

    impl FakePlanner {
        fn commanding() -> Self {
            Self {
                state: Arc::new(Mutex::new(FakePlannerState {
                    step: PlannerStep::Command(Twist2 {
                        linear: Point2::new(0.2, 0.0),
                        angular_z: 0.1,
                    }),
                    wait_reason: None,
                    costmap_updates: 0,
                    odometry_updates: 0,
                    stops: 0,
                })),
            }
        }
    }

    impl NavPlanner for FakePlanner {
        fn replace_path(&mut self, _path: Path2D) {}

        fn update_odometry(&mut self, _odometry: PoseStamped) {
            self.state
                .lock()
                .expect("fake planner state")
                .odometry_updates += 1;
        }

        fn update_costmap(&mut self, _costmap: CostMap2D) {
            self.state
                .lock()
                .expect("fake planner state")
                .costmap_updates += 1;
        }

        fn step(&mut self) -> PlannerStep {
            self.state.lock().expect("fake planner state").step.clone()
        }

        fn execution_wait_reason(&self) -> Option<String> {
            self.state.lock().expect("fake planner state").wait_reason.clone()
        }

        fn stop(&mut self) {
            self.state.lock().expect("fake planner state").stops += 1;
        }
    }

    fn test_scheduler(
        motion: FakeMotion,
        planner: FakePlanner,
    ) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>) {
        test_scheduler_with_ttls(
            motion,
            planner,
            Duration::from_secs(1),
            Duration::from_secs(1),
        )
    }

    fn test_scheduler_with_ttls(
        motion: FakeMotion,
        planner: FakePlanner,
        pose_ttl: Duration,
        costmap_ttl: Duration,
    ) -> (NavSchedulerHandle, JoinHandle<Result<(), RuntimeError>>) {
        let scheduler = NavScheduler::new(
            motion,
            planner,
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),

                path_wait_timeout: Duration::from_secs(1),
                    pose_ttl,
                costmap_ttl,
            },
        )
        .expect("test scheduler");
        spawn_scheduler(scheduler)
    }

    async fn takeover_scheduler_fixture() -> (
        NavScheduler<FakeMotion, FakePlanner>, watch::Sender<SchedulerStatus>, Arc<Mutex<FakeMotionState>>,
    ) {
        let motion = Arc::new(Mutex::new(FakeMotionState { twist_probe: Some(Vec::new()), ..Default::default() }));
        let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()), FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        let (status, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&status, map_pose(), None).unwrap();
        scheduler.observe_costmap(&status, costmap_at(1), Instant::now()).unwrap();
        scheduler.start_session(&status, session()).await.unwrap();
        scheduler.submit_path(&status, path()).await.unwrap();
        scheduler.control_tick(&status).await.unwrap();
        (scheduler, status, motion)
    }

    #[tokio::test]
    async fn replacement_lease_rejection_stops_execution_and_releases_previous_lease() {
        let (mut scheduler,status,motion)=takeover_scheduler_fixture().await;
        motion.lock().unwrap().rejected_generation=Some(2);
        let mut next=session_for("goal-2");next.motion_generation=2;
        assert!(matches!(scheduler.start_session(&status,next).await,Err(RuntimeError::LeaseNotGranted(_))));
        assert!(scheduler.session.is_none());
        assert_eq!(scheduler.status.last_terminal,Some(TerminalOutcome::Stopped));
        assert!(scheduler.current_execution_identity.is_none());
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|call|call.starts_with("release:")).count(),1);
    }

    #[tokio::test]
    async fn replacement_grant_continues_old_path_until_new_path_arrives_without_zero() {
        let (mut scheduler, status, motion) = takeover_scheduler_fixture().await;
        let old_identity = scheduler.current_execution_identity.clone().unwrap();
        let old_stops = scheduler.planner.state.lock().unwrap().stops;
        let mut next = session_for("goal-2"); next.motion_generation = 2;
        scheduler.start_session(&status, next).await.unwrap();
        assert_eq!(scheduler.status.session_id.as_deref(), Some("goal-2"));
        assert_eq!(scheduler.status.phase, SchedulerPhase::Controlling);
        assert!(scheduler.status.motion_sink_ready);
        assert!(Arc::ptr_eq(&old_identity, scheduler.current_execution_identity.as_ref().unwrap()));
        let began = scheduler.path_wait_started_at;
        scheduler.control_tick(&status).await.unwrap();
        assert_eq!(scheduler.path_wait_started_at, began);
        assert_eq!(scheduler.planner.state.lock().unwrap().stops, old_stops);
        let mut new_path = path_for("goal-2", 1);
        new_path.path = ExecutionPath::Planar(Path2D::stamped(NavFrameId::Map, Timestamp::default(),
            vec![Point2::new(1., 0.), Point2::new(3., 1.)]));
        let expected = new_path.path.clone();
        scheduler.submit_path(&status, new_path).await.unwrap();
        assert_eq!(scheduler.current_execution_identity.as_ref().unwrap().session_id, "goal-2");
        assert_eq!(scheduler.current_path.as_ref().unwrap().0, expected);
        scheduler.control_tick(&status).await.unwrap();
        let motion = motion.lock().unwrap();
        assert!(!motion.calls.iter().any(|call| call.starts_with("release:")));
        assert!(motion.twist_probe.as_ref().unwrap().iter().all(|(_, _, twist)| *twist != Twist2::ZERO));
    }

    #[tokio::test]
    async fn replacement_planning_failure_zeroes_once_and_reports_new_goal_failure() {
        let (mut scheduler, status, motion) = takeover_scheduler_fixture().await;
        scheduler.start_session(&status, session_for("goal-2")).await.unwrap();
        let (reply, received) = oneshot::channel();
        scheduler.handle_input(&status, InputCommand { kind: InputKind::PlanningDiagnostic(
            "goal-2".into(), execution_identity_for("goal-2", 0).goal_key(), "新目标无可行路径".into(), true, false), reply }).await;
        received.await.unwrap().unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));
        assert!(scheduler.status.message.contains("新目标无可行路径"));
        assert!(scheduler.session.is_none());
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("release:")).count(), 1);
    }

    #[tokio::test]
    async fn planning_reason_survives_pose_updates_and_rejects_old_goal_reports() {
        let motion=FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let (handle,task)=spawn_scheduler(NavScheduler::new(motion,FakePlanner::commanding(),SchedulerConfig::default()).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.start_session(session()).await.unwrap();
        handle.report_planning_diagnostic("goal-1".into(),execution_identity_for("goal-1",0).goal_key(),"等待路径：起点支撑不足".into(),false).await.unwrap();
        handle.update_odometry(map_pose_at(2)).await.unwrap();
        handle.report_planning_diagnostic("goal-1".into(),GoalKey {producer_runtime_generation:9,goal_generation:1},"旧目标的错误".into(),false).await.unwrap();
        assert_eq!(handle.status().message,"等待路径：起点支撑不足");
        handle.cancel("goal-1").await.unwrap();
        handle.report_planning_diagnostic("goal-1".into(),execution_identity_for("goal-1",0).goal_key(),"晚到报告".into(),false).await.unwrap();
        assert_eq!(handle.status().last_terminal,Some(TerminalOutcome::Cancelled));
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn source_pose_age_uses_slow_clock_and_still_rejects_stopped_clock_or_link() {
        const CHILD: &str = "NEH_TEST_POSE_DATA_TIME";
        let Some(path) = std::env::var_os(CHILD) else {
            let path = std::env::temp_dir().join(format!("neh-nav-data-clock-{}", std::process::id()));
            let status = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", "tests::source_pose_age_uses_slow_clock_and_still_rejects_stopped_clock_or_link"])
                .env(CHILD, &path).env("NUWAX_SOURCE_TIME_FILE", &path).status().unwrap();
            std::fs::remove_file(path).unwrap();
            assert!(status.success());
            return;
        };
        let source = 1_000_000_000_000_u64;
        let write_clock = |current: u64, live: bool| {
            let mono = if live { nuwax_motion_safety_guard::measured_motion::monotonic_ns() } else { 0 };
            let mut bytes = b"NEHTIME1".to_vec();
            for value in [1_u64, 1, current, 0, mono] { bytes.extend_from_slice(&value.to_le_bytes()); }
            std::fs::write(&path, bytes).unwrap();
        };
        write_clock(source + 200_000_000, true);
        let mut scheduler = NavScheduler::new(FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        let (tx, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_costmap(&tx, costmap_at(1), Instant::now()).unwrap();
        // 0.35倍时这份来源年龄200ms的数据，实际墙龄已超过原500ms来源期限。
        let acquired = Instant::now() - Duration::from_millis(572);
        let mut pose = map_pose_at(1);
        pose.stamp.nanos = source as i64;
        scheduler.observe_odometry_qualified_at(&tx, pose, None, acquired,
            Some(Duration::from_millis(500))).unwrap();
        assert_eq!(scheduler.pose_acquired_at, Some(acquired), "数据判龄不能刷新执行采集时刻");
        scheduler.refresh_input_status(Instant::now() + Duration::from_secs(1));
        assert!(scheduler.status.planner_inputs_fresh, "墙钟推进不等于来源数据变旧");
        write_clock(source + 600_000_000, true);
        assert!(scheduler.stale_input_reason(Instant::now()).unwrap().contains("pose source deadline elapsed"));
        write_clock(source + 200_000_000, true);
        assert!(scheduler.stale_input_reason(Instant::now() + Duration::from_secs(6))
            .unwrap().contains("liveness"), "来源钟有更新，断流输入仍须拒绝");
        write_clock(source + 200_000_000, false);
        assert!(scheduler.stale_input_reason(Instant::now()).unwrap().contains("停止推进"));
    }

    #[tokio::test]
    async fn slow_geometry_keeps_renewing_and_cancellation_does_not_wait_for_compute() {
        struct Slow {entered:Arc<std::sync::atomic::AtomicBool>}
        impl NavPlanner for Slow {
            fn replace_path(&mut self,_:Path2D) {}
            fn update_odometry(&mut self,_:PoseStamped) {}
            fn update_costmap(&mut self,_:CostMap2D) {}
            fn step(&mut self)->PlannerStep {
                self.entered.store(true,Ordering::SeqCst);
                std::thread::sleep(Duration::from_millis(700));
                PlannerStep::Command(Twist2 {linear:Point2::new(0.2,0.0),angular_z:0.0})
            }
            fn stop(&mut self) {}
        }
        let entered=Arc::new(std::sync::atomic::AtomicBool::new(false));
        let motion=FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let calls=motion.0.clone();
        let worker=planner_worker::PlannerWorker::new(Slow {entered:entered.clone()}).unwrap();
        let (handle,task)=spawn_scheduler(NavScheduler::new(motion,worker,SchedulerConfig::default()).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();
        handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.start_session(session()).await.unwrap();handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1),async {
            while !entered.load(Ordering::SeqCst) {tokio::time::sleep(Duration::from_millis(5)).await;}
        }).await.unwrap();
        tokio::time::sleep(Duration::from_millis(520)).await;
        assert!(calls.lock().unwrap().calls.iter().filter(|call|call.starts_with("twist:")).count()>=2);
        assert!(handle.status().message.contains("局部轨迹正在计算"),
            "零速执行回执不能掩盖仍在等待局部计算的阶段");
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        tokio::time::timeout(Duration::from_millis(100),handle.cancel("goal-1")).await
            .expect("取消不能等待700毫秒局部计算").unwrap();
        assert_eq!(handle.status().last_terminal,Some(TerminalOutcome::Cancelled));
        assert_ne!(handle.status().last_applied_twist.map(|t|t.linear.x),Some(0.2));
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn slow_local_blockage_reaches_scheduler_replanning_state() {
        struct Blocked;
        impl NavPlanner for Blocked {
            fn replace_path(&mut self, _:Path2D) {}
            fn update_odometry(&mut self, _:PoseStamped) {}
            fn update_costmap(&mut self, _:CostMap2D) {}
            fn step(&mut self) -> PlannerStep {
                std::thread::sleep(DEFAULT_CONTROL_HORIZON + Duration::from_millis(50));
                PlannerStep::ReplanRequired("完整机体扫掠受阻".into())
            }
            fn stop(&mut self) {}
        }
        let motion = FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let worker = planner_worker::PlannerWorker::new(Blocked).unwrap();
        let (handle, task) = spawn_scheduler(NavScheduler::new(motion, worker, SchedulerConfig::default()).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();
        handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            while handle.status().phase != SchedulerPhase::AwaitingPath
                || !handle.status().message.contains("完整机体扫掠受阻")
            {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.expect("慢失败必须沿生产调度器触发停车重规划，不能无限发零速");
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        handle.cancel("goal-1").await.unwrap();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    fn execution_identity_for(session_id: &str, path_revision: u64) -> PathExecutionIdentity {
        PathExecutionIdentity {
        goal_key: nuwax_nav_types_messages::GoalKey { producer_runtime_generation: (1) as u64, goal_generation: (1) as u64 },
            session_id: session_id.into(),

            planner_runtime_generation: i64::from(path_revision != 0),
            path_revision,

        }
    }

    pub(crate) fn session_for(session_id: &str) -> NavSession {
        NavSession {
            boundary: Vec::new(),
            options: Default::default(),
            session_id: session_id.into(),
            execution_identity: execution_identity_for(session_id, 0),

            motion_generation: 1,
            goal: ExecutionGoal::Body(nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, 0.0)),
        }
    }

    fn session() -> NavSession {
        session_for("goal-1")
    }

    pub(crate) fn path_for(session_id: &str, path_revision: u64) -> VersionedPath {
        VersionedPath {
            route_kind: RouteKind::Complete,
            session_id: session_id.into(),
            execution_identity: execution_identity_for(session_id, path_revision),
            path_revision,

            path_yaws: None,
            path: ExecutionPath::Planar(Path2D::stamped(
                NavFrameId::Map,
                Timestamp::default(),
                vec![Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)],
            )),
        }
    }

    fn path() -> VersionedPath {
        path_for("goal-1", 1)
    }

    fn map_pose_at(seconds: i64) -> PoseStamped {
        let mut pose = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
        // 测试输入表示真实有效姿态，不能用全零四元数冒充单位旋转。
        pose.pose.orientation = nuwax_nav_types_messages::Quaternion::IDENTITY;
        pose.stamp = Timestamp::from_unix(seconds, 0);
        pose
    }

    fn map_pose() -> PoseStamped {
        map_pose_at(1)
    }

    fn costmap_at(source_timestamp_ns: i64) -> CostmapObservation {
        CostmapObservation {
            costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 40, 40, 0.1)),
            source_timestamp_ns,
        }
    }

    /// 完整机体需要中心线两侧都有地图观测。
    fn body_costmap_at(source_timestamp_ns: i64) -> CostmapObservation {
        let mut map = CostMap2D::new_all_free("map", 40, 40, 0.1);
        map.set_origin(Point2::new(-0.5, -0.5));
        CostmapObservation { costmap: ExecutionMap::Planar(map), source_timestamp_ns }
    }

    async fn wait_for_sink(handle: &NavSchedulerHandle) -> SchedulerStatus {
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                if snapshot.motion_sink_ready {
                    break snapshot;
                }
                status.changed().await.expect("scheduler status open");
            }
        })
        .await
        .expect("motion sink readiness timeout")
    }

    async fn wait_for_terminal(
        handle: &NavSchedulerHandle,
        expected: TerminalOutcome,
    ) -> SchedulerStatus {
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                if snapshot.last_terminal == Some(expected) {
                    break snapshot;
                }
                status.changed().await.expect("scheduler status open");
            }
        })
        .await
        .expect("terminal status timeout")
    }

    async fn wait_for_input_pause(handle: &NavSchedulerHandle) -> SchedulerStatus {
        let mut status = handle.subscribe_status();
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                assert!(snapshot.last_terminal.is_none(), "输入暂缺不能直接结束目标: {snapshot:?}");
                if snapshot.last_applied_twist == Some(Twist2::ZERO)
                    && snapshot.message.starts_with("等待新鲜导航输入:") {
                    return snapshot;
                }
                status.changed().await.unwrap();
            }
        }).await.expect("过期输入必须及时执行零速等待")
    }

    #[tokio::test]
    async fn one_second_pose_interruption_resumes_the_same_path_and_arrives() {
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planning = planner.state.clone();
        let (handle, task) = spawn_scheduler(NavScheduler::new(
            FakeMotion(motion.clone()), planner, SchedulerConfig::default(),
        ).unwrap());
        handle.update_costmap(costmap_at(1)).await.unwrap();
        handle.update_odometry_with_deadline(map_pose_at(1), Instant::now() + Duration::from_millis(500)).await.unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while !handle.status().last_applied_twist.is_some_and(|twist| twist != Twist2::ZERO) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap();
        tokio::time::sleep(Duration::from_secs(1)).await;
        let paused = wait_for_input_pause(&handle).await;
        assert_eq!(paused.path_revision, 1);
        assert_eq!(paused.session_id.as_deref(), Some("goal-1"));

        handle.update_odometry_with_deadline(map_pose_at(2), Instant::now() + Duration::from_millis(500)).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while !handle.status().last_applied_twist.is_some_and(|twist| twist != Twist2::ZERO) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.expect("恢复位姿后同一路径继续执行");
        assert_eq!(handle.status().path_revision, 1);
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("acquire:")).count(), 1);
        planning.lock().unwrap().step = PlannerStep::Succeeded;
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn continuous_input_pause_uses_original_thirty_second_budget() {
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()), FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        assert_eq!(scheduler.config.path_wait_timeout, Duration::from_secs(30));
        let (status, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&status, map_pose(), None).unwrap();
        scheduler.observe_costmap(&status, costmap_at(1), Instant::now()).unwrap();
        scheduler.start_session(&status, session()).await.unwrap();
        scheduler.submit_path(&status, path()).await.unwrap();
        scheduler.control_tick(&status).await.unwrap();
        assert!(scheduler.path_wait_started_at.is_none());
        scheduler.pose_valid_until = Some(Instant::now() - Duration::from_millis(1));
        scheduler.control_tick(&status).await.unwrap();
        let first_wait = scheduler.path_wait_started_at;
        for _ in 0..3 { scheduler.control_tick(&status).await.unwrap(); }
        assert_eq!(scheduler.path_wait_started_at, first_wait);
        assert!(scheduler.lease_active && scheduler.session.is_some());
        assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
        assert!(scheduler.status.last_terminal.is_none());
        // 只设置已经流逝的等待起点，原30秒配置及输入期限保持原值。
        scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(30));
        scheduler.control_tick(&status).await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));
        assert!(scheduler.status.message.contains("pose source deadline"));

        assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("release:")).count(), 1);
    }







    #[tokio::test]
    async fn controller_consumes_map_inputs_and_only_reports_applied_motion() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        let (handle, task) = test_scheduler(FakeMotion(motion_state.clone()), planner);
        wait_for_sink(&handle).await;

        handle.start_session(session()).await.expect("session");
        handle
            .update_costmap(CostmapObservation {
                costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 4, 4, 0.1)),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("costmap");
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle.submit_path(path()).await.expect("versioned path");

        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle.status().applied_commands > 0 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("controller APPLIED timeout");
        let status = handle.status();
        assert!(status.last_applied_twist.is_some());
        assert_eq!(planner_state.lock().expect("planner").costmap_updates, 1);
        assert_eq!(planner_state.lock().expect("planner").odometry_updates, 1);

        handle.cancel("goal-1").await.expect("cancel waits ZEROED");
        let terminal = handle.status();
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::Cancelled));

        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(terminal.session_id, None);
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(planner_state.lock().expect("planner").stops, 3);
        let calls = motion_state.lock().expect("motion").calls.clone();
        assert!(calls.iter().any(|call| call.starts_with("twist:")));
        assert!(calls.iter().any(|call| call.starts_with("release:")));

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    /// 子进程给生产入口提供必需外形，避免并行测试共享进程环境。
    fn run_with_robot_geometry(test: &str) -> bool {
        if std::env::var("NUWAX_TEST_BODY_FIXTURE").as_deref() == Ok(test) { return true; }
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", test, "--nocapture"])
            .env("NUWAX_TEST_BODY_FIXTURE", test)
            .env("NUWAX_ROBOT_GEOMETRY_JSON", r#"{"footprint_xy_m":[[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],"z_min_m":-0.4,"z_max_m":0.2}"#)
            .output().unwrap();
        assert!(output.status.success(), "外形夹具执行失败：{}{}", String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr));
        false
    }

    #[tokio::test]
    async fn basic_in_process_backend_is_scheduler_owned_and_intent_gated() {
        if !run_with_robot_geometry("tests::basic_in_process_backend_is_scheduler_owned_and_intent_gated") { return; }
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let backend = production_local_backend(LocalExecutionBackendKind::BasicInProcess).unwrap();
        assert_eq!(backend.kind(), LocalExecutionBackendKind::BasicInProcess);
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::clone(&motion_state)),
            backend,
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),

                path_wait_timeout: Duration::from_secs(1),
                    pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
            },
        )
        .expect("basic in-process scheduler");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;

        handle.update_odometry(map_pose()).await.expect("map pose");
        handle
            .update_costmap(body_costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        tokio::time::sleep(Duration::from_millis(15)).await;
        assert!(
            motion_state
                .lock()
                .expect("motion")
                .calls
                .iter()
                .all(|call| !call.starts_with("twist:")),
            "the backend cannot bypass scheduler session and path admission"
        );

        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("qualified path");
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if handle.status().last_applied_twist.is_some_and(|twist|twist!=Twist2::ZERO) {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("actual basic planner APPLIED timeout");
        let applied = handle
            .status()
            .last_applied_twist
            .expect("actual basic planner twist");
        assert_ne!(applied, Twist2::ZERO);

        handle.cancel("goal-1").await.expect("verified cancel");
        let terminal = handle.status();
        assert_eq!(terminal.phase, SchedulerPhase::Idle);
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::Cancelled));
        assert_eq!(terminal.session_id, None);
        assert_eq!(terminal.path_revision, 0);
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        let twist_count_after_cancel = motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .filter(|call| call.starts_with("twist:"))
            .count();
        tokio::time::sleep(Duration::from_millis(20)).await;
        assert_eq!(
            motion_state
                .lock()
                .expect("motion")
                .calls
                .iter()
                .filter(|call| call.starts_with("twist:"))
                .count(),
            twist_count_after_cancel,
            "cancelled backend must remain stopped beyond two controller ticks"
        );
        let late = handle
            .submit_path(path())
            .await
            .expect_err("terminal scheduler must reject late path replay");
        assert!(matches!(late, RuntimeError::PathRejected { .. }));

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
        assert!(matches!(
            handle.submit_path(path()).await,
            Err(RuntimeError::SchedulerStopped)
        ));
    }

    #[tokio::test]
    async fn production_backend_waits_for_goal_yaw_and_discards_orientation_mutation() {
        if !run_with_robot_geometry("tests::production_backend_waits_for_goal_yaw_and_discards_orientation_mutation") { return; }
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::clone(&motion_state)),
            production_local_backend(LocalExecutionBackendKind::BasicInProcess).unwrap(),
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
                ..SchedulerConfig::default()
            },
        )
        .expect("生产局部后端");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;
        let goal_yaw = std::f64::consts::FRAC_PI_2;
        let mut endpoint = nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, 0.0);
        endpoint.stamp = Timestamp::from_unix(1, 0);
        handle.update_odometry(endpoint).await.expect("终点位姿");
        handle
            .update_costmap(body_costmap_at(1_000_000_000))
            .await
            .expect("覆盖路径的地图");
        let mut requested=session();
        requested.goal.as_body_mut().unwrap().pose.orientation=nuwax_nav_types_messages::Quaternion::from_yaw(goal_yaw);
        handle.start_session(requested).await.expect("会话");

        let mut oriented_path = path();
        oriented_path.path_yaws = Some(vec![0.0, goal_yaw]);
        handle.submit_path(oriented_path).await.expect("带朝向路径");
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut status = handle.subscribe_status();
            while status.borrow().applied_commands == 0 {
                status.changed().await.expect("控制状态");
            }
        })
        .await
        .expect("实际旋转指令");
        let rotating = handle.status();
        assert_eq!(rotating.last_terminal, None);
        let twist = rotating.last_applied_twist.expect("旋转已执行");
        assert_eq!(twist.linear.x, 0.0);
        assert!(twist.angular_z > 0.0);

        let mut aligned = nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, goal_yaw);
        aligned.stamp = Timestamp::from_unix(2, 0);
        handle.update_odometry(aligned.clone()).await.expect("朝向已到");
        let terminal = wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));


        handle
            .start_session(session_for("goal-2"))
            .await
            .expect("第二次会话");
        let mut second_path = path_for("goal-2", 1);
        second_path.path_yaws = Some(vec![0.0, 0.0]);
        handle
            .submit_path(second_path.clone())
            .await
            .expect("第二次旋转路径");
        second_path.path_yaws = Some(vec![0.0, goal_yaw]);
        assert!(matches!(
            handle.submit_path(second_path).await,
            Err(RuntimeError::PathMutation { .. })
        ));
        assert_eq!(handle.status().last_terminal, None);
        assert_eq!(handle.status().phase, SchedulerPhase::Controlling);

        handle.shutdown().await.expect("关闭");
        task.await.expect("等待调度器").expect("调度器已退出");
    }

    #[tokio::test]
    async fn same_revision_replay_is_idempotent_and_mutation_preserves_original_execution() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = Arc::clone(&planner.state);
        let (handle, task) = test_scheduler(FakeMotion(Arc::clone(&motion_state)), planner);
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");

        let accepted = path();
        handle
            .submit_path(accepted.clone())
            .await
            .expect("initial path");
        handle
            .submit_path(accepted.clone())
            .await
            .expect("exact same-revision replay is idempotent");
        assert_eq!(handle.status().message, "versioned path accepted");
        let mut resent = accepted.clone();
        if let ExecutionPath::Planar(path) = &mut resent.path {
            path.stamp = Timestamp::from_unix(9, 0);
            path.points[0].y = -0.0;
        }
        handle.submit_path(resent).await.expect("same route with a new send time is idempotent");

        let mut wrong_identity = accepted.clone();
        wrong_identity.session_id = "wrong-session".into();
        wrong_identity.execution_identity.session_id = "wrong-session".into();
        let rejected = handle
            .submit_path(wrong_identity)
            .await
            .expect_err("wrong identity must be rejected without fencing");
        assert!(matches!(rejected, RuntimeError::PathRejected { .. }));
        assert_eq!(handle.status().phase, SchedulerPhase::Controlling);

        let replacement = path_for("goal-1", 2);
        handle
            .submit_path(replacement.clone())
            .await
            .expect("higher revision replacement is accepted");
        assert_eq!(handle.status().path_revision, 2);

        let stops_before_mutation = planner_state.lock().unwrap().stops;
        let mut mutation = replacement;
        mutation.path = ExecutionPath::Planar(Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(3.0, 0.0), Point2::new(4.0, 0.0)],
        ));
        let error = handle
            .submit_path(mutation)
            .await
            .expect_err("same-revision mutation must fail closed");
        assert!(matches!(error, RuntimeError::PathMutation { .. }));

        let retained = handle.status();
        assert_eq!(retained.phase, SchedulerPhase::Controlling);
        assert_eq!(retained.last_terminal, None);
        assert_eq!(retained.session_id.as_deref(), Some("goal-1"));
        assert_eq!(retained.path_revision, 2);
        assert!(retained.message.contains("同修订路径变化已丢弃"));
        assert!(!motion_state.lock().unwrap().calls.iter().any(|call| call.starts_with("release:")));
        assert_eq!(planner_state.lock().unwrap().stops, stops_before_mutation);
        handle.submit_path(path_for("goal-1", 2)).await.expect("原路径仍能重放");

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn post_terminal_receipts_preserve_qualified_cause() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let (handle, task) = test_scheduler(FakeMotion(motion_state), planner);
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("initial pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("initial costmap");
        handle.start_session(session()).await.expect("session");

        let accepted = path_for("goal-1", 1);
        handle
            .submit_path(accepted.clone())
            .await
            .expect("initial path");
        handle.invalidate_map("测试实际地图失效终态").await.expect("地图失效须停车");

        let terminal = handle.status();
        assert_eq!(terminal.phase, SchedulerPhase::Failed);
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert!(terminal
            .message
            .starts_with("SafetyFailed cause=MapSourceInvalidated:"));
        let qualified_cause = terminal.message;

        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("post-terminal pose receipt");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("post-terminal costmap receipt");
        let after_receipts = handle.status();
        assert_eq!(after_receipts.phase, SchedulerPhase::Failed);
        assert_eq!(
            after_receipts.last_terminal,
            Some(TerminalOutcome::SafetyFailed)
        );
        assert_eq!(after_receipts.message, qualified_cause);
        assert_eq!(after_receipts.pose_source_timestamp_ns, 2_000_000_000);
        assert_eq!(after_receipts.costmap_source_timestamp_ns, 2_000_000_000);
        assert!(after_receipts.planner_inputs_fresh);

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn natural_terminal_allows_continuous_goal_a_then_b() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        planner.state.lock().expect("planner").step = PlannerStep::Succeeded;
        let (handle, task) = test_scheduler(FakeMotion(motion_state.clone()), planner);
        wait_for_sink(&handle).await;

        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("pose A");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap A");
        handle
            .start_session(session_for("goal-a"))
            .await
            .expect("goal A");
        handle
            .submit_path(path_for("goal-a", 1))
            .await
            .expect("path A");
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        assert!(!task.is_finished(), "natural terminal must not stop owner");

        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("pose B");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("costmap B");
        handle
            .start_session(session_for("goal-b"))
            .await
            .expect("goal B must not see Busy");
        handle
            .submit_path(path_for("goal-b", 1))
            .await
            .expect("path B");
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;

        let fence_count = motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .filter(|call| call.starts_with("release:"))
            .count();
        assert_eq!(fence_count, 2);
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn planner_failure_can_retry_without_restarting_scheduler_owner() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        planner_state.lock().expect("planner").step = PlannerStep::Failed("blocked".into());
        let (handle, task) = test_scheduler(FakeMotion(motion_state), planner);
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("pose 1");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap 1");
        handle
            .start_session(session_for("retry-1"))
            .await
            .expect("first session");
        handle
            .submit_path(path_for("retry-1", 1))
            .await
            .expect("first path");
        let terminal = wait_for_terminal(&handle, TerminalOutcome::PlannerFailed).await;
        assert_eq!(
            terminal.message,
            "PlannerFailed cause=LocalPlanner: blocked"
        );

        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(!task.is_finished(), "planner failure must not stop owner");

        planner_state.lock().expect("planner").step = PlannerStep::Succeeded;
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("pose 2");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("costmap 2");
        handle
            .start_session(session_for("retry-2"))
            .await
            .expect("retry session");
        handle
            .submit_path(path_for("retry-2", 1))
            .await
            .expect("retry path");
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn replan_confirms_zero_requires_new_revision_and_remains_cancellable() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = Arc::clone(&planner.state);
        planner_state.lock().unwrap().step = PlannerStep::ReplanRequired("连接阻断".into());
        let (handle, task) = test_scheduler(FakeMotion(Arc::clone(&motion_state)), planner);
        handle.update_odometry(map_pose()).await.unwrap();
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        let held = handle.status();
        assert_eq!(held.last_applied_twist, Some(Twist2::ZERO));
        assert!(held.applied_commands > 0);
        assert_eq!(held.last_terminal, None);
        let feedback=held.surface_replan_feedback.as_ref().expect("typed rejection after ZERO");
        assert!(feedback.zero_confirmed_rejection);
        assert_eq!(feedback.rejection_reason.as_deref(), Some("连接阻断"));
        assert_eq!(feedback.path_identity.as_ref(),&path().execution_identity);
        // Unsupported backends preserve the refusal but provide no reuse proof.
        assert!(feedback.rejected_context.is_none() && feedback.current_context.is_none());
        assert!(matches!(
            handle.submit_path(path()).await,
            Err(RuntimeError::PathRejected { .. })
        ));

        planner_state.lock().unwrap().step = PlannerStep::Command(Twist2 {
            linear: Point2::new(0.2, 0.0),
            angular_z: 0.0,
        });
        handle.submit_path(path_for("goal-1", 2)).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().last_applied_twist == Some(Twist2::ZERO) {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        })
        .await
        .expect("更高路径修订恢复执行");
        assert!(handle.status().surface_replan_feedback.as_ref()
            .is_none_or(|feedback|!feedback.zero_confirmed_rejection));
        planner_state.lock().unwrap().step = PlannerStep::ReplanRequired("再次阻断".into());
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        assert_eq!(handle.status().surface_replan_feedback.as_ref().unwrap().path_identity.path_revision,2);
        handle.cancel("goal-1").await.unwrap();
        assert!(handle.status().surface_replan_feedback.is_none());
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::Cancelled)
        );

        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn failed_zero_receipt_never_publishes_execution_rejection_feedback() {
        let motion=FakeMotion(Arc::new(Mutex::new(FakeMotionState {fail_twist:true,..Default::default()})));
        let planner=FakePlanner::commanding();
        planner.state.lock().unwrap().step=PlannerStep::ReplanRequired("refused".into());
        let mut scheduler=NavScheduler::new(motion,planner,SchedulerConfig::default()).unwrap();
        let (status,_)=watch::channel(scheduler.status.clone());
        scheduler.planner.update_odometry(map_pose());
        scheduler.planner.update_execution_map(costmap_at(1).costmap).unwrap();
        scheduler.pose_receipt=Some(PlannerInputReceipt {source_timestamp_ns:1,received_at:Instant::now()});
        scheduler.costmap_receipt=Some(PlannerInputReceipt {source_timestamp_ns:1,received_at:Instant::now()});
        scheduler.start_session(&status,session()).await.unwrap();
        scheduler.submit_path(&status,path()).await.unwrap();
        assert!(scheduler.control_tick(&status).await.is_err());
        assert_eq!(scheduler.status.phase,SchedulerPhase::Failed);
        assert!(scheduler.status.surface_replan_feedback.is_none());
    }

    async fn wait_for_phase(handle: &NavSchedulerHandle, expected: SchedulerPhase) {
        tokio::time::timeout(Duration::from_secs(1), async {
            let mut status = handle.subscribe_status();
            while status.borrow().phase != expected {
                status.changed().await.unwrap();
            }
        })
        .await
        .unwrap_or_else(|_|panic!("调度阶段等待超时: {:?}",handle.status()));
    }

    #[tokio::test]
    async fn worker_pending_keeps_original_wait_deadline_goal_and_route_then_times_out() {
        struct BlockedPending {
            release: Option<std::sync::mpsc::Receiver<()>>,
        }
        impl NavPlanner for BlockedPending {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn step(&mut self) -> PlannerStep {
                if let Some(release) = self.release.take() { release.recv().unwrap(); }
                PlannerStep::Pending(vec!["BudgetExhausted: finite search budget".into()])
            }
            fn execution_wait_reason(&self) -> Option<String> { Some("typed search pending".into()) }
            fn stop(&mut self) {}
        }
        let (release, receive) = std::sync::mpsc::channel();
        let worker = crate::planner_worker::PlannerWorker::new(BlockedPending { release: Some(receive) }).unwrap();
        let mut scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))), worker,
            SchedulerConfig { path_wait_timeout: Duration::from_secs(2), ..SchedulerConfig::default() }).unwrap();
        let (sender, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
        scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
        scheduler.start_session(&sender, session()).await.unwrap();
        let original_deadline = scheduler.path_wait_started_at;
        scheduler.submit_path(&sender, path()).await.unwrap();
        let identity = scheduler.current_execution_identity.clone();
        let route = scheduler.current_path.clone();
        // Worker is still computing: this must not be Command(ZERO), which clears the timer.
        scheduler.control_tick(&sender).await.unwrap();
        // Release before assertions so a failed assertion cannot strand the worker on drop.
        release.send(()).unwrap();
        assert_eq!(scheduler.path_wait_started_at, original_deadline);
        assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            scheduler.control_tick(&sender).await.unwrap();
            assert_eq!(scheduler.path_wait_started_at, original_deadline);
            assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
            assert_eq!(scheduler.current_execution_identity, identity);
            assert_eq!(scheduler.current_path, route);
            assert!(scheduler.session.is_some());
            assert_eq!(scheduler.status.phase, SchedulerPhase::Controlling);
            if scheduler.status.message.contains("typed search pending") {
                assert!(scheduler.status.message.contains("BudgetExhausted"));
                assert!(scheduler.status.message.contains("finite search budget"));
                break;
            }
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(3));
        scheduler.control_tick(&sender).await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));

    }

    #[tokio::test]
    async fn scheduler_message_pending_retains_reason_and_failures_without_growth() {
        for wait_reason in [None, Some(""), Some("  "), Some("typed search pending")] {
            for has_failures in [false, true] {
                let planner = FakePlanner::commanding();
                let failures = if has_failures {
                    vec!["BudgetExhausted: finite search budget; observation expired; required clearance".into()]
                } else { vec![] };
                {
                    let mut state = planner.state.lock().unwrap();
                    state.step = PlannerStep::Pending(failures);
                    state.wait_reason = wait_reason.map(str::to_owned);
                }
                let mut scheduler = NavScheduler::new(
                    FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
                    planner, SchedulerConfig::default()).unwrap();
                let (sender, _) = watch::channel(scheduler.status.clone());
                scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
                scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
                scheduler.start_session(&sender, session()).await.unwrap();
                scheduler.submit_path(&sender, path()).await.unwrap();
                let original_deadline = scheduler.path_wait_started_at;
                let identity = scheduler.current_execution_identity.clone();
                scheduler.control_tick(&sender).await.unwrap();
                let message = scheduler.status.message.clone();
                assert!(message.starts_with(wait_reason.filter(|r| !r.trim().is_empty())
                    .unwrap_or("局部轨迹待定，保持零速等待")));
                if has_failures {
                    assert!(message.contains("BudgetExhausted"));
                    assert!(message.contains("finite search budget; observation expired; required clearance"));
                } else {
                    assert!(!message.contains("[]"));
                    assert!(!message.ends_with(':'));
                }
                for _ in 0..3 {
                    scheduler.control_tick(&sender).await.unwrap();
                    assert_eq!(scheduler.status.message, message, "repeated ticks must not accumulate text");
                    assert_eq!(scheduler.path_wait_started_at, original_deadline);
                    assert_eq!(scheduler.current_execution_identity, identity);
                    assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
                    assert_eq!(scheduler.status.last_terminal, None);
                }
            }
        }
    }

    #[tokio::test]
    async fn default_wait_budget_terminates_all_stationary_waits_and_allows_recovery() {
        for reason in [None, Some(""), Some("当前几何尚无可执行运动")] {
        for pending in [false, true] {
            for recover in [false, true] {
                let planner = FakePlanner::commanding();
                {
                    let mut state = planner.state.lock().unwrap();
                    state.step = if pending { PlannerStep::Pending(vec![]) }
                        else { PlannerStep::Command(Twist2::ZERO) };
                    state.wait_reason = reason.map(str::to_owned);
                }
                let mut scheduler = NavScheduler::new(
                    FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
                    planner, SchedulerConfig::default()).unwrap();
                let (sender, _) = watch::channel(scheduler.status.clone());
                scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
                scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
                scheduler.start_session(&sender, session()).await.unwrap();
                scheduler.submit_path(&sender, path()).await.unwrap();
                let started = scheduler.path_wait_started_at;
                scheduler.control_tick(&sender).await.unwrap();
                assert_eq!(scheduler.path_wait_started_at, started, "持续零速不得清除等待期限");
                if recover {
                    let mut state = scheduler.planner.state.lock().unwrap();
                    state.step = PlannerStep::Command(Twist2 { linear: Point2::new(0.2, 0.0), angular_z: 0.0 });
                    state.wait_reason = None;
                }
                scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(31));
                scheduler.control_tick(&sender).await.unwrap();
                assert!(scheduler.path_wait_started_at.is_none());
                if recover {
                    assert_eq!(scheduler.status.last_terminal, None);
                    assert_ne!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
                } else {
                    assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));

                    if let Some(reason) = reason.filter(|s|!s.is_empty()) {
                        assert!(scheduler.status.message.contains(reason));
                    }
                }
            }
        }
        }
        let mut scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        let (sender, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&sender, map_pose(), None).unwrap();
        scheduler.observe_costmap(&sender, costmap_at(1_000_000_000), Instant::now()).unwrap();
        scheduler.start_session(&sender, session()).await.unwrap();
        scheduler.path_wait_started_at = Some(Instant::now() - Duration::from_secs(31));
        scheduler.check_path_wait_timeout(&sender).await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::PlannerFailed));

    }

    #[tokio::test]
    async fn scheduler_message_timeout_first_terminal_retains_cause_and_fence() {
        for cause in ["BudgetExhausted: finite search budget", "NearFieldObservation: cell=[1,2,3] expired",
            "surface does not satisfy the required clearance", "", "  "] {
            let motion = Arc::new(Mutex::new(FakeMotionState::default()));
            let mut scheduler = NavScheduler::new(FakeMotion(motion.clone()),
                FakePlanner::commanding(), SchedulerConfig {
                path_wait_timeout: Duration::from_secs(1), ..SchedulerConfig::default()
            }).unwrap();
            scheduler.session = Some(session());
            scheduler.lease_active = true;
            scheduler.path_wait_started_at = Some(Instant::now() - scheduler.config.path_wait_timeout);
            scheduler.status.message = cause.into();
            let (sender, observed) = watch::channel(scheduler.status.clone());
            scheduler.check_path_wait_timeout(&sender).await.unwrap();
            let status = observed.borrow();
            assert_eq!(status.last_terminal, Some(TerminalOutcome::PlannerFailed));
            assert_eq!(status.phase, SchedulerPhase::Failed);
            assert!(status.message.starts_with("planner path timeout after "));
            assert!(status.message.contains("ms; fencing"));
            if !cause.trim().is_empty() { assert!(status.message.contains(cause)); }
            else { assert!(!status.message.contains("; ;")); }
            assert!(status.message.ends_with("PlannerFailed，租约已释放"));

            assert_eq!(status.last_applied_twist, Some(Twist2::ZERO));
            assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("release:")).count(), 1);
            assert!(scheduler.path_wait_started_at.is_none());
        }
    }

    #[tokio::test]
    async fn unusable_replan_revisions_cannot_extend_the_original_wait_budget() {
        let planner = FakePlanner::commanding();
        planner.state.lock().unwrap().step = PlannerStep::ReplanRequired("无有效连接".into());
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            planner,
            SchedulerConfig {
                tick_interval: Duration::from_millis(2),

                path_wait_timeout: Duration::from_millis(60),
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
                ..SchedulerConfig::default()
            },
        )
        .unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        handle.update_odometry(map_pose()).await.unwrap();
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        tokio::time::timeout(Duration::from_millis(200), async {
            let mut revision = 1;
            while handle.status().last_terminal.is_none() {
                if handle.status().phase == SchedulerPhase::AwaitingPath {
                    revision += 1;
                    let _ = handle.submit_path(path_for("goal-1", revision)).await;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("无效修订不能不断续期等待时间");
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::PlannerFailed)
        );

        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn source_pose_deadline_pauses_while_receipt_is_still_fresh() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler(
            FakeMotion(Arc::clone(&motion_state)),
            FakePlanner::commanding(),
        );
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .unwrap();
        handle
            .update_odometry_with_deadline(map_pose(), Instant::now() + Duration::from_millis(120))
            .await
            .unwrap();
        assert!(matches!(
            handle
                .update_odometry_with_deadline(
                    map_pose_at(2),
                    Instant::now() - Duration::from_millis(1)
                )
                .await,
            Err(RuntimeError::StaleInput(_))
        ));
        assert_eq!(
            handle.status().pose_source_timestamp_ns,
            1_000_000_000,
            "过期输入不能覆盖已接纳位姿"
        );
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        let terminal = wait_for_input_pause(&handle).await;
        assert!(
            terminal.applied_commands > 0,
            "验证运动开始后按来源期限实际停车"
        );
        assert!(
            terminal.pose_receipt_age_ms < 1_000,
            "接收期限仍有效，来源期限已先到"
        );
        assert!(terminal.message.contains("pose source deadline"));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));

        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn supplied_map_receipt_age_survives_construction_and_rejects_the_future() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState { twist_probe: Some(Vec::new()), ..Default::default() }));
        let (handle, task) = test_scheduler(
            FakeMotion(Arc::clone(&motion_state)),
            FakePlanner::commanding(),
        );
        handle.update_odometry(map_pose()).await.unwrap();
        assert!(matches!(
            handle
                .update_costmap_received(
                    costmap_at(1_000_000_000),
                    Instant::now() + Duration::from_secs(1)
                )
                .await,
            Err(RuntimeError::InvalidInput {
                field: "costmap.received_at",
                ..
            })
        ));
        handle
            .update_costmap_received(
                costmap_at(1_000_000_000),
                Instant::now() - Duration::from_secs(2),
            )
            .await
            .unwrap();
        assert!(!handle.status().planner_inputs_fresh);
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        wait_for_input_pause(&handle).await;
        assert!(motion_state
            .lock()
            .unwrap()
            .twist_probe.as_ref().unwrap()
            .iter()
            .all(|(_, _, twist)| *twist == Twist2::ZERO));
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn verified_fence_resets_the_native_command_boundary_before_the_next_goal() {
        let (mut scheduler,status,_motion,start)=zero_position_surface_scheduler().await;
        scheduler.apply_controller_twist(&status,Twist2 {linear:Point2::new(0.2,0.),angular_z:0.}).await.unwrap();
        scheduler.finish_terminal(&status,TerminalOutcome::Cancelled,SchedulerPhase::Idle).await.unwrap();
        let mut goal=start.clone();goal.pose.position.x+=1.;
        let mut request=session_for("after-fence");request.goal=ExecutionGoal::Body(goal.clone());
        scheduler.start_session(&status,request).await.unwrap();
        let mut route=path_for("after-fence",1);route.path=ExecutionPath::Surface(Path3D {poses:vec![start.clone(),goal]});
        scheduler.submit_path(&status,route).await.unwrap();
        scheduler.control_tick(&status).await.unwrap();
        let command=scheduler.status.last_applied_twist.expect("应交付新目标首拍");
        assert!(command.linear.x>=0.&&command.linear.x<=0.5*0.05+1e-9,
            "已确认停车后不能沿用上一目标0.2m/s命令边界：{command:?}");
        let mut next = start;
        let mut previous = command;
        let mut moving = false;
        for _ in 0..20 {
            next.stamp.nanos += 50_000_000;
            scheduler.observe_odometry(&status, next.clone(), None).unwrap();
            scheduler.control_tick(&status).await.unwrap();
            let following = scheduler.status.last_applied_twist.unwrap();
            assert!(following.linear.x >= 0. && following.linear.x <= previous.linear.x + 0.5 * 0.05 + 1e-9,
                "新目标每拍都必须遵守从已确认零速起步的加速度限制：{following:?}");
            if following.linear.x > 0. { moving = true; break; }
            previous = following;
        }
        assert!(moving, "作者A完成初始化后必须恢复实际运动");
    }

    #[tokio::test]
    async fn actual_surface_backend_stops_for_changed_map_and_resumes_only_new_path() {
        let mut local_config = SurfaceFollowerConfig::new(0.4);
        local_config.controller.speed = 0.2;
        local_config.controller.goal_tolerance = 0.12;
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            planner_worker::PlannerWorker::new(crate::native_test_helpers::author_static_map_follower(local_config)).unwrap(),
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),
                pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
                ..SchedulerConfig::default()
            },
        )
        .unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        let mut start = nuwax_nav_astar::helpers::make_pose_stamped("map", 0.55, 1.55, 0.0);
        start.pose.position.z = 0.5;
        start.stamp = Timestamp::from_unix(1, 0);
        let mut goal = start.clone();
        goal.pose.position.x = 4.55;
        let surface_path = Path3D {
            poses: vec![start.clone(), goal.clone()],
        };
        handle.update_odometry(start.clone()).await.unwrap();
        // Fixture: continuously observe the unchanged synthetic pose
        // with timestamps advanced by actual elapsed monotonic time; keep real TTL.
        let observation_clock = Instant::now();
        let observation_handle = handle.clone();
        let observation_pose = start.clone();
        let observation_task = tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_millis(10)).await;
                let elapsed = observation_clock.elapsed();
                let mut pose = observation_pose.clone();
                pose.stamp = Timestamp::from_unix(1 + elapsed.as_secs() as i64, elapsed.subsec_nanos());
                if observation_handle.update_odometry(pose).await.is_err() { break; }
            }
        });

        handle
            .update_costmap(CostmapObservation {
                costmap: surface_follower::prepared_test_surface(surface_follower::test_snapshot(0.0, false)),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .unwrap();
        let mut surface_session = session();
        surface_session.goal = ExecutionGoal::Body(goal);
        handle.start_session(surface_session).await.unwrap();
        let mut first_path = path();
        first_path.path = ExecutionPath::Surface(surface_path.clone());
        handle.submit_path(first_path).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while !handle.status().last_applied_twist.is_some_and(|command|command.linear.x>0.0) {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        })
        .await
        .unwrap();
        assert!(handle.status().last_applied_twist.unwrap().linear.x > 0.0);
        handle
            .update_costmap(CostmapObservation {
                costmap: surface_follower::prepared_test_surface(Arc::new(surface_follower::test_snapshot(0.0, true)
                    .with_recent_obstacle_points(1,1,0,&[],[(0,[0.55,1.55,0.5])]).unwrap())),
                source_timestamp_ns: 2_000_000_000,
            })
            .await
            .unwrap();
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        let stopped_count = handle.status().applied_commands;
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(handle.status().last_terminal, None);
        handle
            .update_costmap(CostmapObservation {
                costmap: surface_follower::prepared_test_surface(surface_follower::test_snapshot(0.0, false)),
                source_timestamp_ns: 3_000_000_000,
            })
            .await
            .unwrap();
        let mut replacement = path_for("goal-1", 2);
        replacement.path = ExecutionPath::Surface(surface_path);
        handle.submit_path(replacement).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().applied_commands <= stopped_count || !handle.status().last_applied_twist.is_some_and(|command|command.linear.x>0.0) {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        })
        .await
        .unwrap();
        assert!(handle.status().last_applied_twist.unwrap().linear.x > 0.0);
        handle.cancel("goal-1").await.unwrap();

        // 原目标仍在上层，收到仅包含下层当前位置的路径不能报告已到达。
        let mut upper_goal_session = session_for("upper-goal");
        upper_goal_session.goal = ExecutionGoal::Body(start.clone());
        upper_goal_session.goal.as_body_mut().unwrap().pose.position.z += 3.0;
        handle.start_session(upper_goal_session).await.unwrap();
        let mut partial = path_for("upper-goal", 1);
        partial.path = ExecutionPath::Surface(Path3D { poses: vec![start] });
        handle.submit_path(partial).await.unwrap();
        wait_for_phase(&handle, SchedulerPhase::AwaitingPath).await;
        assert_eq!(handle.status().last_terminal, None);
        assert_eq!(handle.status().last_applied_twist, Some(Twist2::ZERO));
        handle.cancel("upper-goal").await.unwrap();
        observation_task.abort();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn temporary_map_source_loss_stops_and_recovers_the_same_goal_and_path() {
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planning = planner.state.clone();
        let (handle, task) = spawn_scheduler(NavScheduler::new(
            FakeMotion(motion.clone()), planner, SchedulerConfig::default(),
        ).unwrap());
        handle.update_odometry(map_pose()).await.unwrap();
        let old_receipt = Instant::now();
        handle.update_costmap_received(costmap_at(1), old_receipt).await.unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        let stopped_planner_count = planning.lock().unwrap().stops;
        handle.invalidate_map_received("同域完整输入暂缺", Instant::now(), MapInvalidationKind::TemporaryUnavailable).await.unwrap();
        let paused = wait_for_input_pause(&handle).await;
        assert_eq!(paused.session_id.as_deref(), Some("goal-1"));
        assert_eq!(paused.path_revision, 1);

        assert_eq!(planning.lock().unwrap().stops, stopped_planner_count, "同域暂缺不能删除现有路径");
        assert!(matches!(handle.update_costmap_received(costmap_at(2), old_receipt).await, Err(RuntimeError::StaleInput(_))));
        handle.update_costmap(costmap_at(3)).await.unwrap();
        tokio::time::timeout(Duration::from_secs(1), async {
            while !handle.status().last_applied_twist.is_some_and(|twist| twist != Twist2::ZERO) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.expect("同域来源恢复后原路径继续");
        assert_eq!(handle.status().path_revision, 1);
        assert_eq!(motion.lock().unwrap().calls.iter().filter(|call| call.starts_with("acquire:")).count(), 1);
        handle.cancel("goal-1").await.unwrap();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn explicit_map_invalidation_preserves_owner_and_fences_only_active_motion() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler(
            FakeMotion(Arc::clone(&motion_state)),
            FakePlanner::commanding(),
        );
        handle.update_odometry(map_pose()).await.unwrap();
        let old_receipt = Instant::now();
        assert!(matches!(
            handle
                .invalidate_map_received("未来失效事件", old_receipt + Duration::from_secs(1), MapInvalidationKind::Invalidated)
                .await,
            Err(RuntimeError::InvalidInput {
                field: "map.invalidated_at",
                ..
            })
        ));
        handle
            .update_costmap_received(costmap_at(1_000_000_000), old_receipt)
            .await
            .unwrap();
        let invalidation_receipt = Instant::now();
        handle
            .invalidate_map_received("空闲时切换地图", invalidation_receipt, MapInvalidationKind::Invalidated)
            .await
            .unwrap();
        assert_eq!(handle.status().phase, SchedulerPhase::Idle);
        assert!(!handle.status().planner_inputs_fresh);
        assert!(
            motion_state.lock().unwrap().calls.is_empty(),
            "空闲失效不能申请或围栏租约"
        );
        assert!(matches!(
            handle
                .update_costmap_received(costmap_at(2_000_000_000), old_receipt)
                .await,
            Err(RuntimeError::StaleInput(_))
        ));

        // 同一来源事件可同时表示新地图域；相同原接收时刻的有效新图可立即接入。
        handle
            .update_costmap_received(costmap_at(2_000_000_000), invalidation_receipt)
            .await
            .unwrap();
        handle.start_session(session()).await.unwrap();
        handle.submit_path(path()).await.unwrap();
        handle.invalidate_map("活动地图被清空").await.unwrap();
        let stopped = handle.status();
        assert_eq!(stopped.last_terminal, Some(TerminalOutcome::SafetyFailed));
        assert!(stopped.message.starts_with("SafetyFailed cause=MapSourceInvalidated:"));
        assert_ne!(stopped.last_terminal, Some(TerminalOutcome::Cancelled));
        assert_eq!(stopped.last_applied_twist, Some(Twist2::ZERO));

        assert!(!stopped.planner_inputs_fresh);
        assert!(!task.is_finished());

        handle.update_odometry(map_pose_at(3)).await.unwrap();
        handle
            .update_costmap(costmap_at(3_000_000_000))
            .await
            .unwrap();
        assert!(handle.status().session_id.is_none(), "新地图不能自动复活旧图目标");
        handle
            .start_session(session_for("new-map-goal"))
            .await
            .unwrap();
        handle
            .submit_path(path_for("new-map-goal", 1))
            .await
            .unwrap();
        handle.cancel("new-map-goal").await.unwrap();
        assert_eq!(
            handle.status().last_terminal,
            Some(TerminalOutcome::Cancelled)
        );
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn surface_backend_requires_explicit_configuration_before_connecting_motion() {
        let result = spawn_production_scheduler_with_backend(
            "/not-a-motion-socket",
            SchedulerConfig::default(),
            LocalExecutionBackendKind::SurfaceInProcess,
        )
        .await;
        assert!(matches!(
            result,
            Err(RuntimeError::InvalidInput {
                field: "robot_geometry",
                ..
            })
        ));
        let mut basic =
            BasicInProcessLocalBackend::production(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() });
        let surface_path = Path3D {
            poses: vec![map_pose()],
        };
        assert!(basic
            .replace_execution_path(ExecutionPath::Surface(surface_path), None, map_pose())
            .is_err());
    }

    #[tokio::test]
    async fn default_session_waits_beyond_five_seconds_then_controls_and_cancels() {
        let motion = FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let scheduler = NavScheduler::new(motion, FakePlanner::commanding(), SchedulerConfig::default()).unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.unwrap();
        handle.update_costmap(costmap_at(1_000_000_000)).await.unwrap();
        handle.start_session(session_for("persistent-path")).await.unwrap();
        tokio::time::sleep(Duration::from_millis(5100)).await;
        assert_eq!(handle.status().phase, SchedulerPhase::AwaitingPath);
        assert!(handle.status().last_terminal.is_none());
        handle.submit_path(path_for("persistent-path", 1)).await.unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(handle.status().phase, SchedulerPhase::Controlling);
        assert!(handle.status().last_applied_twist.is_some_and(|twist|twist != Twist2::ZERO));
        handle.cancel("persistent-path").await.unwrap();
        wait_for_terminal(&handle, TerminalOutcome::Cancelled).await;
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn missing_first_path_fences_and_allows_the_next_session() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        let planner_state = planner.state.clone();
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::clone(&motion_state)),
            planner,
            SchedulerConfig {
                tick_interval: Duration::from_millis(5),

                path_wait_timeout: Duration::from_millis(30),
                    pose_ttl: Duration::from_secs(1),
                costmap_ttl: Duration::from_secs(1),
            },
        )
        .expect("scheduler with path deadline");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;

        handle
            .start_session(session_for("missing-path"))
            .await
            .expect("first session");
        let terminal = wait_for_terminal(&handle, TerminalOutcome::PlannerFailed).await;
        assert_eq!(terminal.phase, SchedulerPhase::Failed);

        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .any(|call| call.starts_with("release:")));
        assert!(!task.is_finished(), "path timeout must not stop the owner");

        planner_state.lock().expect("planner").step = PlannerStep::Succeeded;
        handle
            .update_odometry(map_pose_at(2))
            .await
            .expect("retry pose");
        handle
            .update_costmap(costmap_at(2_000_000_000))
            .await
            .expect("retry costmap");
        handle
            .start_session(session_for("after-timeout"))
            .await
            .expect("session after path timeout");
        handle
            .submit_path(path_for("after-timeout", 1))
            .await
            .expect("path after timeout");
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;

        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn exploration_second_frontier_runs_after_first_child_terminal() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let planner = FakePlanner::commanding();
        planner.state.lock().expect("planner").step = PlannerStep::Succeeded;
        let (handle, task) = test_scheduler(FakeMotion(motion_state), planner);
        wait_for_sink(&handle).await;
        handle
            .update_odometry(map_pose_at(1))
            .await
            .expect("pose 1");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("map 1");
        for (index, child) in ["explore:frontier-1", "explore:frontier-2"]
            .into_iter()
            .enumerate()
        {
            handle
                .start_session(session_for(child))
                .await
                .expect("frontier child accepted");
            handle
                .submit_path(path_for(child, 1))
                .await
                .expect("frontier path accepted");
        wait_for_terminal(&handle, TerminalOutcome::Succeeded).await;
            if index == 0 {
                handle
                    .update_odometry(map_pose_at(2))
                    .await
                    .expect("pose 2");
                handle
                    .update_costmap(costmap_at(2_000_000_000))
                    .await
                    .expect("map 2");
            }
        }
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn default_map_receipt_keeps_observed_map_without_age_expiry() {
        let scheduler = NavScheduler::new(
            FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            FakePlanner::commanding(),
            SchedulerConfig::default(),
        )
        .expect("默认生产调度器配置");
        let (handle, task) = spawn_scheduler(scheduler);
        wait_for_sink(&handle).await;
        let mut latest_map_source = 0;
        for tick in 0_i64..=20 {
            handle.update_odometry(map_pose_at(tick + 1)).await.unwrap();
            if tick % 5 == 0 {
                latest_map_source = 77_000_000_000 + tick * 50_000_000;
                handle
                    .update_costmap(costmap_at(latest_map_source))
                    .await
                    .unwrap();
            }
            assert!(
                handle.status().planner_inputs_fresh,
                "250毫秒实际地图续收不能周期性失效"
            );
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        for tick in 21_i64..=33 {
            handle.update_odometry(map_pose_at(tick + 1)).await.unwrap();
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        let expired = handle.status();
        assert!(expired.planner_inputs_fresh);
        assert_eq!(expired.costmap_source_timestamp_ns, latest_map_source);
        assert!(expired.costmap_receipt_age_ms > 500);
        assert!(
            matches!(
                handle.update_costmap(costmap_at(latest_map_source)).await,
                Err(RuntimeError::StaleInput(_))
            ),
            "旧缓存不能通过相同源时间重放续期"
        );
        assert!(handle.status().planner_inputs_fresh);
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn pose_stream_loss_pauses_without_applied_heartbeat_refresh() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler_with_ttls(
            FakeMotion(motion_state.clone()),
            FakePlanner::commanding(),
            Duration::from_millis(30),
            Duration::from_millis(200),
        );
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.expect("pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("path");
        let loss_started = std::time::Instant::now();
        let terminal = wait_for_input_pause(&handle).await;
        assert!(
            loss_started.elapsed() <= Duration::from_millis(100),
            "pose loss must park within TTL + one tick + fence"
        );
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));

        assert!(terminal
            .message
            .starts_with("等待新鲜导航输入:"));
        assert!(terminal.message.contains("pose receipt age"));
        assert!(!terminal.planner_inputs_fresh);
        assert!(motion_state
            .lock()
            .expect("motion")
            .calls
            .iter()
            .all(|call| !call.starts_with("release:")));
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn first_failure_terminal_keeps_the_reason_after_fencing() {
        for terminal in [TerminalOutcome::PlannerFailed,TerminalOutcome::SafetyFailed] {
            let mut scheduler = NavScheduler::new(
                FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
                FakePlanner::commanding(),SchedulerConfig::default()).unwrap();
            scheduler.session=Some(session());
            scheduler.lease_active=true;
            scheduler.status.message="原始失败原因：目标前方没有可行支撑".into();
            let (sender, observed)=watch::channel(scheduler.status.clone());
            scheduler.finish_terminal(&sender,terminal,SchedulerPhase::Failed).await.unwrap();
            let status=observed.borrow();
            assert_eq!(status.last_terminal,Some(terminal));

            assert!(status.message.contains("目标前方没有可行支撑"),"首次终态丢失原因: {}",status.message);
        }
    }

    #[tokio::test]
    async fn costmap_stream_loss_pauses_within_configured_deadline() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler_with_ttls(
            FakeMotion(motion_state),
            FakePlanner::commanding(),
            Duration::from_millis(200),
            Duration::from_millis(30),
        );
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.expect("pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("path");
        let loss_started = std::time::Instant::now();
        let terminal = wait_for_input_pause(&handle).await;
        assert!(loss_started.elapsed() <= Duration::from_millis(100));
        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert!(!terminal.planner_inputs_fresh);
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn replayed_pose_source_timestamp_does_not_extend_ttl() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState::default()));
        let (handle, task) = test_scheduler_with_ttls(
            FakeMotion(motion_state),
            FakePlanner::commanding(),
            Duration::from_millis(40),
            Duration::from_millis(200),
        );
        wait_for_sink(&handle).await;
        handle.update_odometry(map_pose_at(1)).await.expect("pose");
        handle
            .update_costmap(costmap_at(1_000_000_000))
            .await
            .expect("costmap");
        handle.start_session(session()).await.expect("session");
        handle.submit_path(path()).await.expect("path");
        tokio::time::sleep(Duration::from_millis(20)).await;
        let replay = handle
            .update_odometry(map_pose_at(1))
            .await
            .expect_err("same source stamp must be rejected");
        assert!(matches!(replay, RuntimeError::StaleInput(_)));
        wait_for_input_pause(&handle).await;
        handle.shutdown().await.expect("shutdown");
        task.await.expect("join").expect("task result");
    }

    #[tokio::test]
    async fn ambiguous_applied_failure_revokes_and_zeroes_before_task_failure() {
        let motion_state = Arc::new(Mutex::new(FakeMotionState {
            fail_twist: true,
            ..FakeMotionState::default()
        }));
        let (handle, task) =
            test_scheduler(FakeMotion(motion_state.clone()), FakePlanner::commanding());
        wait_for_sink(&handle).await;
        handle.start_session(session()).await.expect("session");
        handle
            .update_costmap(CostmapObservation {
                costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 4, 4, 0.1)),
                source_timestamp_ns: 1_000_000_000,
            })
            .await
            .expect("costmap");
        handle.update_odometry(map_pose()).await.expect("map pose");
        handle.submit_path(path()).await.expect("path");

        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.status().last_terminal != Some(TerminalOutcome::SafetyFailed) {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.expect("失败必须先确认归零终态");
        assert!(!task.is_finished(), "已确认停车的目标失败不能结束调度拥有方");
        let calls = motion_state.lock().expect("motion").calls.clone();
        let twist = calls
            .iter()
            .position(|call| call.starts_with("twist:"))
            .expect("twist call");
        let fence = calls
            .iter()
            .position(|call| call.starts_with("release:"))
            .expect("fence call");
        assert!(
            fence > twist,
            "ambiguous command failure must be fenced afterwards"
        );
        assert!(handle.status().motion_sink_ready);
        let terminal = handle.status();
        assert_eq!(terminal.last_terminal, Some(TerminalOutcome::SafetyFailed));

        assert_eq!(terminal.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(
            terminal.message,
            "SafetyFailed cause=Motion: motion guard operation failed: 注入速度发送失败"
        );
        motion_state.lock().unwrap().fail_twist = false;
        handle.start_session(session()).await.expect("停车后仍可接纳后续目标");
        handle.cancel(&session().session_id).await.unwrap();
        handle.shutdown().await.unwrap();
        task.await.unwrap().unwrap();
    }












    include!("dispatch_handoff_tests.rs");

}

#[cfg(test)]
#[test]
fn remaining_route_distance_preserves_bends_and_height() {
    let route = [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [3.0, 4.0, 0.0], [3.0, 4.0, 2.0]];
    assert!((remaining_route_distance(&route, [1.0, 0.0, 0.0])-8.0).abs()<1e-12);
    assert!((remaining_route_distance(&route, [3.0, 2.0, 0.0])-4.0).abs()<1e-12);
    assert_eq!(remaining_route_distance(&route, [3.0, 4.0, 2.0]), 0.0);
}
