//! Wavefront runtime(tokio + stream/RPC begin/end/stop loop)。
//!
//! 首版仅提供 stub + 类型 + 公开字段;具体 stream/transport 接线
//! 由 `nuwax-exploration` runtime adapter 负责,不在本 crate 实现。
//! 公共输出只允许 `FrontierGoal`,**不**生成 `NavigationGoal` 或 `FrontierGoalInput`
//! (方案 §4.11 / §4.12)。
//!
//! ## 行为约定(方案 §4.11 + 审查 BLOCKER-4 修正)
//!
//! - `begin` 重复调用返回 `AlreadyActive`。
//! - `end` 在非 Active 下返回 `NotActive`。
//! - `request_stop_movement` 取消探索并释放 active / movement hold;不发布 stop goal(归属上层)。
//! - **`report_goal_reached(true)`** 显式标记当前 goal 已达成(下一 tick 重新选择)。
//! - **`goal_timeout`**:当最近一次 publish 时间距 `now` 超过 `cfg.goal_timeout_ms`,
//!   视为 timeout(下一 tick 重新选择);不立即 Stop,允许多次 timeout 后累计到
//!   `cfg.max_consecutive_failures` 才 Stop,匹配方案 §4.11 "连续失败" 语义。
//! - `tick` 在缺 costmap / odom 时每 500ms 占位返回 `NoInput`,不报错、不静默丢失。
//! - `run_until` / `Drop` 必须保证 active / movement hold 被释放。
//! - 连续失败:`goals_published >= 2` 且 `consecutive_failures >= max_consecutive_failures` 触发 stop。
//! - **`StopReason::UserCancelled`** 由 `request_stop_movement` 触发,不复用
//!   `NoInformationGain`(`MEDIUM-2` 修正)。

use std::time::Duration;

use nuwax_nav_types_messages::geometry::{Point2, PoseStamped, Timestamp};
use nuwax_nav_types_messages::grid::OccupancyGrid2D;
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::config::WavefrontConfig;
use crate::detector::WavefrontError;
use crate::frontier_goal::{FrontierGoalIdSource, GoalSelection, StopReason};
use crate::selector::WavefrontGoalSelector;
use crate::session::ExplorationSession;

/// Runtime 公开状态(测试用)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimePhase {
    /// 尚未 begin
    Idle,
    /// 已 begin,正在循环
    Active,
    /// 已 end / 已 stop
    Stopped,
}

/// Runtime 公开错误(只占位)
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RuntimeError {
    /// 重复 begin
    AlreadyActive,
    /// 在未 begin 状态下 end / stop_movement
    NotActive,
}

/// Runtime 缺哪类输入(用于 `NoInput` 分支)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissingInput {
    /// 缺 costmap
    Costmap,
    /// 缺 odom
    Odometry,
    /// costmap 和 odom 都缺
    Both,
}

/// 单次 `tick` 输出(非 goal 分支)
#[derive(Debug, Clone, PartialEq)]
pub enum RuntimeTickOutcome {
    /// 已选择并发布 frontier goal
    Goal(FrontierGoal),
    /// 缺 costmap/odom,占位等待(每 500ms tick)
    NoInput {
        /// 缺哪类输入
        waited_for: MissingInput,
    },
    /// 当前未 active
    Idle,
    /// 探索需要停止(已释放 active / movement hold)
    Stop {
        /// 停止原因(no-gain / no-reachable-free / user-cancel)
        reason: StopReason,
    },
    /// 算法层错误(不释放 active;由 caller 决定如何处置)
    Error(String),
}

/// Runtime 公共输入
#[derive(Debug, Clone, Copy)]
pub struct RuntimeInputs<'a> {
    /// 当前 costmap(None 表示缺)
    pub costmap: Option<&'a OccupancyGrid2D>,
    /// 当前 odom / pose(None 表示缺)
    pub odom: Option<&'a PoseStamped>,
    /// 当前时间戳(传入 `select_goal_at` 作 `now`)
    pub now: Timestamp,
}

impl<'a> RuntimeInputs<'a> {
    /// 缺什么输入
    #[must_use]
    pub fn missing(&self) -> Option<MissingInput> {
        match (self.costmap, self.odom) {
            (None, None) => Some(MissingInput::Both),
            (None, _) => Some(MissingInput::Costmap),
            (_, None) => Some(MissingInput::Odometry),
            (Some(_), Some(_)) => None,
        }
    }
}

/// `FrontierGoal` 发送目标 trait(由 `nuwax-exploration` 提供具体实现)。
pub trait WavefrontGoalSink {
    /// 发送一个 frontier goal;runtime 不消费返回值。
    fn publish(&mut self, goal: FrontierGoal);
}

/// 单次发布的 active goal 状态机(rework22 P0-10)。
///
/// `InFlight` 表示已发布但尚未收到 reached / failed / timeout 反馈;
/// 在该状态下 `tick()` 不会重复发布新 goal,直到下游上报终止信号。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalStatus {
    /// 已发布,等待下游反馈
    InFlight,
    /// 下游报告已到达
    Reached,
    /// 下游报告失败(导航链路 / 路径 / 校验等)
    Failed,
    /// 超过 `cfg.goal_timeout_ms` 未收到反馈,被 runtime 自身视作超时
    Timeout,
    /// 上层主动取消(例如 user cancel / stop_movement)
    Cancelled,
}

/// 当前已发布但未终结的 active goal(rework22 P0-10 active-goal gate)。
///
/// `pending_goal` 字段保留该信息,直到 runtime 收到 `Reached / Failed /
/// Timeout / Cancelled` 终结信号。`tick()` 在 `pending_goal.status == InFlight`
/// 时跳过重新选 goal + publish 步骤,避免在下游尚未处理完旧 goal 时堆积新目标。
#[derive(Debug, Clone)]
pub struct PendingGoal {
    /// 最近一次发布的 goal
    pub target: FrontierGoal,
    /// 发布时刻(runtime `Timestamp`)
    pub published_at: Timestamp,
    /// 当前状态
    pub status: GoalStatus,
}

/// Runtime diagnostics(测试可断言)
#[derive(Debug, Clone)]
pub struct RuntimeDiagnostics {
    /// 当前 phase
    pub phase: RuntimePhase,
    /// 是否 active
    pub active: bool,
    /// movement hold 是否仍然持有
    pub movement_held: bool,
    /// 上一次 `tick` 输出
    pub last_outcome: Option<String>,
    /// 上一次 `NoInput` 等待的输入类型
    pub last_no_input_wait: Option<MissingInput>,
    /// 已处理 tick 数
    pub ticks_processed: u64,
    /// 已发布 goal 数
    pub goals_published: u64,
    /// stop 计数
    pub stop_count: u64,
}

impl Default for RuntimeDiagnostics {
    fn default() -> Self {
        Self {
            phase: RuntimePhase::Idle,
            active: false,
            movement_held: false,
            last_outcome: None,
            last_no_input_wait: None,
            ticks_processed: 0,
            goals_published: 0,
            stop_count: 0,
        }
    }
}

/// `run_until` 摘要
#[derive(Debug, Clone)]
pub struct RuntimeSummary {
    /// 已处理 tick 数
    pub ticks_processed: u64,
    /// 已发布 goal 数
    pub goals_published: u64,
    /// 退出时 phase
    pub last_phase: RuntimePhase,
    /// stop 计数
    pub stop_count: u64,
}

impl Default for RuntimeSummary {
    fn default() -> Self {
        Self {
            ticks_processed: 0,
            goals_published: 0,
            last_phase: RuntimePhase::Idle,
            stop_count: 0,
        }
    }
}

/// 缺输入占位 tick 周期(方案 §4.11:500ms)
pub const NO_INPUT_TICK: Duration = Duration::from_millis(500);

/// Wavefront runtime 骨架。
///
/// 字段语义:
/// - `selector`:唯一算法 owner(detector + scorer 内部组合)
/// - `goal_sink`:`FrontierGoal` 发送目标(由 `nuwax-exploration` 注入)
/// - `id_source`:可注入(生产 uuid,测试 deterministic)
/// - `config`:`WavefrontConfig`
/// - `state`:`ExplorationSession`
/// - `phase`:当前 phase
/// - `stop_movement_flag`:用户请求 stop movement(下一 tick 生效)
/// - `goal_reached_flag`:用户报告当前 goal 已达成(下一 tick 重新选择,不记 failure)
/// - `last_goal_published_at`:最近一次 publish 的 timestamp(用于 goal_timeout 判定)
/// - `timeout_failure_count`:连续 goal_timeout 计数(超过 max_consecutive_failures 触发 stop)
pub struct WavefrontExplorerRuntime<S, G, I>
where
    S: AsMut<WavefrontGoalSelector> + Send,
    G: WavefrontGoalSink + Send,
    I: FrontierGoalIdSource + Send,
{
    selector: S,
    goal_sink: G,
    id_source: I,
    config: WavefrontConfig,
    state: ExplorationSession,
    phase: RuntimePhase,
    active: bool,
    movement_held: bool,
    stop_movement_flag: bool,
    goal_reached_flag: bool,
    last_goal_published_at: Option<Timestamp>,
    timeout_failure_count: u32,
    /// rework22 P0-10:最近一次发布但未终结的 active goal。
    /// 字段存在表示"已有 active goal",runtime 在 `InFlight` 状态不会重复发布。
    pending_goal: Option<PendingGoal>,
    last_outcome: Option<String>,
    last_no_input_wait: Option<MissingInput>,
    ticks_processed: u64,
    goals_published: u64,
    stop_count: u64,
}

/// `Drop` guard:无论 panic / early return,必须释放 active / movement hold。
impl<S, G, I> Drop for WavefrontExplorerRuntime<S, G, I>
where
    S: AsMut<WavefrontGoalSelector> + Send,
    G: WavefrontGoalSink + Send,
    I: FrontierGoalIdSource + Send,
{
    fn drop(&mut self) {
        // 强制释放所有 guard;若已经 end 过,no-op。
        self.active = false;
        self.movement_held = false;
        if self.phase == RuntimePhase::Active {
            self.phase = RuntimePhase::Stopped;
        }
    }
}

impl<S, G, I> WavefrontExplorerRuntime<S, G, I>
where
    S: AsMut<WavefrontGoalSelector> + Send,
    G: WavefrontGoalSink + Send,
    I: FrontierGoalIdSource + Send,
{
    /// 构造 runtime
    #[must_use]
    pub fn new(selector: S, goal_sink: G, id_source: I, config: WavefrontConfig) -> Self {
        Self {
            selector,
            goal_sink,
            id_source,
            config,
            state: ExplorationSession::new(),
            phase: RuntimePhase::Idle,
            active: false,
            movement_held: false,
            stop_movement_flag: false,
            goal_reached_flag: false,
            last_goal_published_at: None,
            timeout_failure_count: 0,
            pending_goal: None,
            last_outcome: None,
            last_no_input_wait: None,
            ticks_processed: 0,
            goals_published: 0,
            stop_count: 0,
        }
    }

    /// 当前 phase
    #[must_use]
    pub fn phase(&self) -> RuntimePhase {
        self.phase
    }

    /// 是否 active
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// movement hold 是否仍然持有
    #[must_use]
    pub fn is_movement_held(&self) -> bool {
        self.movement_held
    }

    /// diagnostics 快照
    #[must_use]
    pub fn diagnostics(&self) -> RuntimeDiagnostics {
        RuntimeDiagnostics {
            phase: self.phase,
            active: self.active,
            movement_held: self.movement_held,
            last_outcome: self.last_outcome.clone(),
            last_no_input_wait: self.last_no_input_wait,
            ticks_processed: self.ticks_processed,
            goals_published: self.goals_published,
            stop_count: self.stop_count,
        }
    }

    /// begin(若已 active 返回 `AlreadyActive`)
    pub fn begin(&mut self) -> Result<(), RuntimeError> {
        if self.phase == RuntimePhase::Active {
            return Err(RuntimeError::AlreadyActive);
        }
        self.state.reset();
        self.stop_movement_flag = false;
        self.goal_reached_flag = false;
        self.last_goal_published_at = None;
        self.timeout_failure_count = 0;
        self.pending_goal = None;
        self.active = true;
        self.movement_held = true;
        self.phase = RuntimePhase::Active;
        Ok(())
    }

    /// end(若未 active 返回 `NotActive`);释放 active / movement hold
    pub fn end(&mut self) -> Result<(), RuntimeError> {
        if self.phase != RuntimePhase::Active {
            return Err(RuntimeError::NotActive);
        }
        self.release_guards();
        self.phase = RuntimePhase::Stopped;
        Ok(())
    }

    /// 请求 stop movement(取消探索并释放 active / movement hold)。
    ///
    /// 仅在 `Active` 下接受;非 active 视为 no-op(`Ok(())`),不返回错误,
    /// 防止 caller 在 begin 之前误用。
    /// 不发布 stop goal — 由 `nuwax-exploration` runtime adapter 或 stack 处理。
    /// 下一 tick 返回 `RuntimeTickOutcome::Stop { reason: UserCancelled }`
    /// (`MEDIUM-2` 修正:不复用 `NoInformationGain`)。
    pub fn request_stop_movement(&mut self) -> Result<(), RuntimeError> {
        if self.phase != RuntimePhase::Active {
            return Ok(());
        }
        self.stop_movement_flag = true;
        Ok(())
    }

    /// 报告当前 goal 已达成(下一 tick 重新选择,不计 failure)。
    ///
    /// 方案 §4.11 行为:`goal_reached` 输入入口;`nuwax-exploration` runtime adapter
    /// 收到 navigation 反馈后调用本方法。
    /// 非 active 视为 no-op。
    ///
    /// rework22 P0-10:active-goal gate:同步将 `pending_goal` 从 `InFlight` 推到
    /// `Reached` / `Failed`,下一 tick 即可重新选 goal(rework22 P0-11 之前的
    /// `goal_reached_flag` 兼容路径同时保留,新代码以 `pending_goal.status` 为主)。
    pub fn report_goal_reached(&mut self, reached: bool) {
        if self.phase != RuntimePhase::Active {
            return;
        }
        if reached {
            self.goal_reached_flag = true;
            // 重置 timeout 计数(成功路径)
            self.timeout_failure_count = 0;
            self.last_goal_published_at = None;
            // 推进 pending_goal 状态机(若存在)
            if let Some(pg) = self.pending_goal.as_mut() {
                if pg.status == GoalStatus::InFlight {
                    pg.status = GoalStatus::Reached;
                }
            }
        } else {
            // 失败反馈:同样终结 pending_goal,让下一 tick 重新选
            if let Some(pg) = self.pending_goal.as_mut() {
                if pg.status == GoalStatus::InFlight {
                    pg.status = GoalStatus::Failed;
                }
            }
        }
    }

    /// 最近一次 publish 的 timestamp(测试可断言)
    #[must_use]
    pub fn last_goal_published_at(&self) -> Option<Timestamp> {
        self.last_goal_published_at
    }

    /// 连续 goal_timeout 计数(测试可断言)
    #[must_use]
    pub fn timeout_failure_count(&self) -> u32 {
        self.timeout_failure_count
    }

    /// 当前 pending goal(若有)——测试可断言
    #[must_use]
    pub fn pending_goal(&self) -> Option<&PendingGoal> {
        self.pending_goal.as_ref()
    }

    /// 单次 tick(同步;测试用):调 selector,发布 frontier goal。
    ///
    /// 行为(方案 §4.11):
    /// - phase != Active -> `Idle`
    /// - 缺 costmap/odom -> `NoInput`(每 500ms 占位)
    /// - `stop_movement_flag` -> 释放 guard,返回 `Stop { UserCancelled }`(用户取消;`MEDIUM-2` 修正)
    /// - `goal_reached_flag` -> 清零(不发布新 goal,等下一 tick 重新选择)
    /// - 当前 goal 已发布且超过 `cfg.goal_timeout_ms` -> 记 timeout_failure,继续走选择
    /// - 调 `select_goal_at`:
    ///   - `Goal(g)` -> publish + record + return `Goal(g)`
    ///   - `NoFrontier` -> `Idle`(active 保持)
    ///   - `Stop(reason)` -> 释放 guard,return `Stop { reason }`
    /// - 算法层 `Err` -> record_failure,根据 `max_consecutive_failures` 决定是否 stop
    pub fn tick(&mut self, inputs: RuntimeInputs<'_>) -> Result<RuntimeTickOutcome, RuntimeError> {
        self.ticks_processed = self.ticks_processed.saturating_add(1);

        if self.phase != RuntimePhase::Active {
            self.last_outcome = Some("idle".to_string());
            return Ok(RuntimeTickOutcome::Idle);
        }

        // 用户 stop_movement(MEDIUM-2 修正:不复用 NoInformationGain)
        if self.stop_movement_flag {
            self.stop_movement_flag = false;
            // rework22 P0-10:取消任何 active pending_goal
            if let Some(pg) = self.pending_goal.as_mut() {
                if pg.status == GoalStatus::InFlight {
                    pg.status = GoalStatus::Cancelled;
                }
            }
            self.release_guards();
            self.phase = RuntimePhase::Stopped;
            self.stop_count = self.stop_count.saturating_add(1);
            self.last_outcome = Some("stop_movement".to_string());
            return Ok(RuntimeTickOutcome::Stop {
                reason: StopReason::UserCancelled,
            });
        }

        // user already reported goal_reached — consume flag and continue
        if self.goal_reached_flag {
            self.goal_reached_flag = false;
            self.last_outcome = Some("goal_reached".to_string());
            // rework22 P0-10:同时清掉 pending_goal(若仍为 InFlight)
            // report_goal_reached 已把它推到 Reached,这里直接清。
            self.pending_goal = None;
            self.last_goal_published_at = None;
            // 不返回任何 goal;下一 tick 重新走选择
            return Ok(RuntimeTickOutcome::Idle);
        }

        // 缺输入 -> 占位 500ms tick
        if let Some(missing) = inputs.missing() {
            self.last_no_input_wait = Some(missing);
            self.last_outcome = Some("no_input".to_string());
            return Ok(RuntimeTickOutcome::NoInput {
                waited_for: missing,
            });
        }

        // rework22 P0-10 active-goal gate:有 pending_goal 且状态为 InFlight
        // 时跳过重新选 goal + publish,避免在下游尚未处理完旧 goal 时堆积。
        if let Some(pg) = self.pending_goal.as_ref() {
            if pg.status == GoalStatus::InFlight {
                // 仍可能 timeout:沿用既有 goal_timeout 判定
                let elapsed_ns = inputs.now.diff_nanos(pg.published_at);
                let elapsed_ms = (elapsed_ns / 1_000_000) as u64;
                if elapsed_ms > self.config.goal_timeout_ms {
                    // 推进 pending_goal 为 Timeout,清零发布时刻,继续选下一个
                    if let Some(pg_mut) = self.pending_goal.as_mut() {
                        pg_mut.status = GoalStatus::Timeout;
                    }
                    self.timeout_failure_count =
                        self.timeout_failure_count.saturating_add(1);
                    self.last_outcome = Some("goal_timeout".to_string());
                    if self.timeout_failure_count >= self.config.max_consecutive_failures {
                        self.stop_count = self.stop_count.saturating_add(1);
                        self.release_guards();
                        self.phase = RuntimePhase::Stopped;
                        return Ok(RuntimeTickOutcome::Stop {
                            reason: StopReason::ConsecutiveFailuresExceeded,
                        });
                    }
                    self.last_goal_published_at = None;
                    // 清 pending_goal(本 tick 不再视为 active)
                    self.pending_goal = None;
                } else {
                    // 仍在等待下游反馈,不发布新 goal
                    self.last_outcome = Some("goal_in_flight".to_string());
                    return Ok(RuntimeTickOutcome::Idle);
                }
            } else {
                // 已终结(Reached / Failed / Cancelled / Timeout),本 tick
                // 重新选 goal 之前先清掉旧 pending_goal。
                self.pending_goal = None;
                self.last_goal_published_at = None;
            }
        }

        // goal_timeout 检查(方案 §4.11):最近一次 publish 时间距 now 超过 goal_timeout_ms
        // (兼容旧路径;当 pending_goal 为 None 时仍走原逻辑。)
        if self.pending_goal.is_none() {
            if let Some(published_at) = self.last_goal_published_at {
                let elapsed_ns = inputs.now.diff_nanos(published_at);
                let elapsed_ms = (elapsed_ns / 1_000_000) as u64;
                if elapsed_ms > self.config.goal_timeout_ms {
                    self.timeout_failure_count = self.timeout_failure_count.saturating_add(1);
                    self.last_outcome = Some("goal_timeout".to_string());
                    if self.timeout_failure_count >= self.config.max_consecutive_failures {
                        // 连续 timeout 超过 max_consecutive_failures:Stop
                        self.stop_count = self.stop_count.saturating_add(1);
                        self.release_guards();
                        self.phase = RuntimePhase::Stopped;
                        return Ok(RuntimeTickOutcome::Stop {
                            reason: StopReason::ConsecutiveFailuresExceeded,
                        });
                    }
                    // 单次 timeout:把已发布 goal 视为已"放弃",继续选下一个
                    self.last_goal_published_at = None;
                }
            }
        }

        // 必有 costmap & odom
        let costmap = inputs.costmap.expect("checked by missing() above");
        let odom = inputs.odom.expect("checked by missing() above");
        let robot_pose = Point2 {
            x: odom.pose.position.x,
            y: odom.pose.position.y,
        };

        let result = self.selector.as_mut().select_goal_at(
            robot_pose,
            costmap,
            &mut self.state,
            inputs.now,
            &mut self.id_source,
        );

        match result {
            Ok(GoalSelection::Goal(goal)) => {
                self.goal_sink.publish(goal.clone());
                self.state.record_goal_published();
                self.state
                    .mark_explored_goal(goal.pose.pose.position.to_2d());
                self.goals_published = self.goals_published.saturating_add(1);
                self.last_goal_published_at = Some(inputs.now);
                // rework22 P0-10:记录 active goal,后续 tick 不会重复 publish
                self.pending_goal = Some(PendingGoal {
                    target: goal.clone(),
                    published_at: inputs.now,
                    status: GoalStatus::InFlight,
                });
                self.last_outcome = Some("goal".to_string());
                Ok(RuntimeTickOutcome::Goal(goal))
            }
            Ok(GoalSelection::NoFrontier) => {
                self.last_outcome = Some("no_frontier".to_string());
                Ok(RuntimeTickOutcome::Idle)
            }
            Ok(GoalSelection::Stop(reason)) => {
                self.stop_count = self.stop_count.saturating_add(1);
                self.last_outcome = Some("stop".to_string());
                self.release_guards();
                self.phase = RuntimePhase::Stopped;
                Ok(RuntimeTickOutcome::Stop { reason })
            }
            Err(e) => {
                self.state.record_failure();
                self.last_outcome = Some(format!("error:{}", error_label(&e)));
                self.maybe_stop_on_failures()?;
                Ok(RuntimeTickOutcome::Error(error_label(&e)))
            }
        }
    }

    /// 运行直到 `should_exit()` 返回 `true` 或 stop;每轮按 `NO_INPUT_TICK` 推进。
    ///
    /// `inputs_provider` 在每轮 tick 前返回新 `RuntimeInputs`;若返回 `None` 表示缺输入,
    /// 本轮走 `NoInput` 占位。
    pub fn run_until<P, F>(&mut self, mut inputs_provider: P, mut should_exit: F) -> RuntimeSummary
    where
        P: FnMut() -> Option<RuntimeInputs<'static>>,
        F: FnMut() -> bool,
    {
        // 兜底:若 caller 忘记 begin,自动 begin 一次
        if self.phase == RuntimePhase::Idle {
            let _ = self.begin();
        }
        loop {
            if should_exit() {
                break;
            }
            match inputs_provider() {
                Some(inputs) => {
                    // inputs 是 'static lifetime 的借用;直接消费
                    let outcome = self.tick(inputs);
                    if let Ok(RuntimeTickOutcome::Stop { .. }) = outcome {
                        break;
                    }
                }
                None => {
                    // 缺输入占位
                    let no_input = RuntimeInputs::<'static> {
                        costmap: None,
                        odom: None,
                        now: Timestamp::default(),
                    };
                    let _ = self.tick(no_input);
                }
            }
        }
        // loop exit:无论 should_exit 还是 stop,均保证 guard 释放
        if self.phase == RuntimePhase::Active {
            self.release_guards();
            self.phase = RuntimePhase::Stopped;
        }
        RuntimeSummary {
            ticks_processed: self.ticks_processed,
            goals_published: self.goals_published,
            last_phase: self.phase,
            stop_count: self.stop_count,
        }
    }

    /// 取得一个 guard:在 guard 存活期间任何 panic / early return 都会释放 active。
    pub fn guard(&mut self) -> RuntimeGuard<'_, S, G, I> {
        RuntimeGuard { runtime: self }
    }

    /// 释放 active / movement hold
    fn release_guards(&mut self) {
        self.active = false;
        self.movement_held = false;
        // rework22 P0-10:释放 guard 时一并清理 pending_goal,避免下次 begin
        // 误读到 stale active goal。
        self.pending_goal = None;
        self.last_goal_published_at = None;
    }

    /// 连续失败次数超限时自动 stop
    fn maybe_stop_on_failures(&mut self) -> Result<(), RuntimeError> {
        let max_fail = self.config.max_consecutive_failures;
        let published = self.state.goals_published();
        if self.state.consecutive_failures() >= max_fail && published >= 2 {
            self.stop_count = self.stop_count.saturating_add(1);
            self.release_guards();
            self.phase = RuntimePhase::Stopped;
            // 取消后 caller 下一 tick 看到 Idle;这里不发 outcome
        }
        Ok(())
    }
}

/// Drop guard:在 guard drop 时若 runtime 仍 active,强制 end 等价(释放 active / movement hold)。
pub struct RuntimeGuard<'a, S, G, I>
where
    S: AsMut<WavefrontGoalSelector> + Send,
    G: WavefrontGoalSink + Send,
    I: FrontierGoalIdSource + Send,
{
    runtime: &'a mut WavefrontExplorerRuntime<S, G, I>,
}

impl<'a, S, G, I> RuntimeGuard<'a, S, G, I>
where
    S: AsMut<WavefrontGoalSelector> + Send,
    G: WavefrontGoalSink + Send,
    I: FrontierGoalIdSource + Send,
{
    /// 是否 active
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.runtime.is_active()
    }
    /// movement hold 是否仍然持有
    #[must_use]
    pub fn is_movement_held(&self) -> bool {
        self.runtime.is_movement_held()
    }
    /// 取得 diagnostics 引用
    #[must_use]
    pub fn diagnostics(&self) -> RuntimeDiagnostics {
        self.runtime.diagnostics()
    }
    /// 释放 guard(等价于 `runtime.end()` 但不返回错误;若非 active 则 no-op)
    pub fn release(&mut self) {
        if self.runtime.phase == RuntimePhase::Active {
            self.runtime.release_guards();
            self.runtime.phase = RuntimePhase::Stopped;
        }
    }
}

impl<'a, S, G, I> Drop for RuntimeGuard<'a, S, G, I>
where
    S: AsMut<WavefrontGoalSelector> + Send,
    G: WavefrontGoalSink + Send,
    I: FrontierGoalIdSource + Send,
{
    fn drop(&mut self) {
        // panic-safe 释放
        self.runtime.active = false;
        self.runtime.movement_held = false;
        if self.runtime.phase == RuntimePhase::Active {
            self.runtime.phase = RuntimePhase::Stopped;
        }
    }
}

fn error_label(e: &WavefrontError) -> String {
    format!("{e:?}")
}
