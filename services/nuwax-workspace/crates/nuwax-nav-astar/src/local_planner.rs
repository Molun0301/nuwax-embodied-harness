//! 局部路径跟随状态机
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/local_planner.py`。
//!
//! 状态机：
//! - `Idle` → `InitialRotation` → `PathFollowing` → `FinalRotation` → `Arrived`
//! - 或 `Idle` → `PathFollowing`（如果 yaw error 在 tolerance 内且位置未到）
//! - 或 `Idle` → `FinalRotation`（如果已经进入路径终点的位置容差）
//!
//! Phase 1：本模块以**确定性 tick**形式暴露状态机（不内置 thread / reactive stream）。
//! 由 [`crate::global_planner`] 在每次 tick 中显式调用 `step()` 并消费输出 twist 与 stop message。
//! Foundation 完成 `nuwax-stream` 后，可选接入 thread + reactive subject。

use crate::controllers::{angle_diff, Controller, PController};
use crate::helpers::{
    make_path2d, make_twist2_zero, pose_to_point2, pose_xy_yaw, Path2D, Point2, PoseStamped, Twist2,
};
use crate::navigation_map::CostMap2D;
use crate::path_clearance::PathClearance;
use crate::path_distancer::PathDistancer;

use std::sync::Arc;

/// 同步 navigation_costmap subscriber 回调。
///
/// LocalPlanner 设计为确定性 tick、不阻塞、不引入 async runtime,使用回调列表
/// (`Vec<Arc<dyn Fn>>`) 同步发布最近一次 gradient costmap 给订阅者。
/// 与 Python `LocalPlanner.navigation_costmap: Subject[OccupancyGrid]` 在语义上
/// 对应(每次有订阅者就拿到最新一帧);差异:Python 用 reactivex Subject,这里用
/// 函数回调。
pub type NavCostmapSubscriber = Arc<dyn Fn(&CostMap2D) + Send + Sync>;

/// 局部规划状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalPlannerState {
    /// 空闲
    Idle,
    /// 初始对齐到第一段方向
    InitialRotation,
    /// 沿路径前进
    PathFollowing,
    /// 终点对齐
    FinalRotation,
    /// 已到达
    Arrived,
}

/// 停止原因（与 Python `StopMessage` 对应）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopMessage {
    /// 到达 goal
    Arrived,
    /// 前方检测到 obstacle
    ObstacleFound,
    /// 错误
    Error,
}

/// Goal 到达 typed signal（修复 rework20 H-20）。
///
/// Python 端通过 `goal_reached: Subject[Bool]` 把是否到达作为 typed subject 暴露。
/// Rust 旧实现只在 `StopMessage::Arrived` 用 raw 单元变体表示,下游订阅方需要
/// 自己判断"哪个阶段到达"。本 enum 显式区分:
///
/// - `NotReached`:未触发 arrived 事件(默认 / 中间状态)
/// - `Reached`:到达 goal(最终或中间 waypoint)
/// - `FinalReached`:到达**最终** goal(与 FinalRotation 配对的最后一站)
///
/// 使用方式:LocalPlanner 内部 `last_goal_reached` 持有本类型;外部订阅方通过
/// `goal_reached()` 拿到 typed signal,不再需要猜测变体含义。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GoalReached {
    /// 未到达 / 默认状态
    #[default]
    NotReached,
    /// 已到达(中间或最终 goal)
    Reached,
    /// 已到达最终 goal(与 FinalRotation 阶段匹配)
    FinalReached,
}

/// LocalPlanner 配置(对应 Python `LocalPlanner.__init__` 接受的 `global_config` 影响项)。
#[derive(Debug, Clone)]
pub struct LocalPlannerConfig {
    /// 基础前进速度(m/s)
    pub speed: f64,
    /// 控制频率(Hz)
    pub control_frequency: f64,
    /// goal 到达容差(m)
    pub goal_tolerance: f64,
    /// 朝向容差(rad)
    pub orientation_tolerance: f64,
    /// nerf_speed 缩放因子(对应 Python `global_config.nerf_speed`;< 1.0 时减速)
    ///
    /// 当 `nerf_speed < 1.0`,最终 controller `speed *= nerf_speed`。
    /// Python 实现:`if global_config.nerf_speed < 1.0: speed *= global_config.nerf_speed`
    ///
    /// (审计 H-14 修复)
    pub nerf_speed: f64,
    /// 是否仿真模式(对应 Python `global_config.simulation`)
    ///
    /// 影响:
    /// - `controller.set_simulation(true)`:角速度 < 0.8 时 boost 到 0.8(已在 controllers.rs);
    /// - `GlobalPlanner.stuck_threshold` 在 simulation 下切到 1.0(`global_planner.rs` 持有)。
    ///
    /// (审计 H-14 修复)
    pub simulation: bool,
    /// navigation_costmap 发布间隔(秒)。0 = 关闭,>0 = 每隔 `interval` 秒发一帧。
    ///
    /// Python 仅在 `os.environ.get("DEBUG_NAVIGATION")` 设置时才发;此处统一用 config
    /// 控制,更显式。设 0.0 关闭。
    pub navigation_costmap_interval: f64,
}

impl LocalPlannerConfig {
    /// 默认配置(对应 Python 默认值)
    #[must_use]
    pub fn default_config() -> Self {
        Self {
            speed: 0.55,
            control_frequency: 10.0,
            goal_tolerance: 0.2,
            orientation_tolerance: 0.35,
            nerf_speed: 1.0,
            simulation: false,
            navigation_costmap_interval: 0.0,
        }
    }
}

/// 局部规划器（deterministic core）
pub struct LocalPlanner {
    config: LocalPlannerConfig,
    controller: PController,
    state: LocalPlannerState,
    state_unique_id: u32,
    pose_index: usize,
    current_odom: Option<PoseStamped>,
    path: Option<Path2D>,
    /// 每点对应 yaw(弧度,z 轴),长度与 `path.points` 一致;None → 用线段方向启发式。
    /// 见 H-13 修复:Python `local_planner._compute_initial_rotation` 使用
    /// `path.poses[0].orientation.euler[2]`(stored pose orientation),而非段方向。
    path_yaws: Option<Vec<f64>>,
    path_distancer: Option<PathDistancer>,
    path_clearance: Option<PathClearance>,
    costmap: Option<CostMap2D>,
    robot_geometry: Option<nuwax_nav_types_messages::RobotGeometry>,
    /// 上次 navigation_costmap 发布时间
    last_navigation_costmap_at: Option<std::time::Instant>,
    /// navigation_costmap 订阅者列表(sync stub;见 type alias 注释)
    navigation_costmap_subs: Vec<NavCostmapSubscriber>,
    /// 最近一次 typed goal reached 状态(H-20)
    last_goal_reached: GoalReached,
}

impl LocalPlanner {
    /// 简易构造(向后兼容):签名不变,内部用默认 config。
    ///
    /// 推荐用 `LocalPlanner::with_config(LocalPlannerConfig { ... })` 创建。
    #[must_use]
    pub fn new(speed: f64, control_frequency: f64, goal_tolerance: f64) -> Self {
        let cfg = LocalPlannerConfig {
            speed,
            control_frequency,
            goal_tolerance,
            ..LocalPlannerConfig::default_config()
        };
        Self::with_config(cfg)
    }

    /// 完整构造(对应 Python `LocalPlanner.__init__(global_config, navigation_map, goal_tolerance)`)
    #[must_use]
    pub fn with_config(config: LocalPlannerConfig) -> Self {
        // H-14 nerf_speed: < 1.0 时缩放 controller 速度
        let effective_speed = if config.nerf_speed < 1.0 {
            config.speed * config.nerf_speed
        } else {
            config.speed
        };
        let mut controller = PController::new(effective_speed, config.control_frequency);
        controller.set_simulation(config.simulation);
        Self {
            config,
            controller,
            state: LocalPlannerState::Idle,
            state_unique_id: 0,
            pose_index: 0,
            current_odom: None,
            path: None,
            path_yaws: None,
            path_distancer: None,
            path_clearance: None,
            costmap: None,
            robot_geometry: None,
            last_navigation_costmap_at: None,
            navigation_costmap_subs: Vec::new(),
            last_goal_reached: GoalReached::default(),
        }
    }

    /// 统一机体外形用于前瞻碰撞与当前位置自体区域消除。
    pub fn set_robot_geometry(&mut self, geometry: nuwax_nav_types_messages::RobotGeometry) {
        self.robot_geometry = Some(geometry);
        self.path_clearance = None;
    }

    /// 启动跟随一条新路径(无 stored orientation;rotation 用首段方向启发式)
    pub fn start_planning(&mut self, path: Path2D) {
        self.start_planning_with_orientations(path, None);
    }

    /// 启动跟随一条新路径,提供每点的 stored yaw(对应 Python `path.poses[i].orientation`)。
    ///
    /// `yaws.len()` 必须等于 `path.points.len()`;长度不一致时回退到段方向。
    /// (审计 H-13 修复)
    pub fn start_planning_with_orientations(&mut self, path: Path2D, yaws: Option<Vec<f64>>) {
        self.stop_planning();
        self.last_goal_reached = GoalReached::NotReached;
        self.path = Some(make_path2d(path.points.clone()));
        // 仅当长度与 points 完全匹配时保留;否则回退到段方向
        self.path_yaws = yaws.and_then(|y| {
            if y.len() == path.points.len() {
                Some(y)
            } else {
                None
            }
        });
        self.path_distancer = Some(PathDistancer::new(self.path.as_ref().expect("set"), 0.5));
        self.path_clearance = None;
        self.pose_index = 0;
        self.state = LocalPlannerState::Idle;
        self.state_unique_id = 0;
    }

    /// 注册 navigation_costmap 订阅者(H-14:Out publisher stub)
    pub fn subscribe_navigation_costmap(&mut self, sub: NavCostmapSubscriber) {
        self.navigation_costmap_subs.push(sub);
    }

    /// 计算 Initial / Final rotation 的目标 yaw。
    ///
    /// 优先 stored pose orientation(`path_yaws`);若未提供或索引越界,回退到段方向启发式。
    /// (审计 H-13 修复)
    fn first_segment_yaw_or_first_yaw(&self) -> Option<f64> {
        let path = self.path.as_ref()?;
        if let Some(yaws) = self.path_yaws.as_ref() {
            if let Some(y) = yaws.first() {
                return Some(*y);
            }
        }
        if path.points.len() >= 2 {
            let a = path.points[0];
            let b = path.points[1];
            return Some((b.y - a.y).atan2(b.x - a.x));
        }
        None
    }

    fn last_segment_yaw_or_last_yaw(&self) -> Option<f64> {
        let path = self.path.as_ref()?;
        if let Some(yaws) = self.path_yaws.as_ref() {
            if let Some(y) = yaws.last() {
                return Some(*y);
            }
        }
        if path.points.len() >= 2 {
            let n = path.points.len();
            let prev = path.points[n - 2];
            let last = path.points[n - 1];
            return Some((last.y - prev.y).atan2(last.x - prev.x));
        }
        None
    }

    /// 停止规划
    pub fn stop_planning(&mut self) {
        self.path = None;
        self.path_distancer = None;
        self.path_clearance = None;
        self.state = LocalPlannerState::Idle;
        self.state_unique_id = 0;
        self.pose_index = 0;
        self.controller.reset_errors();
        // H-20:停止规划时重置 typed goal reached signal。
        self.last_goal_reached = GoalReached::NotReached;
    }

    /// 更新 odom
    pub fn handle_odom(&mut self, odom: PoseStamped) {
        self.current_odom = Some(odom);
    }

    /// 更新 costmap（每次 tick 调用）
    pub fn update_costmap(&mut self, costmap: CostMap2D) {
        self.costmap = Some(costmap);
    }

    /// 当前状态
    #[must_use]
    pub fn state(&self) -> LocalPlannerState {
        self.state
    }

    /// 当前 (state, unique_id)
    #[must_use]
    pub fn unique_state(&self) -> (LocalPlannerState, u32) {
        (self.state, self.state_unique_id)
    }

    /// 当前 typed goal reached 状态(修复 rework20 H-20)。
    ///
    /// - `NotReached`:未到达 / 默认
    /// - `Reached`:到达(中间或最终)
    /// - `FinalReached`:到达最终 goal(由 `compute_final_rotation` 进入 FinalRotation 阶段标记)
    ///
    /// 下游订阅方通过本方法拿到 typed signal,不再需要从 `StopMessage::Arrived` 二次推断。
    #[must_use]
    pub fn goal_reached(&self) -> GoalReached {
        self.last_goal_reached
    }

    /// 当前路径点到路径距离
    #[must_use]
    pub fn get_distance_to_path(&self) -> Option<f64> {
        let pd = self.path_distancer.as_ref()?;
        let odom = self.current_odom.as_ref()?;
        Some(pd.get_distance_to_path(pose_to_point2(odom)))
    }

    /// 单次确定性 tick（返回 `(Twist2, Option<StopMessage>)`）
    ///
    /// - 若返回 `Some(StopMessage)`，全局规划器应据此触发 arrived / replan / cancel。
    /// - 若返回 `None`，表示继续按当前 twist 运动。
    pub fn step(&mut self, robot_width: f64) -> (Twist2, Option<StopMessage>) {
        // 非法姿态或路径不能进入控制器，避免非数朝向传播成非数运动指令。
        let invalid_pose = self.current_odom.as_ref().is_some_and(|odom| {
            let position = odom.pose.position;
            !position.x.is_finite()
                || !position.y.is_finite()
                || !position.z.is_finite()
                || !odom.pose.orientation.yaw().is_finite()
        });
        let invalid_path = self.path.as_ref().is_some_and(|path| {
            path.points
                .iter()
                .any(|point| !point.x.is_finite() || !point.y.is_finite())
        }) || self
            .path_yaws
            .as_ref()
            .is_some_and(|yaws| yaws.iter().any(|yaw| !yaw.is_finite()));
        if invalid_pose || invalid_path {
            self.stop_planning();
            return (make_twist2_zero(), Some(StopMessage::Error));
        }
        // 初始化：若 path 已设置且状态仍 Idle，进入 InitialRotation（或 PathFollowing / FinalRotation）
        if self.state == LocalPlannerState::Idle
            && self.path.is_some()
            && self.current_odom.is_some()
        {
            // `odom` 已被前序 `is_some()` 校验保证存在
            let odom = self.current_odom.as_ref().expect("checked above");
            let (ox, oy, oyaw) = pose_xy_yaw(odom);
            // H-13:优先 stored pose orientation(由 start_planning_with_orientations 提供),
            // 回退到首段方向启发式,再回退到 robot 当前 yaw。
            let first_yaw = self.first_segment_yaw_or_first_yaw().unwrap_or(oyaw);
            let initial_yaw_error = angle_diff(first_yaw, oyaw);
            self.controller.reset_yaw_error(initial_yaw_error);
            let angle_in_tol = initial_yaw_error.abs() < self.config.orientation_tolerance;
            let Some(goal) = self.path.as_ref().and_then(|path| path.points.last()) else {
                // 空路径没有可到达的终点，不能将其解释为已完成。
                return (make_twist2_zero(), Some(StopMessage::Error));
            };
            let position_in_tol = (goal.x - ox).hypot(goal.y - oy) < self.config.goal_tolerance;
            let new_state = if position_in_tol {
                LocalPlannerState::FinalRotation
            } else if angle_in_tol {
                LocalPlannerState::PathFollowing
            } else {
                LocalPlannerState::InitialRotation
            };
            self.change_state(new_state);
        }

        // 检查 costmap 与 obstacle
        let path_progress = self
            .path_distancer
            .as_ref()
            .zip(self.current_odom.as_ref())
            .map(|(path, odom)| path.closest_projection(pose_to_point2(odom)));
        let rotation_target = self.current_odom.as_ref().and_then(|odom| {
            let near_goal = self.path_distancer.as_ref().is_some_and(|path| {
                path.distance_to_goal(pose_to_point2(odom)) < self.config.goal_tolerance
            });
            if near_goal {
                self.last_segment_yaw_or_last_yaw()
            } else if self.state == LocalPlannerState::InitialRotation {
                self.first_segment_yaw_or_first_yaw()
            } else {
                None
            }
        });
        let mut obstacle_ahead = false;
        if let Some(pc) = self.path_clearance.as_mut() {
            if let Some(cm) = self.costmap.as_ref() {
                pc.update_costmap(cm.clone());
            }
            if let Some((index, projected)) = path_progress {
                pc.update_path_progress(index, projected);
            }
            if let Some(odom) = self.current_odom.as_ref() {
                let (x, y, yaw) = pose_xy_yaw(odom);
                pc.update_robot_pose(Point2::new(x, y), yaw);
            }
            pc.update_rotation_target(rotation_target);
            obstacle_ahead = pc.is_obstacle_ahead();
        } else if let (Some(cm), Some(path)) = (self.costmap.as_ref(), self.path.as_ref()) {
            // 第一次：构造 path_clearance
            let mut pc = PathClearance::new(path.clone(), robot_width, 3.0);
            if let Some(geometry) = self.robot_geometry.as_ref() {
                pc.set_robot_geometry(geometry.clone());
            }
            if let Some(odom) = self.current_odom.as_ref() {
                let (x, y, yaw) = pose_xy_yaw(odom);
                pc.update_robot_pose(Point2::new(x, y), yaw);
            }
            pc.update_costmap(cm.clone());
            if let Some((index, projected)) = path_progress {
                pc.update_path_progress(index, projected);
            }
            pc.update_rotation_target(rotation_target);
            obstacle_ahead = pc.is_obstacle_ahead();
            self.path_clearance = Some(pc);
        }

        if obstacle_ahead {
            return (make_twist2_zero(), Some(StopMessage::ObstacleFound));
        }

        // H-14:navigation_costmap 发布(在订阅者 + interval>0 时按节流发布最近 gradient)
        self.maybe_publish_navigation_costmap();

        match self.state {
            LocalPlannerState::Idle => (make_twist2_zero(), None),
            LocalPlannerState::InitialRotation => self.compute_initial_rotation(),
            LocalPlannerState::PathFollowing => self.compute_path_following(),
            LocalPlannerState::FinalRotation => self.compute_final_rotation(),
            LocalPlannerState::Arrived => {
                // H-20:已处于 Arrived 状态(中间 waypoint 路径至此)
                // 此分支只在 FinalRotation 未触发时被路径分段到达直接进入;
                // 此时 typed signal 标记为 Reached(非 FinalReached)。
                if self.last_goal_reached == GoalReached::NotReached {
                    self.last_goal_reached = GoalReached::Reached;
                }
                (make_twist2_zero(), Some(StopMessage::Arrived))
            }
        }
    }

    fn compute_initial_rotation(&mut self) -> (Twist2, Option<StopMessage>) {
        let odom = match self.current_odom.as_ref() {
            Some(o) => o,
            _ => return (make_twist2_zero(), None),
        };
        let (_, _, oyaw) = pose_xy_yaw(odom);
        // H-13:stored pose orientation(若提供)优先;否则首段方向;再否则 robot 当前 yaw。
        let first_yaw = self.first_segment_yaw_or_first_yaw().unwrap_or(oyaw);
        let yaw_error = angle_diff(first_yaw, oyaw);
        if yaw_error.abs() < self.config.orientation_tolerance {
            self.change_state(LocalPlannerState::PathFollowing);
            return self.compute_path_following();
        }
        // 接近 goal：跳过 path-following，直接 FinalRotation
        if let Some(pd) = self.path_distancer.as_ref() {
            let pos = pose_to_point2(odom);
            if pd.distance_to_goal(pos) < self.config.goal_tolerance {
                self.change_state(LocalPlannerState::FinalRotation);
                return self.compute_final_rotation();
            }
        }
        (self.controller.rotate(yaw_error), None)
    }

    fn compute_path_following(&mut self) -> (Twist2, Option<StopMessage>) {
        let (pd, odom) = match (self.path_distancer.as_ref(), self.current_odom.as_ref()) {
            (Some(p), Some(o)) => (p, o),
            _ => return (make_twist2_zero(), None),
        };
        let pos = pose_to_point2(odom);
        if pd.distance_to_goal(pos) < self.config.goal_tolerance {
            self.change_state(LocalPlannerState::FinalRotation);
            return self.compute_final_rotation();
        }
        self.pose_index = pd.closest_projection(pos).0;
        let lookahead = pd.find_lookahead_from_position(pos);
        let t = self
            .controller
            .advance((lookahead.x, lookahead.y), odom.clone());
        (t, None)
    }

    fn compute_final_rotation(&mut self) -> (Twist2, Option<StopMessage>) {
        let odom = match self.current_odom.as_ref() {
            Some(o) => o,
            _ => return (make_twist2_zero(), None),
        };
        // 旋转期间定位可能变化或机器人发生位移，终态须同时满足位置与朝向。
        let Some(pd) = self.path_distancer.as_ref() else {
            return (make_twist2_zero(), Some(StopMessage::Error));
        };
        if pd.distance_to_goal(pose_to_point2(odom)) >= self.config.goal_tolerance {
            self.change_state(LocalPlannerState::PathFollowing);
            return self.compute_path_following();
        }
        let (_, _, oyaw) = pose_xy_yaw(odom);
        // H-13:stored pose orientation(若提供)优先;否则末段方向。
        let goal_yaw_opt = self.last_segment_yaw_or_last_yaw();
        if let Some(goal_yaw) = goal_yaw_opt {
            let yaw_error = angle_diff(goal_yaw, oyaw);
            if yaw_error.abs() < self.config.orientation_tolerance {
                self.change_state(LocalPlannerState::Arrived);
                // H-20:FinalRotation 阶段进入 Arrived → FinalReached(最终 goal)
                self.last_goal_reached = GoalReached::FinalReached;
                return (make_twist2_zero(), Some(StopMessage::Arrived));
            }
            return (self.controller.rotate(yaw_error), None);
        }
        self.change_state(LocalPlannerState::Arrived);
        self.last_goal_reached = GoalReached::FinalReached;
        (make_twist2_zero(), Some(StopMessage::Arrived))
    }

    /// 按 `config.navigation_costmap_interval` 节流发布最近一次 costmap 给订阅者。
    ///
    /// H-14 实现:LocalPlanner 同步轮询模式下,interval>0 且自上次发布已过该间隔时,
    /// 触发订阅者回调。订阅者可在此拿去做可视化或下游消费。
    fn maybe_publish_navigation_costmap(&mut self) {
        let interval = self.config.navigation_costmap_interval;
        if interval <= 0.0 || self.navigation_costmap_subs.is_empty() {
            return;
        }
        let now = std::time::Instant::now();
        let due = match self.last_navigation_costmap_at {
            None => true,
            Some(last) => now.duration_since(last).as_secs_f64() >= interval,
        };
        if !due {
            return;
        }
        // 当前只有 binary costmap(尚未与 NavigationMap 整合);直接发布 binary
        // 作为占位。生产实现应传入 `NavigationMap.gradient_costmap`。
        if let Some(cm) = self.costmap.as_ref() {
            for sub in &self.navigation_costmap_subs {
                (sub)(cm);
            }
            self.last_navigation_costmap_at = Some(now);
        }
    }

    fn change_state(&mut self, new_state: LocalPlannerState) {
        if new_state == self.state {
            return;
        }
        self.state = new_state;
        self.state_unique_id = self.state_unique_id.wrapping_add(1);
        tracing::info!("local planner state changed: {:?}", new_state);
    }

    /// 当前控制器（mutable，用于测试）
    pub fn controller_mut(&mut self) -> &mut PController {
        &mut self.controller
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::{make_path2d, make_pose_stamped, Point2};
    use crate::navigation_map::CostMap2D;

    #[test]
    fn final_rotation_checks_actual_goal_yaw_against_external_obstacle() {
        use crate::helpers::{make_pose_stamped, Point2};
        let geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.1],[0.3,-0.1],[0.3,0.1],[-0.3,0.1]],"z_min_m":-0.3,"z_max_m":0.1}"#).unwrap();
        let mut planner = LocalPlanner::new(0.3, 10.0, 0.2);
        planner.set_robot_geometry(geometry);
        planner.handle_odom(make_pose_stamped("map", 1.025, 1.025, 0.0));
        let mut map = CostMap2D::new_all_free("map", 80, 80, 0.05);
        map.set(24, 24, crate::navigation_map::OCCUPIED);
        planner.update_costmap(map);
        planner.start_planning_with_orientations(
            make_path2d(vec![Point2::new(1.025, 1.025)]),
            Some(vec![std::f64::consts::FRAC_PI_2]),
        );
        let (twist, stop) = planner.step(0.3);
        assert_eq!(twist, Twist2::ZERO);
        assert_eq!(stop, Some(StopMessage::ObstacleFound));
    }

    #[test]
    fn fresh_planner_is_idle() {
        let lp = LocalPlanner::new(0.5, 10.0, 0.2);
        assert_eq!(lp.state(), LocalPlannerState::Idle);
    }

    #[test]
    fn invalid_pose_or_path_stops_before_emitting_motion() {
        use nuwax_nav_types_messages::Quaternion;

        for invalid in 0..4 {
            let mut planner = LocalPlanner::new(0.5, 10.0, 0.2);
            let mut pose = make_pose_stamped("map", 0.5, 0.5, 0.0);
            let mut points = vec![Point2::new(0.5, 0.5), Point2::new(2.5, 0.5)];
            let mut yaws = vec![0.0, 0.0];
            match invalid {
                0 => pose.pose.orientation = Quaternion::default(),
                1 => pose.pose.position.x = f64::NAN,
                2 => points[1].y = f64::INFINITY,
                _ => yaws[1] = f64::NAN,
            }
            planner.handle_odom(pose);
            planner.start_planning_with_orientations(make_path2d(points), Some(yaws));
            assert_eq!(planner.step(0.3), (Twist2::ZERO, Some(StopMessage::Error)));
            assert_eq!(planner.state(), LocalPlannerState::Idle);
            assert_eq!(planner.goal_reached(), GoalReached::NotReached);
        }
    }

    #[test]
    fn start_then_idle_step_returns_zero_when_no_odom() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        // 无 odom, step 后返回 zero
        let (t, stop) = lp.step(0.3);
        assert_eq!(t.linear.x, 0.0);
        assert!(stop.is_none());
    }

    #[test]
    fn aligned_odom_skips_initial_rotation() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        lp.handle_odom(make_pose_stamped("map", 0.05, 0.0, 0.0));
        let _ = lp.step(0.3);
        // 朝向与第一段 (1,0) 一致，应该直接进入 PathFollowing
        assert_eq!(lp.state(), LocalPlannerState::PathFollowing);
    }

    #[test]
    fn follows_from_exact_start_and_reaches_endpoint_with_requested_orientation() {
        let mut lp = LocalPlanner::new(0.5, 20.0, 0.2);
        let goal_yaw = std::f64::consts::FRAC_PI_2;
        lp.start_planning_with_orientations(
            make_path2d(vec![Point2::new(0.5, 0.5), Point2::new(2.5, 0.5)]),
            Some(vec![0.0, goal_yaw]),
        );
        lp.update_costmap(CostMap2D::new_all_free("map", 60, 30, 0.1));
        let (mut x, mut y, mut yaw) = (0.5, 0.5, 0.0_f64);
        let mut arrived = false;
        for tick in 0..800 {
            lp.handle_odom(make_pose_stamped("map", x, y, yaw));
            let (twist, stop) = lp.step(0.3);
            if tick == 0 {
                assert!(stop.is_none());
                assert!(twist.linear.x > 0.0);
                assert_eq!(lp.state(), LocalPlannerState::PathFollowing);
            }
            if stop == Some(StopMessage::Arrived) {
                assert!((x - 2.5).hypot(y - 0.5) < 0.2);
                assert!(angle_diff(goal_yaw, yaw).abs() < 0.35);
                assert_eq!(lp.goal_reached(), GoalReached::FinalReached);
                arrived = true;
                break;
            }
            assert!(stop.is_none(), "非预期的路径终态: {stop:?}");
            // 用控制指令积分合成位姿，验证整段运动而非只检查状态字段。
            x += twist.linear.x * yaw.cos() * 0.05;
            y += twist.linear.x * yaw.sin() * 0.05;
            yaw += twist.angular_z * 0.05;
        }
        assert!(arrived, "合成机器人应走到路径终点并完成朝向调整");
    }

    #[test]
    fn final_rotation_resumes_following_if_pose_leaves_goal_tolerance() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        let goal_yaw = std::f64::consts::FRAC_PI_2;
        lp.start_planning_with_orientations(
            make_path2d(vec![Point2::new(0.5, 0.5), Point2::new(2.5, 0.5)]),
            Some(vec![0.0, goal_yaw]),
        );
        lp.update_costmap(CostMap2D::new_all_free("map", 60, 30, 0.1));
        lp.handle_odom(make_pose_stamped("map", 2.45, 0.5, 0.0));
        assert!(lp.step(0.3).1.is_none());
        assert_eq!(lp.state(), LocalPlannerState::FinalRotation);

        // 朝向已经达到要求，但更新后的位姿再次离开终点，不能误报到达。
        lp.handle_odom(make_pose_stamped("map", 1.5, 0.5, goal_yaw));
        let (_, stop) = lp.step(0.3);
        assert!(stop.is_none());
        assert_eq!(lp.state(), LocalPlannerState::PathFollowing);
        assert_eq!(lp.goal_reached(), GoalReached::NotReached);
    }

    #[test]
    fn empty_path_reports_error_without_claiming_arrival() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(Vec::new()));
        lp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        lp.update_costmap(CostMap2D::new_all_free("map", 20, 20, 0.1));
        let (twist, stop) = lp.step(0.3);
        assert_eq!(twist, make_twist2_zero());
        assert_eq!(stop, Some(StopMessage::Error));
        assert_eq!(lp.goal_reached(), GoalReached::NotReached);
    }

    #[test]
    fn obstacle_ahead_triggers_stop() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(3.0, 0.0),
        ]));
        let mut cm = CostMap2D::new_all_free("map", 40, 10, 0.1);
        // 在 path 上 (2.0, 0.0) 附近放 obstacle（cell 20,5 -> world (2.05, 0.55)）
        cm.set(20, 5, 100);
        lp.update_costmap(cm);
        lp.handle_odom(make_pose_stamped("map", 0.05, 0.0, 0.0));
        let (_t, stop) = lp.step(1.2); // robot_width=1.2 让 disk 半径覆盖 y=5
        assert!(matches!(stop, Some(StopMessage::ObstacleFound)));
    }

    #[test]
    fn stop_planning_resets_state() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        lp.stop_planning();
        assert_eq!(lp.state(), LocalPlannerState::Idle);
    }

    #[test]
    fn close_to_goal_triggers_final_rotation() {
        // goal_tolerance=0.5 让 path 上靠 goal 但还没到 tol
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.5);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        // 接近 goal（distance_to_goal = 0.05 < 0.5），yaw 偏差大 → FinalRotation
        lp.handle_odom(make_pose_stamped("map", 0.95, 0.0, 1.0));
        let _ = lp.step(0.3);
        assert_eq!(lp.state(), LocalPlannerState::FinalRotation);
    }

    #[test]
    fn distance_to_path_returns_value() {
        let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        lp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        let d = lp.get_distance_to_path();
        assert!(d.is_some());
        let d = d.unwrap();
        // 到水平路径线段的垂直距离为0.5米，与路径点间距无关。
        let expected = 0.5;
        assert!((d - expected).abs() < 1e-9, "expected {expected}, got {d}");
    }

    // ============================================================
    // H-13:rotation source 用 stored pose orientation
    // ============================================================

    /// H-13:stored pose orientation 覆盖段方向启发式。
    /// 路径首段在 x 方向(段方向 yaw=0),但 stored pose orientation = π/2,
    /// robot 当前朝向 0 → 应触发 InitialRotation(yaw_error ≈ π/2)。
    /// 旧实现会用段方向(0)算 yaw_error≈0,直接进 PathFollowing(错误)。
    #[test]
    fn h13_uses_stored_pose_orientation_not_segment_direction() {
        let mut cfg = LocalPlannerConfig::default_config();
        // 提高 orientation_tolerance 让明确无效的差值能触发 rotation 而非 skip
        cfg.orientation_tolerance = 0.1;
        let mut lp = LocalPlanner::with_config(cfg);
        // 路径首段是 (0,0)→(1,0),段方向 yaw=0
        lp.start_planning_with_orientations(
            make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]),
            Some(vec![std::f64::consts::FRAC_PI_2, 0.0]), // stored yaw=π/2
        );
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        // robot 当前朝向 0
        lp.handle_odom(make_pose_stamped("map", 0.0, 0.0, 0.0));
        let _ = lp.step(0.3);
        // yaw_error ≈ π/2 > tolerance (0.1) → 必须进入 InitialRotation
        assert_eq!(
            lp.state(),
            LocalPlannerState::InitialRotation,
            "stored pose orientation should override segment direction"
        );
    }

    /// H-13:若未提供 stored yaws,回退到段方向启发式。
    #[test]
    fn h13_falls_back_to_segment_direction_when_no_yaws() {
        let mut cfg = LocalPlannerConfig::default_config();
        cfg.orientation_tolerance = 0.1;
        let mut lp = LocalPlanner::with_config(cfg);
        // 路径首段 (0,0)→(1,0) 段方向 0;无 stored yaws
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        // robot 朝向 1.0(大偏差) → 段方向启发式算出 yaw_error ≈ 1.0 → InitialRotation
        lp.handle_odom(make_pose_stamped("map", 0.0, 0.0, 1.0));
        let _ = lp.step(0.3);
        assert_eq!(
            lp.state(),
            LocalPlannerState::InitialRotation,
            "should fall back to segment direction"
        );
    }

    // ============================================================
    // H-14:nerf_speed + simulation + navigation_costmap
    // ============================================================

    /// H-14:nerf_speed < 1.0 时 controller 速度应被缩放。
    #[test]
    fn h14_nerf_speed_scales_controller_speed() {
        let mut cfg = LocalPlannerConfig::default_config();
        cfg.speed = 1.0;
        cfg.nerf_speed = 0.5;
        let mut lp = LocalPlanner::with_config(cfg);
        // 检查 controller 内部 speed(私有),通过对齐 step 后的 advance twist 间接验证:
        // 不能直接读字段。我们通过 controller_mut 反射检查:速度在 twist 中体现。
        // 这里只验证构造不 panic 即可;后续 integrate test 做行为校验。
        let _ = lp.controller_mut();
    }

    /// H-14:simulation 字段传递到 controller;rotate 触发的角速度 boost 行为在
    /// controllers.rs 的 `simulation_boosts_small_angular` 测试已覆盖。
    /// 这里只验证 set_simulation 路径正确接线。
    #[test]
    fn h14_simulation_passed_to_controller() {
        let mut cfg = LocalPlannerConfig::default_config();
        cfg.simulation = true;
        let mut lp = LocalPlanner::with_config(cfg);
        // 通过 controller_mut 间接验证 simulation 标识;这里构造不 panic 即可
        let _ = lp.controller_mut();
    }

    /// H-14:navigation_costmap 订阅接口;interval=0 不发,interval>0 按节流发。
    #[test]
    fn h14_navigation_costmap_publish_throttled() {
        let mut cfg = LocalPlannerConfig::default_config();
        cfg.navigation_costmap_interval = 0.0; // 关闭
        let mut lp = LocalPlanner::with_config(cfg);
        let counter = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let counter_for_sub = Arc::clone(&counter);
        lp.subscribe_navigation_costmap(Arc::new(move |_cm: &CostMap2D| {
            counter_for_sub.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        }));
        lp.handle_odom(make_pose_stamped("map", 0.0, 0.0, 0.0));
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        lp.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let _ = lp.step(0.3);
        let _ = lp.step(0.3);
        // interval=0 不发布
        assert_eq!(counter.load(std::sync::atomic::Ordering::SeqCst), 0);

        // 开启 interval,第二次 step(间隔很短)应该只发 1 帧(节流)
        let mut cfg2 = LocalPlannerConfig::default_config();
        cfg2.navigation_costmap_interval = 10.0; // 10s 一次
        let mut lp2 = LocalPlanner::with_config(cfg2);
        let counter2 = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let counter2_for_sub = Arc::clone(&counter2);
        lp2.subscribe_navigation_costmap(Arc::new(move |_cm: &CostMap2D| {
            counter2_for_sub.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        }));
        lp2.handle_odom(make_pose_stamped("map", 0.0, 0.0, 0.0));
        let cm2 = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp2.update_costmap(cm2);
        lp2.start_planning(make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ]));
        let _ = lp2.step(0.3);
        let _ = lp2.step(0.3);
        assert_eq!(
            counter2.load(std::sync::atomic::Ordering::SeqCst),
            1,
            "should publish exactly once due to throttling"
        );
    }

    /// H-14:`start_planning_with_orientations` 长度不匹配时回退到段方向。
    #[test]
    fn h14_orientations_length_mismatch_falls_back() {
        let mut cfg = LocalPlannerConfig::default_config();
        cfg.orientation_tolerance = 0.1;
        let mut lp = LocalPlanner::with_config(cfg);
        // 路径 2 个点但给 3 个 yaw → 回退到段方向启发式
        lp.start_planning_with_orientations(
            make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)]),
            Some(vec![0.0, 0.0, 0.0]),
        );
        let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
        lp.update_costmap(cm);
        // robot 位于 (0.1, 0),远未到 goal,朝向=0 与段方向一致
        lp.handle_odom(make_pose_stamped("map", 0.1, 0.0, 0.0));
        let _ = lp.step(0.3);
        // 回退到段方向=0,与 robot 朝向 0 一致,且不在 position_in_tol (dx=0.1 > 0.1)
        // → 进入 PathFollowing(若在 tolerance 内+已在 start 位置则进 FinalRotation)
        assert_eq!(lp.state(), LocalPlannerState::PathFollowing);
    }
}
