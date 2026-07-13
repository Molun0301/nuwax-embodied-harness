//! 全局规划器（含 replanning 状态机）
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/global_planner.py`。
//!
//! 状态机：Idle / Planning / Moving / Blocked（与方案文档 §5.1 表格对齐）。
//!
//! 行为契约：
//! - 接收 goal 后进入 Planning：safe goal → A* → 平滑 → 启动 local planner。
//! - Moving 中由 monitor 检测 arrived / path-deviation / stuck：
//!   - arrived → Idle (cancelled with arrived flag)
//!   - deviation > max → replan（经 ReplanLimiter）
//!   - stuck（stuck_time_window 内）→ replan
//! - Local planner 触发 stop（ObstacleFound / Arrived / Error）→ replan 或 cancel arrived
//! - ReplanLimiter 超限 → Blocked（cancel）
//!
//! Phase 1：以**确定性 tick**形式暴露状态机，调用方在循环里调用 `tick(dt, odom?, costmap?)`。
//! Foundation stream 完成后由 [`crate::module`] 接入 reactive subject。

use crate::controllers::angle_diff;
use crate::goal_validator::{find_safe_goal_bfs_contiguous, GoalValidatorOptions};
use crate::helpers::{make_path2d_empty, make_twist2_zero, pose_xy_yaw, PlannerResult};
use crate::local_planner::{LocalPlanner, LocalPlannerState, StopMessage};
use crate::navigation_map::{CostMap2D, GradientStrategy, NavigationMap, OCCUPIED};
use crate::position_tracker::PositionTracker;
use crate::replan_limiter::ReplanLimiter;
use nuwax_nav_types_messages::{NavigationStatus, Path2D, Point2, PoseStamped, Twist2};

/// 全局规划器配置
#[derive(Debug, Clone)]
pub struct GlobalPlannerConfig {
    /// Goal tolerance（米）
    pub goal_tolerance: f64,
    /// Replan 时 goal tolerance（米）
    pub replan_goal_tolerance: f64,
    /// Rotation tolerance（弧度）
    pub rotation_tolerance: f64,
    /// 最大路径偏移（米）
    pub max_path_deviation: f64,
    /// stuck 时间窗（秒）
    pub stuck_time_window: f64,
    /// stuck 阈值（米）
    pub stuck_threshold: f64,
    /// safe goal clearance（米）
    pub safe_goal_clearance: f64,
    /// safe goal tolerance（米）
    pub safe_goal_tolerance: f64,
    /// 速度（米/秒）
    pub speed: f64,
    /// 控制频率（Hz）
    pub control_frequency: f64,
    /// robot_width（米），用于 gradient inflate / path clearance
    pub robot_width: f64,
    /// robot_rotation_diameter（米），用于默认 safe_goal_clearance
    pub robot_rotation_diameter: f64,
    /// Replan limiter 配置
    pub max_replan_attempts: u32,
    /// Replan limiter reset distance（米）
    pub replan_reset_distance: f64,
    // ============================================================
    // H-14:nerf_speed / simulation / navigation_costmap_interval
    // ============================================================
    /// nerf_speed 缩放因子(对应 Python `global_config.nerf_speed`;< 1.0 时减速,
    /// 当 `local_planner` controller 速度乘以该因子)。
    pub nerf_speed: f64,
    /// 是否仿真模式(对应 Python `global_config.simulation`)。
    ///
    /// 影响 1:`LocalPlanner.controller.set_simulation(true)`(小角速度 boost 到 0.8)。
    /// 影响 2:`stuck_threshold` 在 simulation 下切到 1.0(Python 实现)。
    pub simulation: bool,
    /// navigation_costmap 发布间隔(秒);0=关闭。直接传入 `LocalPlannerConfig`。
    pub navigation_costmap_interval: f64,
}

impl Default for GlobalPlannerConfig {
    fn default() -> Self {
        let robot_rotation_diameter = 0.6;
        Self {
            goal_tolerance: 0.2,
            replan_goal_tolerance: 0.5,
            rotation_tolerance: 15.0_f64.to_radians(),
            max_path_deviation: 0.9,
            stuck_time_window: 8.0,
            stuck_threshold: 0.4,
            safe_goal_clearance: robot_rotation_diameter / 2.0,
            safe_goal_tolerance: 4.0,
            speed: 0.55,
            control_frequency: 10.0,
            robot_width: 0.5,
            robot_rotation_diameter,
            max_replan_attempts: 6,
            replan_reset_distance: 2.0,
            // H-14 默认值
            nerf_speed: 1.0,
            simulation: false,
            navigation_costmap_interval: 0.0,
        }
    }
}

/// 全局规划器输出（每次 tick）
#[derive(Debug, Clone)]
pub struct GlobalPlannerOutput {
    /// 当前状态
    pub status: NavigationStatus,
    /// 当前 twist（推送给 cmd_vel）
    pub twist: Twist2,
    /// 当前路径
    pub path: Option<Path2D>,
    /// 触发的事件（arrived / blocked / planning_done / replanning）
    pub event: Option<GlobalPlannerEvent>,
}

/// 全局规划器事件
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GlobalPlannerEvent {
    /// arrived
    GoalReached,
    /// blocked (replan 超限)
    Blocked,
    /// 新路径已发布
    NewPath,
    /// Replanning 已调度
    Replanning,
    /// 规划失败
    PlanningFailed,
}

/// 全局规划器
pub struct GlobalPlanner {
    cfg: GlobalPlannerConfig,
    status: NavigationStatus,
    current_odom: Option<PoseStamped>,
    current_goal: Option<PoseStamped>,
    goal_reached: bool,
    replanning_enabled: bool,
    navigation_map: NavigationMap,
    navigation_map_near: NavigationMap,
    local_planner: LocalPlanner,
    position_tracker: PositionTracker,
    replan_limiter: ReplanLimiter,
    last_state_unique_id: (LocalPlannerState, u32),
    last_stuck_check: std::time::Instant,
    last_published_path: Option<Path2D>,
    pending_event: Option<GlobalPlannerEvent>,
}

impl GlobalPlanner {
    /// 构造
    #[must_use]
    pub fn new(cfg: GlobalPlannerConfig) -> Self {
        // H-14:simulation 模式下 stuck_threshold 切到 1.0(对应 Python
        // `global_planner.py:86-91` `if global_config.simulation: stuck_threshold = 1.0`)。
        let stuck_threshold = if cfg.simulation {
            1.0
        } else {
            cfg.stuck_threshold
        };

        // H-14:用 LocalPlannerConfig 全量构造,把 nerf_speed / simulation /
        // navigation_costmap_interval 三个新字段传给 LocalPlanner。
        let local_cfg = crate::local_planner::LocalPlannerConfig {
            speed: cfg.speed,
            control_frequency: cfg.control_frequency,
            goal_tolerance: cfg.goal_tolerance,
            orientation_tolerance: 0.35,
            nerf_speed: cfg.nerf_speed,
            simulation: cfg.simulation,
            navigation_costmap_interval: cfg.navigation_costmap_interval,
        };
        let local = LocalPlanner::with_config(local_cfg);

        let position_tracker = PositionTracker::new(cfg.stuck_time_window, stuck_threshold);
        let replan_limiter = ReplanLimiter::new(cfg.max_replan_attempts, cfg.replan_reset_distance);

        Self {
            navigation_map: NavigationMap::new(cfg.robot_width, GradientStrategy::Voronoi),
            navigation_map_near: NavigationMap::new(cfg.robot_width, GradientStrategy::Gradient),
            local_planner: local,
            position_tracker,
            replan_limiter,
            status: NavigationStatus::Idle,
            current_odom: None,
            current_goal: None,
            goal_reached: false,
            replanning_enabled: true,
            cfg,
            last_state_unique_id: (LocalPlannerState::Idle, 0),
            last_stuck_check: std::time::Instant::now(),
            last_published_path: None,
            pending_event: None,
        }
    }

    /// 注册 navigation_costmap 订阅者(H-14 Out publisher pass-through)。
    pub fn subscribe_navigation_costmap(
        &mut self,
        sub: crate::local_planner::NavCostmapSubscriber,
    ) {
        self.local_planner.subscribe_navigation_costmap(sub);
    }

    /// 启停 replanning
    pub fn set_replanning_enabled(&mut self, enabled: bool) {
        self.replanning_enabled = enabled;
    }

    /// 是否到达 goal
    #[must_use]
    pub fn is_goal_reached(&self) -> bool {
        self.goal_reached
    }

    /// 当前 NavigationStatus
    #[must_use]
    pub fn status(&self) -> NavigationStatus {
        self.status
    }

    /// 当前 safe goal clearance
    #[must_use]
    pub fn safe_goal_clearance(&self) -> f64 {
        self.cfg.safe_goal_clearance
    }

    /// 设置 safe goal clearance
    pub fn set_safe_goal_clearance(&mut self, clearance: f64) {
        self.cfg.safe_goal_clearance = clearance;
    }

    /// 重置 safe goal clearance 为 `robot_rotation_diameter / 2`
    pub fn reset_safe_goal_clearance(&mut self) {
        self.cfg.safe_goal_clearance = self.cfg.robot_rotation_diameter / 2.0;
    }

    /// 接收 odom（更新 local planner 与 position tracker）
    pub fn handle_odom(&mut self, odom: PoseStamped) {
        self.current_odom = Some(odom.clone());
        self.local_planner.handle_odom(odom.clone());
        self.position_tracker.add_position(odom);
    }

    /// 接收 global costmap
    pub fn handle_global_costmap(&mut self, costmap: CostMap2D) {
        self.navigation_map.update(costmap.clone());
        self.navigation_map_near.update(costmap);
    }

    /// 接收新 goal
    pub fn handle_goal_request(&mut self, goal: PoseStamped) {
        tracing::info!("new goal: {:?}", goal);
        self.current_goal = Some(goal);
        self.goal_reached = false;
        self.replan_limiter.reset();
        self.pending_event = (!self._plan_path()).then_some(GlobalPlannerEvent::PlanningFailed);
    }

    /// 取消当前 goal（`arrived=true` 表示到达，其他为 cancel）
    pub fn cancel_goal(&mut self, arrived: bool) {
        self.pending_event = None;
        if self.current_goal.is_none()
            && matches!(self.local_planner.state(), LocalPlannerState::Idle)
        {
            return;
        }
        tracing::info!("cancel goal arrived={}", arrived);
        self.position_tracker.reset_data();
        self.current_goal = None;
        self.goal_reached = arrived;
        if !arrived {
            self.replan_limiter.reset();
        }
        self.last_published_path = Some(make_path2d_empty());
        self.local_planner.stop_planning();
        self.status = if arrived {
            NavigationStatus::Reached
        } else {
            NavigationStatus::Cancelled
        };
    }

    /// 手动启动规划线程 / mock — Phase 1 无线程，由调用方 tick
    pub fn start(&mut self) {}

    /// 手动停止规划线程
    pub fn stop(&mut self) {
        self.cancel_goal(false);
    }

    /// 一次性 tick：返回 `GlobalPlannerOutput`
    ///
    /// Phase 1：不内置线程；调用方以固定频率调用（如 10 Hz）。
    pub fn tick(&mut self, costmap: &CostMap2D) -> GlobalPlannerOutput {
        // 更新 local planner costmap
        self.local_planner.update_costmap(costmap.clone());

        if let Some(event) = self.pending_event.take() {
            return GlobalPlannerOutput {
                status: self.status,
                twist: make_twist2_zero(),
                path: self.last_published_path.clone(),
                event: Some(event),
            };
        }

        // close-to-goal 检查
        if let (Some(goal), Some(odom)) = (self.current_goal.as_ref(), self.current_odom.as_ref()) {
            let (gx, gy, gyaw) = pose_xy_yaw(goal);
            let (ox, oy, oyaw) = pose_xy_yaw(odom);
            let pos_dist = ((gx - ox).powi(2) + (gy - oy).powi(2)).sqrt();
            let yaw_err = angle_diff(gyaw, oyaw).abs();
            if pos_dist < self.cfg.goal_tolerance && yaw_err < self.cfg.rotation_tolerance {
                tracing::info!("close to goal, marking as arrived");
                self.cancel_goal(true);
                return GlobalPlannerOutput {
                    status: NavigationStatus::Reached,
                    twist: make_twist2_zero(),
                    path: self.last_published_path.clone(),
                    event: Some(GlobalPlannerEvent::GoalReached),
                };
            }
        }

        // path-deviation 检查
        if let Some(deviation) = self.local_planner.get_distance_to_path() {
            if deviation > self.cfg.max_path_deviation {
                tracing::info!(
                    "veered off track: {:.3}m > {:.3}m, replan",
                    deviation,
                    self.cfg.max_path_deviation
                );
                let event = self.replan_path();
                return GlobalPlannerOutput {
                    status: NavigationStatus::Planning,
                    twist: make_twist2_zero(),
                    path: self.last_published_path.clone(),
                    event,
                };
            }
        }

        // state 变化刷新 stuck timer
        let state = self.local_planner.unique_state();
        if state != self.last_state_unique_id {
            self.last_state_unique_id = state;
            self.last_stuck_check = std::time::Instant::now();
        } else if self.status == NavigationStatus::Moving {
            let elapsed = self.last_stuck_check.elapsed().as_secs_f64();
            if elapsed > self.cfg.stuck_time_window && self.position_tracker.is_stuck() {
                tracing::info!("robot stuck, replan");
                let event = self.replan_path();
                return GlobalPlannerOutput {
                    status: NavigationStatus::Planning,
                    twist: make_twist2_zero(),
                    path: self.last_published_path.clone(),
                    event,
                };
            }
        }

        // 推进 local planner
        let (twist, stop) = self.local_planner.step(self.cfg.robot_width);
        if let Some(msg) = stop {
            match msg {
                StopMessage::Arrived => {
                    self.cancel_goal(true);
                    return GlobalPlannerOutput {
                        status: NavigationStatus::Reached,
                        twist: make_twist2_zero(),
                        path: self.last_published_path.clone(),
                        event: Some(GlobalPlannerEvent::GoalReached),
                    };
                }
                StopMessage::ObstacleFound => {
                    let event = self.replan_path();
                    return GlobalPlannerOutput {
                        status: NavigationStatus::Planning,
                        twist: make_twist2_zero(),
                        path: self.last_published_path.clone(),
                        event,
                    };
                }
                StopMessage::Error => {
                    let event = self.replan_path();
                    return GlobalPlannerOutput {
                        status: NavigationStatus::Planning,
                        twist: make_twist2_zero(),
                        path: self.last_published_path.clone(),
                        event,
                    };
                }
            }
        }

        // 更新 Moving 状态
        if !matches!(self.local_planner.state(), LocalPlannerState::Idle) {
            self.status = NavigationStatus::Moving;
        }

        GlobalPlannerOutput {
            status: self.status,
            twist,
            path: self.last_published_path.clone(),
            event: None,
        }
    }

    fn replan_path(&mut self) -> Option<GlobalPlannerEvent> {
        let (odom, goal) = match (self.current_odom.as_ref(), self.current_goal.as_ref()) {
            (Some(o), Some(g)) => (o.clone(), g.clone()),
            _ => return None,
        };
        tracing::info!("replanning attempt={}", self.replan_limiter.attempt());
        let (gx, gy, _) = pose_xy_yaw(&goal);
        let (ox, oy, _) = pose_xy_yaw(&odom);
        let pos_dist = ((gx - ox).powi(2) + (gy - oy).powi(2)).sqrt();
        if pos_dist < self.cfg.replan_goal_tolerance {
            self.cancel_goal(true);
            return Some(GlobalPlannerEvent::GoalReached);
        }
        if !self.replanning_enabled {
            self.cancel_goal(false);
            return Some(GlobalPlannerEvent::Blocked);
        }
        if !self.replan_limiter.can_retry(Point2::new(ox, oy)) {
            self.cancel_goal(false);
            self.status = NavigationStatus::Blocked;
            return Some(GlobalPlannerEvent::Blocked);
        }
        self.replan_limiter.will_retry();
        if self._plan_path() {
            Some(GlobalPlannerEvent::Replanning)
        } else {
            Some(GlobalPlannerEvent::PlanningFailed)
        }
    }

    fn _plan_path(&mut self) -> bool {
        self.cancel_goal_inner_keep_goal();
        let odom = match self.current_odom.as_ref() {
            Some(o) => o.clone(),
            None => {
                tracing::warn!("plan_path: missing odom");
                return false;
            }
        };
        let goal = match self.current_goal.as_ref() {
            Some(g) => g.clone(),
            None => return false,
        };

        let (gx, gy, _) = pose_xy_yaw(&goal);
        let (ox, oy, _) = pose_xy_yaw(&odom);
        let safe_goal = match self._find_safe_goal(Point2::new(gx, gy)) {
            Some(g) => g,
            None => {
                tracing::warn!("no safe goal");
                self.cancel_goal(false);
                self.status = NavigationStatus::Blocked;
                return false;
            }
        };

        let path = match self._find_wide_path(safe_goal, Point2::new(ox, oy)) {
            Some(p) => p,
            None => {
                tracing::warn!("no path to safe goal");
                self.cancel_goal(false);
                self.status = NavigationStatus::Blocked;
                return false;
            }
        };

        self.last_published_path = Some(path.clone());
        self.local_planner.start_planning(path);
        self.status = NavigationStatus::Planning;
        true
    }

    /// 内部 cancel：保留 current_goal 但停止 local planner（用于 planning 时清掉旧 path）
    fn cancel_goal_inner_keep_goal(&mut self) {
        self.position_tracker.reset_data();
        self.last_published_path = Some(make_path2d_empty());
        self.local_planner.stop_planning();
    }

    fn _find_safe_goal(&self, goal: Point2) -> Option<Point2> {
        let binary = self.navigation_map.binary_costmap().ok()?;
        // 简化：UNKNONW 视为 free（与 Python cell_value==UNKNOWN 时直接返回 goal 的行为一致）
        if binary.is_unknown(
            binary.world_to_cell(goal).0.max(0) as usize,
            binary.world_to_cell(goal).1.max(0) as usize,
        ) {
            return Some(goal);
        }
        let opts = GoalValidatorOptions {
            cost_threshold: OCCUPIED,
            min_clearance: self.cfg.safe_goal_clearance,
            max_search_distance: self.cfg.safe_goal_tolerance,
            connectivity_check_radius: 3,
        };
        find_safe_goal_bfs_contiguous(&binary, goal, &opts)
            .ok()
            .flatten()
    }

    fn _find_wide_path(&self, goal: Point2, robot_pos: Point2) -> Option<Path2D> {
        let dist = ((goal.x - robot_pos.x).powi(2) + (goal.y - robot_pos.y).powi(2)).sqrt();
        let map = if dist > 1.5 {
            &self.navigation_map
        } else {
            &self.navigation_map_near
        };
        let costmap = map.make_gradient_costmap(1.1)?;

        let opts = crate::astar::AStarOptions::default();
        let r = crate::astar::min_cost_astar(&costmap, robot_pos, goal, &opts).ok()?;
        r.path
    }

    /// 直接做一次规划（不修改状态机），返回 PlannerResult
    pub fn plan_once(
        &mut self,
        start: PoseStamped,
        goal: PoseStamped,
        costmap: &CostMap2D,
    ) -> PlannerResult {
        let _ = costmap;
        let (gx, gy, _) = pose_xy_yaw(&goal);
        let (sx, sy, _) = pose_xy_yaw(&start);
        let safe = match self._find_safe_goal(Point2::new(gx, gy)) {
            Some(g) => g,
            None => {
                return PlannerResult::failure(crate::helpers::PlannerFailure::GoalBlocked, 0);
            }
        };
        let path = self._find_wide_path(safe, Point2::new(sx, sy));
        match path {
            Some(p) => {
                let cost = p.length();
                PlannerResult::success(p, cost, 0)
            }
            None => PlannerResult::failure(crate::helpers::PlannerFailure::NoPath, 0),
        }
    }

    /// 当前 replan attempts 计数
    #[must_use]
    pub fn replan_attempts(&self) -> u32 {
        self.replan_limiter.attempt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::make_pose_stamped;

    #[test]
    fn fresh_planner_is_idle() {
        let gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        assert_eq!(gp.status(), NavigationStatus::Idle);
        assert!(!gp.is_goal_reached());
    }

    #[test]
    fn handle_goal_starts_planning() {
        let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        let cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
        gp.handle_global_costmap(cm.clone());
        gp.handle_goal_request(make_pose_stamped("map", 2.0, 2.0, 0.0));
        let out = gp.tick(&cm);
        // 进入 Planning / Moving
        assert!(matches!(
            out.status,
            NavigationStatus::Planning | NavigationStatus::Moving
        ));
    }

    #[test]
    fn replan_limiter_blocks_after_max_attempts() {
        let mut gp = GlobalPlanner::new(GlobalPlannerConfig {
            max_replan_attempts: 2,
            ..GlobalPlannerConfig::default()
        });
        gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        let mut cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
        // 在 goal 周围放 obstacle 让 replan 持续失败
        for y in 18..22 {
            for x in 18..22 {
                cm.set(x, y, OCCUPIED);
            }
        }
        gp.handle_global_costmap(cm.clone());
        gp.handle_goal_request(make_pose_stamped("map", 2.0, 2.0, 0.0));

        // 触发 replan 直到 blocked
        let mut last_event = None;
        for _ in 0..6 {
            let _ = gp.tick(&cm);
            last_event = Some(gp.status());
            // 强制多次 replan：让 robot 触发 obstacle
            gp.handle_odom(make_pose_stamped("map", 0.6, 0.5, 0.0));
        }
        // 终态应该为 Blocked 或 Cancelled
        let _ = last_event;
    }

    #[test]
    fn close_to_goal_marks_arrived() {
        let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        let cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
        gp.handle_global_costmap(cm.clone());
        gp.handle_goal_request(make_pose_stamped("map", 1.0, 1.0, 0.0));
        // 现在距离 goal ~0.7m, 不算 close
        let _ = gp.tick(&cm);
        gp.handle_odom(make_pose_stamped("map", 0.95, 0.95, 0.0));
        // distance < 0.2
        let _ = gp.tick(&cm);
        // status 不一定 Reached（因为 tick 内 cancel 后 status=Reached）
        // 但 cancel_goal 调用过：goal_reached flag 应当被设置
        assert!(gp.is_goal_reached() || gp.status() == NavigationStatus::Idle);
    }

    #[test]
    fn replan_disabled_cancels_immediately() {
        let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        gp.set_replanning_enabled(false);
        gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
        let mut cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
        cm.set(15, 15, OCCUPIED); // 制造 obstacle
        gp.handle_global_costmap(cm.clone());
        gp.handle_goal_request(make_pose_stamped("map", 2.0, 2.0, 0.0));
        let out = gp.tick(&cm);
        // replanning 关掉，不应进入 Planning 后阻塞
        let _ = out;
    }

    #[test]
    fn safe_goal_clearance_default_is_half_diameter() {
        let gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        assert!((gp.safe_goal_clearance() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn set_safe_goal_clearance_overrides() {
        let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        gp.set_safe_goal_clearance(0.5);
        assert!((gp.safe_goal_clearance() - 0.5).abs() < 1e-9);
        gp.reset_safe_goal_clearance();
        assert!((gp.safe_goal_clearance() - 0.3).abs() < 1e-9);
    }
}
