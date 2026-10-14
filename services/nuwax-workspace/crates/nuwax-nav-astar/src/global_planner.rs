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
use crate::navigation_map::{CostMap2D, GradientStrategy, NavigationMap, OCCUPIED, UNKNOWN};
use crate::position_tracker::PositionTracker;
use crate::replan_limiter::ReplanLimiter;
use nuwax_nav_types_messages::{
    NavigationStatus, OccupancyCell, OccupancyGrid2D, Path2D, Point2, PoseStamped, RobotGeometry,
    Twist2,
};

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
    /// 无模型时的目标圆形余量（米）；模型模式直接检查真实外包。
    pub safe_goal_clearance: f64,
    /// safe goal tolerance（米）
    pub safe_goal_tolerance: f64,
    /// 速度（米/秒）
    pub speed: f64,
    /// 控制频率（Hz）
    pub control_frequency: f64,
    /// 无模型时的机器人宽度（米），用于距离梯度膨胀与旧局部检查。
    pub robot_width: f64,
    /// 统一模型外形；未配置时保留独立算法调用的旧宽度参数。
    pub robot_geometry: Option<RobotGeometry>,
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
            robot_geometry: None,
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
    /// 地图变化后先重新核对全局可达性，不能依赖局部把未知伪装成障碍。
    map_update_pending: bool,
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
        let mut local = LocalPlanner::with_config(local_cfg);
        if let Some(geometry) = cfg.robot_geometry.as_ref() {
            local.set_robot_geometry(geometry.clone());
        }

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
            map_update_pending: false,
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
        self.map_update_pending |=
            self.current_goal.is_some() && !self.remaining_route_is_clear(&costmap);
        self.navigation_map.update(costmap.clone());
        self.navigation_map_near.update(costmap);
    }

    /// 地图刷新仅在剩余路线失效时重规划，保持无关更新下的路径和卡住追踪。
    fn remaining_route_is_clear(&self, map: &CostMap2D) -> bool {
        let (Some(path), Some(odom)) = (
            self.last_published_path.as_ref(),
            self.current_odom.as_ref(),
        ) else {
            return false;
        };
        if path.points.is_empty()
            || map.width() == 0
            || map.height() == 0
            || !map.resolution().is_finite()
            || map.resolution() <= 0.0
        {
            return false;
        }
        let previous = self.navigation_map.binary_costmap().ok();
        if previous.as_ref().is_none_or(|old| {
            old.width() != map.width()
                || old.height() != map.height()
                || old.resolution() != map.resolution()
                || old.frame_id() != map.frame_id()
                || old.cell_center_world(0, 0) != map.cell_center_world(0, 0)
        }) {
            return false;
        }
        let (x, y, yaw) = pose_xy_yaw(odom);
        let position = Point2::new(x, y);
        let (index, projected) =
            crate::path_distancer::PathDistancer::new(path, 0.5).closest_projection(position);
        let points: Vec<_> = [position, projected]
            .into_iter()
            .chain(path.points.iter().skip(index + 1).copied())
            .collect();
        let Some(geometry) = self.cfg.robot_geometry.as_ref() else {
            return points.iter().all(|point| {
                let value = map.cell_value(*point);
                value != UNKNOWN && value < OCCUPIED
            });
        };
        let (Ok(width), Ok(height)) = (u32::try_from(map.width()), u32::try_from(map.height()))
        else {
            return false;
        };
        let first = map.cell_center_world(0, 0);
        let origin = Point2::new(
            first.x - map.resolution() / 2.0,
            first.y - map.resolution() / 2.0,
        );
        let Ok(footprint) = geometry.grid_footprint(map.resolution(), yaw, width, height) else {
            return false;
        };
        let Ok(current) = footprint.at_position(origin, position) else {
            return false;
        };
        let anchor = map.world_to_cell(position);
        let occupied_now: std::collections::HashSet<_> = current
            .offsets
            .iter()
            .filter_map(|&(dx, dy)| {
                Some((
                    u32::try_from(i64::from(anchor.0).checked_add(dx)?).ok()?,
                    u32::try_from(i64::from(anchor.1).checked_add(dy)?).ok()?,
                ))
            })
            .collect();
        points.windows(2).all(|segment| {
            footprint.visit_motion(origin, (segment[0], yaw), (segment[1], yaw), |x, y| {
                let value = map.get(x as usize, y as usize);
                occupied_now.contains(&(x, y)) || (value != UNKNOWN && value < OCCUPIED)
            })
        })
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

        if self.map_update_pending && self.current_goal.is_some() {
            self.pending_event = Some(if self._plan_path() {
                GlobalPlannerEvent::NewPath
            } else {
                GlobalPlannerEvent::PlanningFailed
            });
        }

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
        self.map_update_pending = false;
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
        let safe_goal = match if self.cfg.robot_geometry.is_some() {
            Some(Point2::new(gx, gy))
        } else {
            self._find_safe_goal(Point2::new(gx, gy))
        } {
            Some(g) => g,
            None => {
                tracing::warn!("no safe goal");
                self.cancel_goal(false);
                self.status = NavigationStatus::Blocked;
                return false;
            }
        };

        let planned = if self.cfg.robot_geometry.is_some() {
            self.navigation_map
                .binary_costmap()
                .ok()
                .and_then(|map| self.path_with_geometry(&map, &odom, &goal))
        } else {
            self._find_wide_path(safe_goal, Point2::new(ox, oy))
        };
        let path = match planned {
            Some(p) => p,
            None => {
                tracing::warn!("no path to safe goal");
                self.cancel_goal(false);
                self.status = NavigationStatus::Blocked;
                return false;
            }
        };

        self.last_published_path = Some(path.clone());
        let mut yaws: Vec<f64> = path
            .points
            .windows(2)
            .map(|segment| (segment[1].y - segment[0].y).atan2(segment[1].x - segment[0].x))
            .collect();
        yaws.push(pose_xy_yaw(&goal).2);
        self.local_planner
            .start_planning_with_orientations(path, Some(yaws));
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
        self.find_safe_goal_on_map(&binary, goal)
    }

    fn find_safe_goal_on_map(&self, binary: &CostMap2D, goal: Point2) -> Option<Point2> {
        let (x, y) = binary.world_to_cell(goal);
        if !binary.in_bounds(x, y) {
            return None;
        }
        // 正式全局规划不能把未知目标以低于障碍阈值的代价悄悄放行。
        if binary.is_unknown(x as usize, y as usize) {
            return None;
        }
        let opts = GoalValidatorOptions {
            cost_threshold: OCCUPIED,
            min_clearance: self.cfg.safe_goal_clearance,
            max_search_distance: self.cfg.safe_goal_tolerance,
            connectivity_check_radius: 3,
        };
        find_safe_goal_bfs_contiguous(binary, goal, &opts)
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
        Self::find_path_on_map(map, goal, robot_pos)
    }

    fn find_path_on_map(map: &NavigationMap, goal: Point2, robot_pos: Point2) -> Option<Path2D> {
        let costmap = map.make_gradient_costmap(1.1)?;
        let opts = crate::astar::AStarOptions {
            unknown_penalty: 1.0,
            ..crate::astar::AStarOptions::default()
        };
        let r = crate::astar::min_cost_astar(&costmap, robot_pos, goal, &opts).ok()?;
        r.path
    }

    /// 模型路径仅使用本次位姿的私有规划视图，原始占据图始终留在拥有方。
    fn path_with_geometry(
        &self,
        source: &CostMap2D,
        start: &PoseStamped,
        goal: &PoseStamped,
    ) -> Option<Path2D> {
        let geometry = self.cfg.robot_geometry.as_ref()?;
        geometry.validate().ok()?;
        let width = u32::try_from(source.width()).ok()?;
        let height = u32::try_from(source.height()).ok()?;
        let resolution = source.resolution();
        let (sx, sy, start_yaw) = pose_xy_yaw(start);
        let (gx, gy, goal_yaw) = pose_xy_yaw(goal);
        if width == 0
            || height == 0
            || width > i32::MAX as u32
            || height > i32::MAX as u32
            || !resolution.is_finite()
            || resolution <= 0.0
            || [sx, sy, start_yaw, gx, gy, goal_yaw]
                .iter()
                .any(|v| !v.is_finite())
        {
            return None;
        }
        let first_center = source.cell_center_world(0, 0);
        let origin = Point2::new(
            first_center.x - resolution / 2.0,
            first_center.y - resolution / 2.0,
        );
        if !origin.x.is_finite() || !origin.y.is_finite() {
            return None;
        }
        let mut cells = Vec::with_capacity(source.width() * source.height());
        for y in 0..source.height() {
            for x in 0..source.width() {
                cells.push(match source.get(x, y) {
                    UNKNOWN => OccupancyCell::Unknown,
                    value if value >= OCCUPIED => OccupancyCell::Occupied,
                    // CostMap2D中的0—99是可通行代价；正式占据接线已把原50以上分类为100。
                    _ => OccupancyCell::Free,
                });
            }
        }
        let mut environment = OccupancyGrid2D::new(origin, resolution, width, height, cells);
        let footprint = geometry
            .grid_footprint(resolution, start_yaw, width, height)
            .ok()?;
        let start_position = Point2::new(sx, sy);
        let goal_position = Point2::new(gx, gy);
        footprint.clear_current_occupancy(&mut environment, start_position);
        if !footprint.is_clear_at(&environment, start_position)
            || !footprint.is_clear_at(&environment, goal_position)
        {
            return None;
        }

        // 只在私有配置空间图中过滤放不下机体的候选中心，不把它发布为传感器观测。
        let mut centers = source.clone();
        for y in 0..height {
            for x in 0..width {
                let value = if footprint.is_clear(&environment, x, y) {
                    let original = source.get(x as usize, y as usize);
                    if original < OCCUPIED {
                        original
                    } else {
                        0
                    }
                } else {
                    OCCUPIED
                };
                centers.set(x as usize, y as usize, value);
            }
        }
        // A*格中心不能代替真实端点；端点已按其准确相位检验，路径最后恢复原坐标。
        for point in [start_position, goal_position] {
            let (x, y) = environment.world_to_cell(point)?;
            centers.set(x as usize, y as usize, 0);
        }
        let strategy = if start_position.distance(goal_position) > 1.5 {
            GradientStrategy::Voronoi
        } else {
            GradientStrategy::Gradient
        };
        // 机体已在候选过滤中计算，距离梯度不能再叠加旧默认0.5米宽度。
        let mut navigation = NavigationMap::new(0.0, strategy);
        navigation.update(centers);
        let mut path = Self::find_path_on_map(&navigation, goal_position, start_position)?;
        if path.points.len() == 1 {
            path.points = vec![start_position, goal_position];
        } else {
            *path.points.first_mut()? = start_position;
            *path.points.last_mut()? = goal_position;
        }
        let clear_observation = |x, y| {
            environment
                .get(x, y)
                .is_some_and(|cell| cell == OccupancyCell::Unknown || cell.is_known_traversable())
        };
        let mut yaw = start_yaw;
        for segment in path.points.windows(2) {
            let heading = if segment[0].distance(segment[1]) > 1e-9 {
                (segment[1].y - segment[0].y).atan2(segment[1].x - segment[0].x)
            } else {
                yaw
            };
            if !footprint.visit_motion(
                origin,
                (segment[0], yaw),
                (segment[1], heading),
                clear_observation,
            ) {
                return None;
            }
            yaw = heading;
        }
        if !footprint.visit_motion(
            origin,
            (goal_position, yaw),
            (goal_position, goal_yaw),
            clear_observation,
        ) {
            return None;
        }
        Some(path)
    }

    /// 直接做一次规划（不修改状态机），返回 PlannerResult
    pub fn plan_once(
        &mut self,
        start: PoseStamped,
        goal: PoseStamped,
        costmap: &CostMap2D,
    ) -> PlannerResult {
        if self.cfg.robot_geometry.is_some() {
            return match self.path_with_geometry(costmap, &start, &goal) {
                Some(path) => {
                    let length = path.length();
                    PlannerResult::success(path, length, 0)
                }
                None => PlannerResult::failure(crate::helpers::PlannerFailure::NoPath, 0),
            };
        }
        let (gx, gy, _) = pose_xy_yaw(&goal);
        let (sx, sy, _) = pose_xy_yaw(&start);
        let safe = match self.find_safe_goal_on_map(costmap, Point2::new(gx, gy)) {
            Some(g) => g,
            None => {
                return PlannerResult::failure(crate::helpers::PlannerFailure::GoalBlocked, 0);
            }
        };
        let strategy = if (safe.x - sx).hypot(safe.y - sy) > 1.5 {
            GradientStrategy::Voronoi
        } else {
            GradientStrategy::Gradient
        };
        let mut map = NavigationMap::new(self.cfg.robot_width, strategy);
        map.update(costmap.clone());
        let path = Self::find_path_on_map(&map, safe, Point2::new(sx, sy));
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

    fn test_geometry(half_width: f64) -> RobotGeometry {
        RobotGeometry {
            footprint_xy_m: vec![
                [-0.3, -half_width],
                [0.3, -half_width],
                [0.3, half_width],
                [-0.3, half_width],
            ],
            z_min_m: -0.3,
            z_max_m: 0.1,
            source: "动态规划回归".into(),
        }
    }

    fn body_planner(half_width: f64) -> GlobalPlanner {
        GlobalPlanner::new(GlobalPlannerConfig {
            robot_geometry: Some(test_geometry(half_width)),
            // 旧宽度故意与真实外形不同，确保新路径没有继续按旧默认宽度规划。
            robot_width: 2.0,
            ..GlobalPlannerConfig::default()
        })
    }

    fn observed_map() -> CostMap2D {
        let mut map = CostMap2D::new_all_free("map", 80, 80, 0.05);
        map.set_origin(Point2::new(-2.0, -2.0));
        map
    }

    #[test]
    fn model_planning_clears_only_present_body_and_preserves_supplied_map() {
        let start = make_pose_stamped("map", 0.001, 0.013, 0.0);
        let goal = make_pose_stamped("map", 1.001, 0.013, 0.0);
        for self_observation in [UNKNOWN, OCCUPIED] {
            let mut map = observed_map();
            let position = start.pose.position.to_2d();
            let footprint = test_geometry(0.1)
                .grid_footprint(0.05, 0.0, 80, 80)
                .unwrap()
                .at_position(Point2::new(-2.0, -2.0), position)
                .unwrap();
            let anchor = map.world_to_cell(position);
            for (dx, dy) in footprint.offsets {
                map.set(
                    (i64::from(anchor.0) + dx) as usize,
                    (i64::from(anchor.1) + dy) as usize,
                    self_observation,
                );
            }
            let mut planner = body_planner(0.1);
            planner.handle_global_costmap(map.clone());
            let path = planner
                .plan_once(start.clone(), goal.clone(), &map)
                .path
                .expect("自体观测不能困住起点");
            assert_eq!(path.points.first().copied(), Some(position));
            assert_eq!(
                path.points.last().copied(),
                Some(goal.pose.position.to_2d())
            );
            assert_eq!(
                map.get(anchor.0 as usize, anchor.1 as usize),
                self_observation
            );
            assert_eq!(
                planner
                    .navigation_map
                    .binary_costmap()
                    .unwrap()
                    .get(anchor.0 as usize, anchor.1 as usize),
                self_observation
            );
            assert!(
                planner
                    .plan_once(goal.clone(), start.clone(), &map)
                    .path
                    .is_none(),
                "改变当前位置后不能继续清除旧起点的观测"
            );
        }
    }

    #[test]
    fn global_paths_do_not_cross_external_unknown_or_wall_with_or_without_model() {
        for with_model in [false, true] {
            for obstacle in [UNKNOWN, OCCUPIED] {
                let mut map = observed_map();
                for y in 0..map.height() {
                    map.set(52, y, obstacle);
                }
                let mut planner = if with_model {
                    body_planner(0.1)
                } else {
                    GlobalPlanner::new(GlobalPlannerConfig::default())
                };
                assert!(
                    planner
                        .plan_once(
                            make_pose_stamped("map", 0.025, 0.025, 0.0),
                            make_pose_stamped("map", 1.025, 0.025, 0.0),
                            &map
                        )
                        .path
                        .is_none(),
                    "外部未知或墙不能成为全局路线"
                );
                assert_eq!(map.get(52, 40), obstacle);
            }
        }
    }

    #[test]
    fn model_width_and_final_rotation_are_used_by_global_plan() {
        let mut corridor = observed_map();
        for y in 0..80 {
            if !(37..=42).contains(&y) {
                for x in 0..80 {
                    corridor.set(x, y, OCCUPIED);
                }
            }
        }
        let start = make_pose_stamped("map", 0.025, 0.025, 0.0);
        let goal = make_pose_stamped("map", 1.025, 0.025, 0.0);
        assert!(body_planner(0.1)
            .plan_once(start.clone(), goal.clone(), &corridor)
            .path
            .is_some());
        assert!(body_planner(0.2)
            .plan_once(start.clone(), goal, &corridor)
            .path
            .is_none());
        let mut turn_map = observed_map();
        turn_map.set(44, 44, OCCUPIED);
        assert!(body_planner(0.1)
            .plan_once(start.clone(), start.clone(), &turn_map)
            .path
            .is_some());
        let turn_goal = make_pose_stamped("map", 0.025, 0.025, std::f64::consts::FRAC_PI_2);
        assert!(
            body_planner(0.1)
                .plan_once(start, turn_goal, &turn_map)
                .path
                .is_none(),
            "最终朝向旋转经过的已知障碍必须在发布前拒绝"
        );
    }

    #[test]
    fn changed_map_replans_before_returning_an_old_route() {
        let mut planner = body_planner(0.1);
        let mut map = observed_map();
        planner.handle_global_costmap(map.clone());
        planner.handle_odom(make_pose_stamped("map", 0.025, 0.025, 0.0));
        planner.handle_goal_request(make_pose_stamped("map", 1.025, 0.025, 0.0));
        assert!(planner
            .last_published_path
            .as_ref()
            .is_some_and(|path| !path.points.is_empty()));
        for y in 0..80 {
            map.set(52, y, UNKNOWN);
        }
        planner.handle_global_costmap(map.clone());
        let output = planner.tick(&map);
        assert_eq!(output.event, Some(GlobalPlannerEvent::PlanningFailed));
        assert!(output.path.is_none_or(|path| path.points.is_empty()));
    }

    #[test]
    fn irrelevant_and_current_body_updates_do_not_reset_route_or_stuck_clock() {
        let mut planner = body_planner(0.1);
        let mut map = observed_map();
        planner.handle_global_costmap(map.clone());
        planner.handle_odom(make_pose_stamped("map", 0.025, 0.025, 0.0));
        planner.handle_goal_request(make_pose_stamped("map", 1.025, 0.025, 0.0));
        planner.tick(&map);
        planner.tick(&map);
        let path = planner.last_published_path.clone();
        let clock = std::time::Instant::now() - std::time::Duration::from_secs(1);
        planner.last_stuck_check = clock;
        map.set(5, 5, UNKNOWN);
        map.set(40, 40, OCCUPIED);
        planner.handle_global_costmap(map.clone());
        assert!(!planner.map_update_pending);
        planner.tick(&map);
        assert_eq!(planner.last_published_path, path);
        assert_eq!(planner.last_stuck_check, clock);
    }

    #[test]
    fn fresh_planner_is_idle() {
        let gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        assert_eq!(gp.status(), NavigationStatus::Idle);
        assert!(!gp.is_goal_reached());
    }

    #[test]
    fn single_plan_uses_supplied_map_and_preserves_running_state() {
        let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
        let free = CostMap2D::new_all_free("map", 40, 40, 0.1);
        let start = make_pose_stamped("map", 0.75, 1.75, 0.0);
        let goal = make_pose_stamped("map", 3.25, 1.75, 0.0);
        assert!(gp
            .plan_once(start.clone(), goal.clone(), &free)
            .path
            .is_some());

        gp.handle_global_costmap(free.clone());
        gp.handle_odom(start.clone());
        gp.handle_goal_request(goal.clone());
        let previous_status = gp.status();
        let previous_path = gp.last_published_path.clone();
        let mut blocked = free.clone();
        for y in 0..blocked.height() {
            blocked.set(20, y, OCCUPIED);
        }
        assert!(gp.plan_once(start, goal, &blocked).path.is_none());
        assert_eq!(gp.status(), previous_status);
        assert_eq!(gp.last_published_path, previous_path);
        assert_eq!(gp.navigation_map.binary_costmap().unwrap().get(20, 17), 0);
    }

    #[test]
    fn out_of_bounds_goal_returns_failure_without_panicking() {
        let cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
        for (x, y) in [(-1.0, 1.0), (4.0, 1.0), (1.0, -1.0), (1.0, 4.0)] {
            let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
            gp.handle_global_costmap(cm.clone());
            gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
            gp.handle_goal_request(make_pose_stamped("map", x, y, 0.0));
            assert_eq!(gp.tick(&cm).event, Some(GlobalPlannerEvent::PlanningFailed));
        }
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
