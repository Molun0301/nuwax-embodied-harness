//! 规划器核心：`plan_on_costmap` 顶层 API 与 `SimplePlanner` 状态机。

use std::collections::HashSet;

use crate::astar::astar;
use crate::config::SimplePlannerConfig;
use crate::costmap::Costmap;
use crate::pointcloud::classify_points;
use crate::stuck::{progress_tick, StuckState};
use crate::waypoint::lookahead;
use crate::{NavError, Point2, Point3, PointStamped, Pose3, PoseStamped, Timestamp};
use nuwax_nav_types_messages::{NavFrameId, Quaternion};

/// 在 costmap 上做 A*，输出世界坐标路径。
///
/// - 有界占据图的端点及路径均须具备完整轮廓余量，旧地形输入保留端点解堵行为。
/// - `max_expansions = None` 用 200_000 默认值。
/// - `inflation_override = Some(r)` 用 `r` 替代 `costmap.inflation_radius()` 构造阻塞集合。
/// - 找不到路径返回 `Ok(None)`；costmap 错误（如 cell_size <= 0 已构造时拦住）返回 `Err`。
pub fn plan_on_costmap(
    costmap: &Costmap,
    start: Point2,
    goal: Point2,
    max_expansions: Option<usize>,
    inflation_override: Option<f64>,
) -> Result<Option<Vec<Point2>>, NavError> {
    plan_on_costmap_interruptible(
        costmap,
        start,
        goal,
        max_expansions,
        inflation_override,
        &|| false,
    )
}
fn plan_on_costmap_interruptible(
    costmap: &Costmap,
    start: Point2,
    goal: Point2,
    max_expansions: Option<usize>,
    inflation_override: Option<f64>,
    cancelled: &impl Fn() -> bool,
) -> Result<Option<Vec<Point2>>, NavError> {
    if !start.x.is_finite() || !start.y.is_finite() || !goal.x.is_finite() || !goal.y.is_finite() {
        return Err(NavError::InvalidInput {
            field: "start/goal",
            reason: "non-finite coordinate".into(),
        });
    }
    let requested_radius = inflation_override.unwrap_or_else(|| costmap.inflation_radius());
    if !requested_radius.is_finite() || requested_radius < 0.0 {
        return Err(NavError::InvalidInput {
            field: "inflation_radius",
            reason: "轮廓半径必须是有限非负数".into(),
        });
    }
    let effective_radius = costmap.effective_radius(requested_radius);
    let blocked: HashSet<(i64, i64)> = if costmap.has_occupancy() {
        // 有界输入直接检查候选圆盘，不对整片未知区构造稀疏障碍展开。
        HashSet::new()
    } else {
        costmap.blocked_at(effective_radius)
    };

    let start_cell = costmap.world_to_cell(start.x, start.y);
    let goal_cell = costmap.world_to_cell(goal.x, goal.y);

    if !costmap.permits_positions(start, goal, effective_radius) {
        return Ok(None);
    }

    let is_blocked = |ix: i64, iy: i64| -> bool {
        if cancelled() {
            return true;
        }
        if costmap.has_occupancy() {
            return !costmap.has_footprint_clearance(ix, iy, effective_radius);
        }
        if (ix, iy) == start_cell || (ix, iy) == goal_cell {
            return false;
        }
        blocked.contains(&(ix, iy))
    };

    let max_exp = max_expansions.unwrap_or(200_000);
    let path = astar(start_cell, goal_cell, is_blocked, max_exp);
    Ok(path.map(|cells| {
        let mut points: Vec<Point2> = cells
            .into_iter()
            .map(|(ix, iy)| {
                let (wx, wy) = costmap.cell_to_world(ix, iy);
                Point2 { x: wx, y: wy }
            })
            .collect();
        // 原点/分辨率来自实际占据图，格中心不能替代用户要求的精确目标。
        if costmap.has_occupancy() {
            if points.len() == 1 {
                points = vec![start, goal];
            } else {
                points[0] = start;
                *points.last_mut().expect("A*路径包含终点") = goal;
            }
        }
        points
    }))
}

/// SimplePlanner 状态机：不持有 transport，只暴露同步 API。
///
/// 业务线程 / event loop / tokio task 应在自己的 runtime 中驱动 `replan_once` 与
/// `update_waypoint`；本结构不启动线程（与 Python 原实现的 thread-per-loop 不同）。
#[derive(Debug, Clone)]
pub struct SimplePlanner {
    cancellation: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
    goal_options: nuwax_nav_types_messages::GoalOptions,
    cached_yaws: Option<Vec<f64>>,
    config: SimplePlannerConfig,
    costmap: Costmap,
    /// 机器人平面坐标
    robot_x: f64,
    robot_y: f64,
    robot_z: f64,
    robot_yaw: f64,
    has_odom: bool,
    /// 目标坐标
    goal_x: Option<f64>,
    goal_y: Option<f64>,
    goal_z: f64,
    goal_yaw: Option<f64>,
    /// Stuck 状态
    stuck: StuckState,
    /// 当前 effective inflation（stuck escalation 可缩小）
    effective_inflation: f64,
    /// A* 缓存路径
    cached_path: Option<Vec<Point2>>,
    /// 上次 A* 时间（monotonic 秒）
    last_plan_time: f64,
    /// 当前 waypoint
    current_wp: Option<Point2>,
}

impl SimplePlanner {
    /// 在私有规划实例查询完整路线，供行程预算复用生产规划与机体检查。
    pub fn query_route(self, from: Pose3, to: Pose3, options: nuwax_nav_types_messages::GoalOptions) -> Result<Vec<Point2>, NavError> {
        self.query_route_with_end_yaw(from, to, options).map(|(points, _)| points)
    }

    /// 查询同一完整机体路线，并保留实际末端朝向，供连续多段行程估计使用。
    /// 定向搜索保留末端 yaw；普通路线及已到达分支使用已通过完整机体检查的目标 yaw。
    pub fn query_route_with_end_yaw(mut self, from: Pose3, to: Pose3, options: nuwax_nav_types_messages::GoalOptions) -> Result<(Vec<Point2>, f64), NavError> {
        self.update_odometry(from);
        self.set_goal_options(options)?;
        self.set_goal(to.position, 0.0);
        self.set_goal_yaw(to.orientation.yaw());
        if self.goal_pose_reached() {
            let points=vec![Point2::new(from.position.x,from.position.y),Point2::new(to.position.x,to.position.y)];
            let view=if let Some(geometry)=self.config.robot_geometry.as_ref(){self.costmap.planning_view(points[0],from.orientation.yaw(),geometry)?}else{self.costmap.clone()};
            if view.path_has_observed_clearance(&points,from.orientation.yaw(),Some(to.orientation.yaw())) {return Ok((points, to.orientation.yaw()));}
        } else {
            self.replan_once(0.0);
            if let Some(path)=self.cached_path {
                // Oriented results start with the start pose and retain one yaw per point.
                // Other routes retain the goal yaw set above and validated by clearance.
                let end_yaw = match self.cached_yaws {
                    Some(yaws) => yaws.last().copied(),
                    None => self.goal_yaw,
                };
                if let Some(end_yaw) = end_yaw { return Ok((path, end_yaw)); }
            }
        }
        Err(NavError::InvalidInput {field:"route",reason:"完整机体路线不可达或计算预算耗尽".into()})
    }

    /// 构造。costmap 按 config 默认参数创建。
    /// # Errors
    /// 转发 `Costmap::new` 的 `InvalidInput`。
    pub fn new(config: SimplePlannerConfig) -> Result<Self, NavError> {
        let costmap = Costmap::new(
            config.cell_size,
            config.obstacle_height_threshold,
            config.inflation_radius,
        )?;
        Ok(Self {
            cancellation: None,
            goal_options: Default::default(),
            cached_yaws: None,
            effective_inflation: config.inflation_radius,
            stuck: StuckState {
                ref_goal_dist: f64::INFINITY,
                last_progress_time: 0.0,
                effective_inflation: config.inflation_radius,
            },
            config,
            costmap,
            robot_x: 0.0,
            robot_y: 0.0,
            robot_z: 0.0,
            robot_yaw: 0.0,
            has_odom: false,
            goal_x: None,
            goal_y: None,
            goal_z: 0.0,
            goal_yaw: None,
            cached_path: None,
            last_plan_time: 0.0,
            current_wp: None,
        })
    }

    /// 到达必须同时满足本目标的位置和朝向容差，供模块与规划核心共同判断。
    pub(crate) fn goal_pose_reached(&self) -> bool {
        self.has_odom
            && self.goal_x.zip(self.goal_y).is_some_and(|(x, y)| {
                (x - self.robot_x).hypot(y - self.robot_y) <= self.config.goal_reached_threshold
                    && self.goal_yaw.is_none_or(|yaw| {
                        (yaw - self.robot_yaw)
                            .sin()
                            .atan2((yaw - self.robot_yaw).cos())
                            .abs()
                            <= self.goal_options.yaw_tolerance_rad.max(1e-6)
                    })
            })
    }
    pub(crate) fn needs_replan(&self, now: f64) -> bool {
        self.has_odom
            && self.goal_x.is_some()
            && self.goal_y.is_some()
            && (self.cached_path.is_none()
                || now - self.last_plan_time >= self.config.replan_cooldown)
    }
    pub(crate) fn set_cancellation(
        &mut self,
        token: std::sync::Arc<std::sync::atomic::AtomicBool>,
    ) {
        self.cancellation = Some(token);
    }
    fn is_cancelled(&self) -> bool {
        self.cancellation
            .as_ref()
            .is_some_and(|token| token.load(std::sync::atomic::Ordering::Relaxed))
    }
    /// 只接纳工作线程的规划结果；地图、位姿和目标继续由输入拥有方持有。
    pub(crate) fn accept_plan(&mut self, computed: &Self) -> bool {
        self.cached_path = computed.cached_path.clone();
        self.cached_yaws = computed.cached_yaws.clone();
        if self.cached_path.is_some()
            && self.costmap.has_occupancy()
            && !self.cached_route_is_clear()
        {
            self.cached_path = None;
            self.cached_yaws = None;
            self.current_wp = None;
            return false;
        }
        self.last_plan_time = computed.last_plan_time;
        self.stuck = computed.stuck;
        true
    }

    /// 直接覆盖 costmap（接收 terrain_map_ext 输入后调用）。
    pub fn replace_costmap(&mut self, mut costmap: Costmap) {
        if costmap.has_occupancy() {
            costmap.preserve_inflation_floor(
                self.config
                    .inflation_radius
                    .max(self.costmap.minimum_inflation_radius()),
            );
            self.effective_inflation = costmap.inflation_radius();
            self.stuck.effective_inflation = self.effective_inflation;
        }
        self.costmap = costmap;
        // 新观测仍由执行者逐周期检查；只在剩余完整机体路线失效时撤销缓存。
        if !self.cached_route_is_clear() {
            self.cached_path = None;
            self.cached_yaws = None;
            self.current_wp = None;
        }
    }

    /// 保留同布局占据索引，当前路线仍按完整新图复核。
    pub fn update_occupancy(&mut self, grid: nuwax_nav_types_messages::OccupancyGrid2D) -> Result<crate::costmap::OccupancyUpdateStats, NavError> {
        let radius = self.config.inflation_radius.max(self.costmap.minimum_inflation_radius());
        let stats = self.costmap.replace_occupancy(grid, radius)?;
        self.effective_inflation = self.costmap.inflation_radius();
        self.stuck.effective_inflation = self.effective_inflation;
        if !self.cached_route_is_clear() {
            self.cached_path = None;
            self.cached_yaws = None;
            self.current_wp = None;
        }
        Ok(stats)
    }

    fn cached_route_is_clear(&self) -> bool {
        use nuwax_nav_control::PlanarPose;
        let (Some(path), Some(geometry)) = (&self.cached_path, &self.config.robot_geometry) else {
            return false;
        };
        if path.is_empty() || !self.has_odom {
            return false;
        }
        let derived_yaws: Vec<f64>;
        let yaws = if let Some(yaws) = self.cached_yaws.as_ref() {
            yaws
        } else {
            derived_yaws = path
                .iter()
                .enumerate()
                .map(|(i, point)| {
                    path.get(i + 1)
                        .map_or(self.goal_yaw.unwrap_or(self.robot_yaw), |next| {
                            (next.y - point.y).atan2(next.x - point.x)
                        })
                })
                .collect();
            &derived_yaws
        };
        if yaws.len() != path.len() {
            return false;
        }
        let current = Point2::new(self.robot_x, self.robot_y);
        let Ok(view) = self
            .costmap
            .planning_view(current, self.robot_yaw, geometry)
        else {
            return false;
        };
        let Some(grid) = view.occupancy_grid() else {
            return false;
        };
        let angle = |a: f64, b: f64| (a - b).sin().atan2((a - b).cos()).abs();
        let nearest = path
            .iter()
            .enumerate()
            .min_by(|(ia, a), (ib, b)| {
                a.distance(current)
                    .total_cmp(&b.distance(current))
                    .then_with(|| {
                        angle(yaws[*ia], self.robot_yaw)
                            .total_cmp(&angle(yaws[*ib], self.robot_yaw))
                    })
            })
            .map(|(i, _)| i)
            .unwrap_or(0);
        if path[nearest].distance(current) > self.config.lookahead_distance.max(0.3) {
            return false;
        }
        let mut from = PlanarPose {
            x: current.x,
            y: current.y,
            yaw: self.robot_yaw,
        };
        for i in nearest..path.len() {
            let to = PlanarPose {
                x: path[i].x,
                y: path[i].y,
                yaw: yaws[i],
            };
            if !crate::oriented::motion_is_clear(grid, geometry, from, to) {
                return false;
            }
            from = to;
        }
        true
    }

    /// 访问当前 costmap。
    #[must_use]
    pub fn costmap(&self) -> &Costmap {
        &self.costmap
    }

    /// 可变访问 costmap（用于 terrain_map 叠加）。
    pub fn costmap_mut(&mut self) -> &mut Costmap {
        &mut self.costmap
    }

    /// 叠入 fresh local terrain 点云。
    /// Returns whether the costmap changed.
    pub fn ingest_local_terrain(&mut self, points: &[Point3]) -> bool {
        classify_points(
            points,
            &mut self.costmap,
            self.robot_z - self.config.ground_offset_below_robot,
        )
    }

    /// 用 terrain_map_ext 重建 costmap。
    /// Returns whether the costmap changed.
    pub fn ingest_world_terrain(&mut self, points: &[Point3]) -> bool {
        let ground_z = if self.has_odom {
            self.robot_z - self.config.ground_offset_below_robot
        } else {
            0.0
        };
        let mut new_cm = Costmap::new(
            self.config.cell_size,
            self.config.obstacle_height_threshold,
            self.config.inflation_radius,
        )
        .expect("config-validated cell_size/inflation_radius");
        let _ = classify_points(points, &mut new_cm, ground_z);
        let dirty = !new_cm.heights().is_empty();
        self.costmap = new_cm;
        dirty
    }

    /// 捕获当前目标行为，旧目标选项不能在新请求中复用。
    pub fn set_goal_options(
        &mut self,
        options: nuwax_nav_types_messages::GoalOptions,
    ) -> Result<(), NavError> {
        options
            .validate()
            .map_err(|reason| NavError::InvalidInput {
                field: "goal_options",
                reason,
            })?;
        if options.allow_reverse
            && self
                .config
                .motion_limits
                .as_ref()
                .is_none_or(|limits| limits.max_reverse_mps <= 0.0)
        {
            return Err(NavError::InvalidInput {
                field: "allow_reverse",
                reason: "缺少倒车能力标定".into(),
            });
        }
        self.goal_options = options;
        self.config.goal_reached_threshold = options.position_tolerance_m.max(1e-6);
        Ok(())
    }
    /// 最近规划的真实机体朝向，和位置路径逐点对应。
    pub fn cached_yaws(&self) -> Option<&[f64]> {
        self.cached_yaws.as_deref()
    }

    /// 设置机器人位姿。
    pub fn update_odometry(&mut self, pose: Pose3) {
        self.robot_x = pose.position.x;
        self.robot_y = pose.position.y;
        self.robot_z = pose.position.z;
        self.robot_yaw = pose.orientation.yaw();
        self.has_odom = true;
    }

    /// 保留实际目标朝向，供发布路径前的模型扫掠核对。
    pub fn set_goal_yaw(&mut self, yaw: f64) {
        self.goal_yaw = Some(yaw);
    }

    /// 当前是否有 odom。
    #[must_use]
    pub fn has_odom(&self) -> bool {
        self.has_odom
    }

    /// 设置新 goal。恢复 effective_inflation、清空缓存。
    pub fn set_goal(&mut self, goal: Point3, now: f64) {
        if !goal.x.is_finite() || !goal.y.is_finite() || !goal.z.is_finite() {
            self.cancel_navigation();
            return;
        }
        self.goal_x = Some(goal.x);
        self.goal_y = Some(goal.y);
        self.goal_z = goal.z;
        self.goal_yaw = None;
        self.costmap
            .set_inflation_radius(self.config.inflation_radius);
        self.effective_inflation = self.costmap.inflation_radius();
        self.stuck = StuckState {
            ref_goal_dist: f64::INFINITY,
            last_progress_time: now,
            effective_inflation: self.effective_inflation,
        };
        self.cached_path = None;
        self.cached_yaws = None;
        self.last_plan_time = 0.0;
        self.current_wp = None;
    }

    /// 清空目标、缓存路径。原地返回 hold waypoint。
    pub fn cancel_navigation(&mut self) {
        let already_idle = self.goal_x.is_none() && self.goal_y.is_none();
        self.goal_x = None;
        self.goal_y = None;
        self.cached_path = None;
        self.cached_yaws = None;
        self.current_wp = None;
        if !already_idle {
            tracing::info!("Goal cleared — idle until new goal");
        }
    }

    /// 接收 stop_movement=true 取消。
    pub fn on_stop_movement(&mut self, stop: bool) {
        if stop {
            self.cancel_navigation();
        }
    }

    /// 当前 effective inflation（受 stuck escalation 影响）。
    #[must_use]
    pub fn effective_inflation(&self) -> f64 {
        self.effective_inflation
    }

    /// 缓存的最近一次规划路径。
    #[must_use]
    pub fn cached_path(&self) -> Option<&[Point2]> {
        self.cached_path.as_deref()
    }

    /// 当前 waypoint。
    #[must_use]
    pub fn current_waypoint(&self) -> Option<Point2> {
        self.current_wp
    }

    /// 重规划一次；返回 `(plan_path, did_replan, did_escalate)`。
    /// - `plan_path`: 成功时为新路径，失败或无 odom/goal 时为 `None`。
    /// - `did_replan`: true 表示确实跑了 A*；冷却内跳过则 false。
    /// - `did_escalate`: true 表示 effective_inflation 被缩小。
    pub fn replan_once(&mut self, mono_now: f64) -> (Option<Option<Vec<Point2>>>, bool, bool) {
        if !self.has_odom || self.goal_x.is_none() || self.goal_y.is_none() {
            return (None, false, false);
        }
        let rx = self.robot_x;
        let ry = self.robot_y;
        let gx = self.goal_x.expect("checked above");
        let gy = self.goal_y.expect("checked above");
        let planning_view = if let Some(geometry) = self.config.robot_geometry.as_ref() {
            match self
                .costmap
                .planning_view(Point2::new(rx, ry), self.robot_yaw, geometry)
            {
                Ok(view) => Some(view),
                Err(error) => {
                    tracing::warn!(%error, "机器人规划外形不可用");
                    self.cached_path = None;
                    self.cached_yaws = None;
                    self.current_wp = None;
                    return (Some(None), true, false);
                }
            }
        } else {
            None
        };
        let goal_dist = ((gx - rx).powi(2) + (gy - ry).powi(2)).sqrt();
        if self.goal_pose_reached() {
            let costmap = planning_view.as_ref().unwrap_or(&self.costmap);
            if !costmap.permits_positions(
                Point2::new(rx, ry),
                Point2::new(gx, gy),
                self.effective_inflation,
            ) {
                self.cached_path = None;
                self.cached_yaws = None;
                self.current_wp = None;
                return (Some(None), true, false);
            }
            self.cancel_navigation();
            return (None, false, false);
        }

        // 冷却内跳过 A*，但仍跑 stuck escalation。
        let cooldown_active = self.cached_path.is_some()
            && mono_now - self.last_plan_time < self.config.replan_cooldown;
        if cooldown_active {
            return (None, false, false);
        }

        // Stuck escalation（注意：即便 cooldown 跳过，仍允许升级由下一次 replan 触发；
        //  与 Python 行为一致：cooldown 内只走缓存 waypoint，不触发 escalation）。
        let (new_stuck, escalated) = progress_tick(
            self.stuck,
            goal_dist,
            mono_now,
            self.config.progress_epsilon,
            self.config.stuck_seconds,
            self.config.stuck_shrink_factor,
            self.config
                .stuck_min_inflation
                .max(self.costmap.minimum_inflation_radius()),
        );
        self.stuck = new_stuck;
        if escalated {
            tracing::warn!(
                stuck_seconds = self.config.stuck_seconds,
                goal_dist = goal_dist,
                inflation_from = self.effective_inflation,
                inflation_to = new_stuck.effective_inflation,
                "Stuck — shrinking inflation"
            );
            self.effective_inflation = new_stuck.effective_inflation;
            self.costmap
                .set_inflation_radius(new_stuck.effective_inflation);
        }

        let mut path = plan_on_costmap_interruptible(
            planning_view.as_ref().unwrap_or(&self.costmap),
            Point2 { x: rx, y: ry },
            Point2 { x: gx, y: gy },
            Some(self.config.max_expansions),
            Some(self.effective_inflation),
            &|| self.is_cancelled(),
        );
        if self.is_cancelled() {
            return (None, false, false);
        }
        let mut planned_yaws = None;
        if let Some(geometry) = self.config.robot_geometry.as_ref() {
            let view = planning_view.as_ref().unwrap_or(&self.costmap);
            let needs_oriented = self.goal_options.allow_reverse
                || self.config.motion_limits.as_ref().is_some_and(|limits| {
                    limits.model != nuwax_nav_types_messages::MotionModel::Differential
                        || !limits.can_rotate_in_place
                })
                || !matches!(&path,Ok(Some(points)) if view.path_has_observed_clearance(points,self.robot_yaw,self.goal_yaw));
            if needs_oriented {
                if let Some(grid) = view.occupancy_grid() {
                    match crate::oriented::plan_oriented_interruptible(
                        grid,
                        geometry,
                        nuwax_nav_control::PlanarPose {
                            x: rx,
                            y: ry,
                            yaw: self.robot_yaw,
                        },
                        nuwax_nav_control::PlanarPose {
                            x: gx,
                            y: gy,
                            yaw: self.goal_yaw.unwrap_or(self.robot_yaw),
                        },
                        self.goal_options,
                        self.config.motion_limits.as_ref(),
                        self.config.max_expansions,
                        &|| self.is_cancelled(),
                    ) {
                        Ok(Some(poses)) => {
                            planned_yaws = Some(poses.iter().map(|p| p.yaw).collect());
                            path = Ok(Some(poses.iter().map(|p| Point2::new(p.x, p.y)).collect()));
                        }
                        Ok(None) => path = Ok(None),
                        Err(error) => path = Err(error),
                    }
                }
            }
        }
        if self.is_cancelled() {
            return (None, false, false);
        }
        self.last_plan_time = mono_now;
        // 只有本次规划成功才能继续发布路线；失败时旧路线已经失效。
        self.cached_path = None;
        self.cached_yaws = None;
        self.current_wp = None;
        match path {
            Ok(Some(p)) => {
                if planned_yaws.is_none()
                    && !planning_view
                        .as_ref()
                        .unwrap_or(&self.costmap)
                        .path_has_observed_clearance(&p, self.robot_yaw, self.goal_yaw)
                {
                    tracing::warn!("实际路径朝向会扫过已观测障碍，拒绝发布该路线");
                    return (Some(None), true, escalated);
                }
                self.cached_yaws = planned_yaws;
                self.cached_path = Some(p.clone());
                (Some(Some(p)), true, escalated)
            }
            Ok(None) => {
                tracing::warn!(
                    robot_x = rx,
                    robot_y = ry,
                    goal_x = gx,
                    goal_y = gy,
                    "A* failed; holding position"
                );
                (Some(None), true, escalated)
            }
            Err(_e) => (None, true, escalated),
        }
    }

    /// 计算并发布当前 waypoint（沿 cached path 应用 lookahead）。
    /// Returns `Some(point)` if a waypoint was emitted; `None` otherwise.
    pub fn update_waypoint(&mut self) -> Option<PointStamped> {
        let cached = self.cached_path.clone()?;
        // waypoint::lookahead 接受 `&[Point2]`；传 `cached.as_slice()` 适配 Vec -> slice
        let (wx, wy) = lookahead(
            cached.as_slice(),
            self.robot_x,
            self.robot_y,
            self.config.lookahead_distance,
        );
        let last = cached.last().copied().unwrap_or(Point2 { x: wx, y: wy });
        let _is_goal = (wx - last.x).abs() < 1e-9 && (wy - last.y).abs() < 1e-9;
        self.current_wp = Some(Point2 { x: wx, y: wy });
        Some(PointStamped {
            frame_id: NavFrameId::from_static(&self.config.world_frame),
            stamp: Timestamp::default(),
            point: Point3 {
                x: wx,
                y: wy,
                z: self.goal_z,
            },
        })
    }

    /// 把 cached path 转成 `Path` 消息。
    #[must_use]
    pub fn build_path_message(&self) -> Option<PoseStamped> {
        // 单一 hold pose 路径：起点为 robot 位姿
        let _ = self.cached_path.as_ref()?;
        Some(PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            pose: Pose3 {
                position: Point3 {
                    x: self.robot_x,
                    y: self.robot_y,
                    z: self.robot_z,
                },
                orientation: Quaternion::from_yaw(self.robot_yaw),
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_query_preserves_actual_final_yaw_and_legacy_points() {
        use nuwax_nav_types_messages::{GoalOptions, MotionLimits, MotionModel, OccupancyCell, OccupancyGrid2D, RobotGeometry};
        let make_planner = |can_rotate_in_place| {
            let config = SimplePlannerConfig {
                robot_geometry: Some(RobotGeometry {
                    self_collision: None,
                    footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                    z_min_m: -0.1, z_max_m: 0.1, source: "route-end-test".into(),
                }),
                motion_limits: Some(MotionLimits {
                    model: MotionModel::Differential,
                    max_forward_mps: 0.5, max_reverse_mps: 0.2, max_lateral_mps: 0.0, max_angular_rps: 1.0,
                    linear_acceleration_mps2: 1.0, guaranteed_braking_mps2: 1.0,
                    angular_acceleration_rps2: 1.0, guaranteed_angular_braking_rps2: 1.0,
                    reaction_time_sec: 0.1, minimum_turning_radius_m: 0.0, can_rotate_in_place,
                }),
                ..Default::default()
            };
            let map = Costmap::from_occupancy(OccupancyGrid2D::filled(
                Point2::new(0.0, 0.0), 0.1, 40, 40, OccupancyCell::Free), config.inflation_radius).unwrap();
            let mut planner = SimplePlanner::new(config).unwrap();
            planner.replace_costmap(map);
            planner
        };
        let from = Pose3 { position: Point3::new(1.05, 1.05, 0.0), orientation: Quaternion::IDENTITY };
        // Reached branch validates and retains its actual two-point segment to target.
        let near = Pose3 { position: Point3::new(1.15, 1.05, 0.0), orientation: Quaternion::from_yaw(0.1) };
        let (points, yaw) = make_planner(true).query_route_with_end_yaw(from, near, GoalOptions::default()).unwrap();
        assert_eq!(points.last(), Some(&Point2::new(1.15, 1.05)));
        assert_eq!(yaw, near.orientation.yaw());
        let target = Pose3 { position: Point3::new(2.05, 1.05, 0.0), orientation: Quaternion::from_yaw(1.0) };
        // Ordinary clearance-validated route ends at the requested yaw.
        let (points, yaw) = make_planner(true).query_route_with_end_yaw(from, target, GoalOptions::default()).unwrap();
        assert_eq!(yaw, target.orientation.yaw());
        assert_eq!(make_planner(true).query_route(from, target, GoalOptions::default()).unwrap(), points);
        // Force oriented search; its orientation-free direct connection ends facing forward,
        // not at the nominal target yaw that query_route previously discarded.
        let options = GoalOptions { yaw_tolerance_rad: std::f64::consts::PI, ..Default::default() };
        let (points, yaw) = make_planner(false).query_route_with_end_yaw(from, target, options).unwrap();
        assert_eq!(points.last(), Some(&Point2::new(2.05, 1.05)));
        assert!(yaw.abs() < 1e-12);
        assert_ne!(yaw, target.orientation.yaw());
        assert_eq!(make_planner(false).query_route(from, target, options).unwrap(), points);
    }

    #[test]
    fn body_view_starts_from_self_returns_but_keeps_surrounding_obstacles() {
        use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D, RobotGeometry};
        let geometry = RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.1],[0.3,-0.1],[0.3,0.1],[-0.3,0.1]],"z_min_m":-0.3,"z_max_m":0.1}"#).unwrap();
        let start = Point2::new(0.025, 0.025);
        let goal = Point2::new(1.025, 0.025);
        for value in [OccupancyCell::Unknown, OccupancyCell::Occupied] {
            let mut grid =
                OccupancyGrid2D::filled(Point2::new(-2.0, -2.0), 0.05, 80, 80, OccupancyCell::Free);
            let footprint = geometry.grid_footprint(0.05, 0.0, 80, 80).unwrap();
            for &(dx, dy) in &footprint.offsets {
                grid.set((40 + dx) as u32, (40 + dy) as u32, value);
            }
            let map = Costmap::from_occupancy(grid.clone(), 0.2).unwrap();
            let view = map.planning_view(start, 0.0, &geometry).unwrap();
            assert!(plan_on_costmap(&view, start, goal, None, None)
                .unwrap()
                .is_some());
            assert!(!map.is_known_free(40, 40));
            let moved = map
                .planning_view(Point2::new(0.825, 0.025), 0.0, &geometry)
                .unwrap();
            assert!(
                !moved.is_known_free(40, 40),
                "当前位置移动后不能继续清除旧位置"
            );
            for y in 0..80 {
                grid.set(52, y, OccupancyCell::Occupied);
            }
            let wall = Costmap::from_occupancy(grid, 0.2)
                .unwrap()
                .planning_view(start, 0.0, &geometry)
                .unwrap();
            assert!(plan_on_costmap(&wall, start, goal, None, None)
                .unwrap()
                .is_none());
        }
    }

    #[test]
    fn bounded_planner_keeps_footprint_clear_of_unknown_corners() {
        use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
        for resolution in [0.1, 0.2] {
            let mut grid = OccupancyGrid2D::filled(
                Point2::new(0.0, 0.0),
                resolution,
                20,
                20,
                OccupancyCell::Free,
            );
            for y in 10..20 {
                for x in 10..20 {
                    grid.set(x, y, OccupancyCell::Unknown);
                }
            }
            let start = grid.cell_center(5, 14).unwrap();
            let goal = grid.cell_center(14, 5).unwrap();
            let map = Costmap::from_occupancy(grid.clone(), 0.2).unwrap();
            let path = plan_on_costmap(&map, start, goal, None, None)
                .unwrap()
                .unwrap();
            let radius = (0.2 / resolution).ceil() as i64;
            for point in path {
                let (x, y) = map.world_to_cell(point.x, point.y);
                for dx in -radius..=radius {
                    for dy in -radius..=radius {
                        if dx * dx + dy * dy <= radius * radius {
                            assert!(
                                map.is_known_free(x + dx, y + dy),
                                "resolution={resolution} 路径点{point:?}的轮廓触及未知凹角"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn bounded_endpoints_require_ceil_disk_even_with_smaller_override() {
        use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
        for blocked in [OccupancyCell::Unknown, OccupancyCell::Occupied] {
            let mut grid =
                OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 20, 20, OccupancyCell::Free);
            grid.set(10, 10, blocked);
            let start = grid.cell_center(5, 5).unwrap();
            let goal = grid.cell_center(8, 10).unwrap();
            let mut map = Costmap::from_occupancy(grid, 0.15).unwrap();
            map.set_inflation_radius(0.01);
            assert_eq!(map.inflation_radius(), 0.15);
            assert!(
                plan_on_costmap(&map, start, goal, None, Some(0.0))
                    .unwrap()
                    .is_none(),
                "0.15米在0.1米图上按2格圆盘检查，不能通过显式覆盖缩小"
            );
            let edge = Point2::new(0.15, 0.55);
            assert!(
                plan_on_costmap(&map, edge, edge, None, None)
                    .unwrap()
                    .is_none(),
                "同格终点也不能绕过图外轮廓检查"
            );
        }
    }

    #[test]
    fn bounded_stuck_and_map_updates_preserve_footprint_floor() {
        use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 40, 30, OccupancyCell::Free);
        for y in 0..grid.height {
            grid.set(20, y, OccupancyCell::Occupied);
        }
        let mut planner = SimplePlanner::new(SimplePlannerConfig::default()).unwrap();
        planner.update_odometry(Pose3 {
            position: Point3::new(0.55, 1.05, 0.0),
            ..Pose3::default()
        });
        planner.replace_costmap(Costmap::from_occupancy(grid.clone(), 0.25).unwrap());
        planner.set_goal(Point3::new(3.05, 1.05, 0.0), 0.0);
        for now in [0.0, 6.0, 12.0] {
            let (path, _, escalated) = planner.replan_once(now);
            assert!(matches!(path, Some(None)));
            assert!(!escalated);
            assert_eq!(planner.effective_inflation(), 0.25);
        }
        planner.replace_costmap(Costmap::from_occupancy(grid, 0.05).unwrap());
        planner.set_goal(Point3::new(3.05, 1.05, 0.0), 13.0);
        assert_eq!(planner.effective_inflation(), 0.25);
        assert_eq!(planner.costmap().inflation_radius(), 0.25);

        // 没有有界占据图的旧地形输入仍保留原有额外膨胀降级行为。
        let mut terrain = SimplePlanner::new(SimplePlannerConfig::default()).unwrap();
        terrain.update_odometry(Pose3::default());
        terrain.set_goal(Point3::new(3.0, 0.0, 0.0), 0.0);
        terrain.replan_once(0.0);
        assert!(terrain.replan_once(6.0).2);
        assert!(terrain.effective_inflation() < 0.2);
    }

    #[test]
    fn failed_replan_stops_old_waypoints_and_recovers_after_obstacle_removal() {
        let mut planner = SimplePlanner::new(SimplePlannerConfig {
            cell_size: 1.0,
            inflation_radius: 0.0,
            max_expansions: 100,
            replan_cooldown: 1.0,
            ..SimplePlannerConfig::default()
        })
        .unwrap();
        planner.update_odometry(Pose3 {
            position: Point3 {
                x: 0.5,
                y: 0.5,
                z: 0.0,
            },
            orientation: Quaternion::IDENTITY,
        });
        planner.set_goal(
            Point3 {
                x: 5.5,
                y: 0.5,
                z: 0.0,
            },
            0.0,
        );
        assert!(matches!(planner.replan_once(0.0), (Some(Some(_)), true, _)));
        assert!(planner.update_waypoint().is_some());

        for x in -1..=1 {
            for y in -1..=1 {
                if (x, y) != (0, 0) {
                    planner
                        .costmap_mut()
                        .update(x as f64 + 0.5, y as f64 + 0.5, 1.0);
                }
            }
        }
        assert!(matches!(planner.replan_once(2.0), (Some(None), true, _)));
        assert!(planner.cached_path().is_none());
        assert!(planner.current_waypoint().is_none());
        assert!(planner.update_waypoint().is_none());

        planner.costmap_mut().clear();
        assert!(matches!(planner.replan_once(2.1), (Some(Some(_)), true, _)));
        assert!(planner.update_waypoint().is_some());
    }
}
