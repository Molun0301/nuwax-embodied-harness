//! 原生 SCAN 执行适配。位姿与实测运动独立输入，命令回执不充当测速。
use crate::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceSnapshot};
use nuwax_nav_astar::{local_planner::LocalPlannerConfig, navigation_map::CostMap2D};
use nuwax_nav_types_messages::{
    GoalOptions, MotionKinematics, MotionLimits, Path2D, Path3D, Point2, PoseStamped, Twist2,
};
use std::sync::Arc;

/// 跟随器与全局路线消费共用的位置容差口径。
pub fn position_within_tolerance(position: [f64; 3], goal: [f64; 3], tolerance: f64, step_height: f64) -> bool {
    if tolerance == 0. {
        nuwax_nav_types_messages::same_finite_position(position, goal)
    } else {
        (position[0] - goal[0]).hypot(position[1] - goal[1]) < tolerance
            && (position[2] - goal[2]).abs() <= step_height.max(tolerance)
    }
}

/// 旧二维后端的前向兼容策略；原生 SCAN 不调用此函数。
pub fn forward_navigation_kinematics(mut motion: MotionKinematics) -> MotionKinematics {
    if motion.model == nuwax_nav_types_messages::MotionModel::Omnidirectional {
        motion.model = nuwax_nav_types_messages::MotionModel::Differential;
    }
    motion.max_reverse_mps = 0.;
    motion.max_lateral_mps = 0.;
    motion
}
/// 平台真实尺寸、运动与目标配置。
///
/// SCAN 使用平台声明的前进、倒退、横移与转向通道，不提升物理能力。
#[derive(Clone, Debug)]
pub struct SurfaceFollowerConfig {
    /// 原生局部搜索与样条配置，尺寸由地图真实机身覆盖。
    pub scan: nuwax_nav_scan_native::Config,
    /// 机体中心到地面的真实标定。
    pub body_to_support_z_m: f64,
    /// 真实运动和制动标定。
    pub motion_limits: Option<MotionLimits>,
    /// 无完整制动标定时已有的运动通道。
    pub basic_kinematics: Option<MotionKinematics>,
    /// 守卫转交的设备命令帧限制，供局部搜索预留刚体变换耦合量。
    pub command_limits: Option<nuwax_motion_safety_guard::wire::MotionCommandLimits>,
    /// 感知拥有方配置的来源时效。
    pub max_observation_age_sec: Option<f64>,
    /// 原速度、周期和目标容差。
    pub controller: LocalPlannerConfig,
    /// 实际角速度上限。
    pub max_angular_speed_rps: f64,
}
impl SurfaceFollowerConfig {
    /// 创建物理标定明确的三维执行配置。
    pub fn new(body_to_support_z_m: f64, obstacle_clearance_m: f64) -> Self {
        Self {
            scan: nuwax_nav_scan_native::Config {
                resolution: 0.1,
                body_length: 0.5,
                body_width: 0.3,
                body_height: 0.3,
                body_to_ground: body_to_support_z_m,
                max_velocity: 0.5,
                max_acceleration: 0.5,
                max_yaw_rate: 0.55,
                // 作者 A 原版 SCAN 的默认规划范围；显式业务配置仍由调用方保留。
                planning_horizon: 3.5,
                sample_dt: 0.05,
                body_center_x: 0.,
                body_center_y: 0.,
                body_z_min: -0.15,
                body_z_max: 0.15,
                max_step_up: 0.2,
                max_step_down: 0.2,
                max_slope: 35_f64.to_radians(),
                max_yaw_acceleration: 11.,
                max_reverse_velocity: 0.,
                max_lateral_velocity: 0.,
                obstacle_clearance_m,
            },
            body_to_support_z_m,
            motion_limits: None,
            basic_kinematics: None,
            command_limits: None,
            max_observation_age_sec: None,
            controller: LocalPlannerConfig::default_config(),
            max_angular_speed_rps: 0.55,
        }
    }
    /// SCAN直接保留实际运动声明，不套用旧二维后端的前向策略。
    pub fn kinematics(&self) -> Option<MotionKinematics> {
        self.motion_limits
            .as_ref()
            .map(MotionLimits::kinematics)
            .or(self.basic_kinematics)
    }
}
use crate::local_workers::{
    MapInitializer, MapInput, MapWorker, PlanInput, PlanWorker, PreparedMap,
};
use nuwax_nav_scan_native::{MotionState, ScanTracker, Trajectory};

/// 目标末段的实际执行阶段，供显示与离线闭环读取。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerminalPhase {
    /// 沿参考路线向转向点行进。
    Travel,
    /// 在转向点采用原末端朝向控制。
    Turning,
    /// 保持目标朝向平移至停点。
    Translating,
}
struct TerminalApproach {
    points: nuwax_nav_scan_native::TerminalPoints,
    snapshot: nuwax_nav_scan_native::CollisionSnapshot,
    reference: Arc<Vec<[f64; 3]>>,
}

/// 地图、规划线程的入口与当拍跟踪器；运动端仍只由调度器拥有。
pub struct SurfaceFollower {
    config: SurfaceFollowerConfig,
    map: Option<Arc<SurfaceSnapshot>>,
    odometry: Option<PoseStamped>,
    goal: Option<PoseStamped>,
    path: Option<Path3D>,
    reference: Arc<Vec<[f64; 3]>>,
    reference_start: usize,
    boundary: Arc<Vec<[f64; 2]>>,
    options: GoalOptions,
    applied: Twist2,
    measured_motion: Option<crate::MeasuredMotion>,
    registered_frame: Option<Arc<crate::RegisteredFrame>>,
    motion_boundary: Option<crate::motion_feedback::MotionBoundary>,
    planning_velocity: [f64; 3],
    motion_sample_time: Option<i64>,
    progress: f64,
    local_path: Vec<[f64; 3]>,
    local_body_path: Option<Path3D>,
    wait_reason: Option<String>,
    map_worker: MapWorker,
    plan_worker: PlanWorker,
    tracker: ScanTracker,
    prepared: Option<PreparedMap>,
    active: Option<Trajectory>,
    active_snapshot: Option<nuwax_nav_scan_native::CollisionSnapshot>,
    pending_curve: Option<(Trajectory, nuwax_nav_scan_native::CollisionSnapshot)>,
    plan_snapshot: Option<nuwax_nav_scan_native::CollisionSnapshot>,
    epoch: u64,
    phase: f64,
    needs_plan: bool,
    planning: bool,
    early_requested: bool,
    plans: u64,
    execution_cycle_complete: bool,
    terminal_phase: TerminalPhase,
    terminal_approach: Option<TerminalApproach>,
    terminal_heading: f64,
    terminal_translation_finished: bool,
    terminal_translation_progress: Option<(f64, i64)>,
}
impl SurfaceFollower {
    /// 生产地图只接受注册原帧；地图维护线程独占概率图。
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
        Self::with_map_initializer(config, None)
    }
    /// 显式地图初始化函数，供静态数值夹具使用；不保存测试模式。
    pub fn with_map_initializer(
        config: SurfaceFollowerConfig,
        initializer: Option<MapInitializer>,
    ) -> Result<Self, RuntimeError> {
        Self::validate_config(&config)?;
        let map_error = |reason| RuntimeError::Planner(reason);
        Ok(Self {
            config,
            map: None,
            odometry: None,
            goal: None,
            path: None,
            reference: Arc::new(Vec::new()),
            reference_start: 0,
            boundary: Arc::new(Vec::new()),
            options: GoalOptions::default(),
            applied: Twist2::ZERO,
            measured_motion: None,
            registered_frame: None,
            motion_boundary: None,
            planning_velocity: [0.; 3],
            motion_sample_time: None,
            progress: 0.,
            local_path: Vec::new(),
            local_body_path: None,
            wait_reason: None,
            map_worker: MapWorker::new(initializer).map_err(map_error)?,
            plan_worker: PlanWorker::new().map_err(map_error)?,
            tracker: ScanTracker::default(),
            pending_curve: None,
            plan_snapshot: None,
            prepared: None,
            active: None,
            active_snapshot: None,
            epoch: 0,
            phase: 0.,
            needs_plan: true,
            planning: false,
            early_requested: false,
            plans: 0,
            execution_cycle_complete: false,
            terminal_phase: TerminalPhase::Travel,
            terminal_approach: None,
            terminal_heading: 0.,
            terminal_translation_finished: false,
            terminal_translation_progress: None,
        })
    }
    /// 在创建工作线程和连接运动端之前核对物理配置。
    pub fn validate_config(config: &SurfaceFollowerConfig) -> Result<(), RuntimeError> {
        if let Some(limits) = &config.command_limits {
            limits
                .validate()
                .map_err(|reason| RuntimeError::InvalidInput {
                    field: "surface.command_limits",
                    reason,
                })?;
        }
        if let Some(limits) = &config.motion_limits {
            limits
                .validate()
                .map_err(|reason| RuntimeError::InvalidInput {
                    field: "surface.motion_limits",
                    reason,
                })?;
        }
        if let Some(motion) = config.basic_kinematics {
            motion
                .validate()
                .map_err(|reason| RuntimeError::InvalidInput {
                    field: "surface.kinematics",
                    reason,
                })?;
        }
        for (field, value) in [
            ("body_to_support_z_m", config.body_to_support_z_m),
            ("surface.speed", config.controller.speed),
            (
                "surface.control_frequency",
                config.controller.control_frequency,
            ),
            (
                "surface.max_angular_speed_rps",
                config.max_angular_speed_rps,
            ),
        ] {
            if !value.is_finite() || value <= 0. {
                return Err(RuntimeError::InvalidInput {
                    field,
                    reason: "必须提供有限正值".into(),
                });
            }
        }
        Ok(())
    }
    /// 最近的真实运动边界，回执不充当测速。
    pub fn measured_boundary(&self) -> Option<&crate::motion_feedback::MotionBoundary> {
        self.motion_boundary.as_ref()
    }
    /// 当前末段阶段的只读观测。
    pub fn terminal_phase(&self) -> TerminalPhase {
        self.terminal_phase
    }
    /// 当前目标的末段选点；无朝向要求时不改写参考路线。
    pub fn terminal_points(&self) -> Option<nuwax_nav_scan_native::TerminalPoints> {
        self.terminal_approach.as_ref().map(|a| a.points)
    }
    /// 沿全局路线的实际进度。
    pub fn progress_m(&self) -> f64 {
        self.progress
    }
    /// 已完成并安装的局部规划数。
    pub fn local_plans(&self) -> u64 {
        self.plans
    }
    /// 已提交的局部规划数。
    pub fn local_planning_calls(&self) -> u64 {
        self.plans + u64::from(self.planning)
    }
    /// 当前原生轨迹的显示副本。
    pub fn local_path(&self) -> Option<&Path3D> {
        self.local_body_path.as_ref()
    }
    /// 当前原生轨迹的显示副本。
    pub fn local_body_path_for_display(&self) -> Option<Path3D> {
        self.local_body_path.clone()
    }
    /// 实际进度观测。
    pub fn progress(&self) -> f64 {
        self.progress
    }
    fn position(pose: &PoseStamped) -> [f64; 3] {
        let p = pose.pose.position;
        [p.x, p.y, p.z]
    }
    fn update_progress(&mut self) {
        let Some(pose) = &self.odometry else {
            return;
        };
        let point = Self::position(pose);
        let mut along = 0.;
        let mut nearest = (f64::INFINITY, self.progress, self.reference_start);
        for (index, pair) in self.reference.windows(2).enumerate() {
            let a = pair[0];
            let b = pair[1];
            let v = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let length = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            if length == 0. {
                continue;
            }
            if index < self.reference_start {
                along += length;
                continue;
            }
            let t = ((0..3).map(|i| (point[i] - a[i]) * v[i]).sum::<f64>() / (length * length))
                .clamp(0., 1.);
            let distance = (0..3)
                .map(|i| (point[i] - a[i] - t * v[i]).powi(2))
                .sum::<f64>();
            if distance < nearest.0 {
                nearest = (distance, along + t * length, index);
            }
            along += length;
        }
        self.progress = self.progress.max(nearest.1);
        self.reference_start = nearest.2;
    }
    fn position_reached(&self) -> bool {
        let (Some(pose), Some(goal)) = (&self.odometry, &self.goal) else {
            return false;
        };
        let config = self.map.as_ref().map_or(self.config.scan, |map| self.native_config(map));
        position_within_tolerance(Self::position(pose), Self::position(goal),
            self.options.position_tolerance_m, config.max_step_up.max(config.max_step_down))
    }
    fn reached(&self) -> bool {
        let (Some(pose), Some(goal)) = (&self.odometry, &self.goal) else {
            return false;
        };
        let yaw = if !self.options.requires_orientation() {
            true
        } else if self.options.yaw_tolerance_rad == 0. {
            nuwax_nav_types_messages::same_finite_yaw(
                pose.pose.orientation.yaw(),
                goal.pose.orientation.yaw(),
            )
        } else {
            nuwax_nav_astar::controllers::angle_diff(
                goal.pose.orientation.yaw(),
                pose.pose.orientation.yaw(),
            )
            .abs()
                < self.options.yaw_tolerance_rad
        };
        let boundary = crate::motion_feedback::boundary_for_pose(pose, self.measured_motion.as_ref());
        let state = MotionState {
            velocity: boundary.velocity.unwrap_or([0.; 3]),
            yaw_rate: boundary.yaw_rate.unwrap_or(0.),
            measured_velocity: u32::from(boundary.velocity.is_some()),
            measured_yaw_rate: u32::from(boundary.yaw_rate.is_some()),
            ..Default::default()
        };
        self.terminal_translation_finished && self.position_reached() && yaw && state.stopped()
    }
    fn native_config(&self, map: &SurfaceSnapshot) -> nuwax_nav_scan_native::Config {
        let mut config = self.config.scan;
        config.resolution = map.navigation_source_columns(None).0;
        config.body_to_ground = self.config.body_to_support_z_m;
        config.sample_dt = 1. / self.config.controller.control_frequency;
        config.max_velocity =
            self.config.controller.speed * self.config.controller.nerf_speed.min(1.);
        config.max_yaw_rate = self.config.max_angular_speed_rps;
        config.max_yaw_acceleration = config.max_yaw_rate / config.sample_dt;
        config.max_step_up = f64::from(map.config().step_threshold_m);
        config.max_step_down = config.max_step_up;
        config.max_slope = f64::from(map.config().max_slope_deg).to_radians();
        if let Some(motion) = self.config.kinematics() {
            config.max_velocity = config.max_velocity.min(motion.max_forward_mps);
            config.max_reverse_velocity = if self.options.allow_reverse {
                motion.max_reverse_mps.min(config.max_velocity)
            } else {
                0.
            };
            config.max_lateral_velocity = motion.max_lateral_mps.min(config.max_velocity);
            config.max_yaw_rate = config.max_yaw_rate.min(motion.max_angular_rps);
        }
        if let Some(limits) = &self.config.motion_limits {
            config.max_acceleration = limits
                .linear_acceleration_mps2
                .min(limits.guaranteed_braking_mps2);
            config.max_yaw_acceleration = limits
                .angular_acceleration_rps2
                .min(limits.guaranteed_angular_braking_rps2);
        }
        if let Some(limits) = &self.config.command_limits {
            let [forward, reverse, lateral, yaw] = limits.search_bounds(
                config.max_velocity,
                config.max_reverse_velocity,
                config.max_lateral_velocity,
                config.max_yaw_rate,
            );
            config.max_velocity = forward;
            config.max_reverse_velocity = reverse;
            config.max_lateral_velocity = lateral;
            config.max_yaw_rate = yaw;
        }
        if let Some(body) = map.body_envelope() {
            let bounds = body.footprint_xy_m.iter().fold(
                [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]],
                |mut b, p| {
                    for i in 0..2 {
                        b[0][i] = b[0][i].min(p[i]);
                        b[1][i] = b[1][i].max(p[i]);
                    }
                    b
                },
            );
            config.body_length = bounds[1][0] - bounds[0][0];
            config.body_width = bounds[1][1] - bounds[0][1];
            config.body_height = body.z_bounds_m[1] - body.z_bounds_m[0];
            config.body_center_x = (bounds[0][0] + bounds[1][0]) * 0.5;
            config.body_center_y = (bounds[0][1] + bounds[1][1]) * 0.5;
            config.body_z_min = body.z_bounds_m[0];
            config.body_z_max = body.z_bounds_m[1];
            config.max_step_up = body.max_step_up_m;
            config.max_step_down = body.max_step_down_m;
            config.max_slope = body.max_cross_slope_deg.to_radians();
        }
        config
    }
    fn submit_map(&self) {
        let (Some(source), Some(pose)) = (&self.map, &self.odometry) else {
            return;
        };
        self.map_worker.submit(MapInput {
            epoch: self.epoch,
            source: source.clone(),
            config: self.native_config(source),
            boundary: self.boundary.clone(),
            pose: Self::position(pose),
            yaw: pose.pose.orientation.yaw(),
            frame: self.registered_frame.clone(),
            active: self.active.clone(),
            active_snapshot: self.active_snapshot.clone(),
            phase: self.phase,
        });
    }
    fn retire_route(&mut self) {
        self.epoch += 1;
        self.plan_worker.retire(self.epoch);
        self.map_worker.retire(self.epoch, false);
        self.tracker.clear_trajectory();
        self.pending_curve = None;
        self.plan_snapshot = None;
        self.active = None;
        self.active_snapshot = None;
        self.phase = 0.;
        self.planning = false;
        self.needs_plan = true;
        self.early_requested = false;
        self.local_path.clear();
        self.local_body_path = None;
        self.execution_cycle_complete = false;
        self.wait_reason = None;
        self.motion_sample_time = None;
        self.planning_velocity = [0.; 3];
        self.terminal_phase = TerminalPhase::Travel;
        self.terminal_approach = None;
        self.terminal_heading = 0.;
        self.terminal_translation_finished = false;
        self.terminal_translation_progress = None;
    }
    fn prepare_terminal(&mut self, map: &PreparedMap, config: &nuwax_nav_scan_native::Config) {
        if self.terminal_phase != TerminalPhase::Travel || !self.options.requires_orientation()
            || self.terminal_approach.as_ref().is_some_and(|old| old.snapshot.same(&map.snapshot)) {
            return;
        }
        let goal = self.goal.as_ref().unwrap();
        let points = map.snapshot.terminal_points(config, Self::position(goal),
            goal.pose.orientation.yaw(), self.options.position_tolerance_m, &self.reference);
        let reference = if points.turn != points.stop {
            let mut clipped = Vec::new();
            let mut arc = 0.;
            for (i, point) in self.reference.iter().enumerate() {
                if i > 0 {
                    arc += (0..3).map(|axis| (point[axis] - self.reference[i-1][axis]).powi(2)).sum::<f64>().sqrt();
                }
                if arc >= points.route_arc { break; }
                clipped.push(*point);
            }
            if clipped.last() != Some(&points.route_point) {
                clipped.push(points.route_point);
            }
            if points.turn != points.route_point {
                clipped.push(points.turn);
            }
            clipped
        } else {
            let mut reference = self.reference.as_ref().clone();
            if points.stop != Self::position(goal) {
                if let Some(last) = reference.last_mut() { *last = points.stop; }
            }
            reference
        };
        self.needs_plan |= self.terminal_approach.as_ref().is_some_and(|old| *old.reference != reference);
        self.terminal_approach = Some(TerminalApproach { points, snapshot: map.snapshot.clone(), reference: Arc::new(reference) });
    }
    fn at_turn_point(&self, state: &MotionState, resolution: f64) -> bool {
        match self.terminal_approach.as_ref() {
            Some(approach) if approach.points.turn != approach.points.stop => {
                let point = approach.points.turn;
                (state.position[0] - point[0]).hypot(state.position[1] - point[1])
                    < self.options.position_tolerance_m.min(0.05_f64.max(resolution))
            }
            _ => self.position_reached(),
        }
    }
    fn begin_terminal_turn(&mut self) {
        // 退休在途规划，防止进入末端阶段后再装回一条旧曲线；保留脱困事件与记账。
        self.epoch += 1;
        self.plan_worker.retire(self.epoch);
        self.map_worker.retire(self.epoch, false);
        self.tracker.clear_trajectory();
        self.active = None;
        self.active_snapshot = None;
        self.pending_curve = None;
        self.plan_snapshot = None;
        self.planning = false;
        self.needs_plan = false;
        self.phase = 0.;
        self.local_path.clear();
        self.local_body_path = None;
        self.terminal_translation_finished = false;
        self.terminal_translation_progress = None;
        self.terminal_heading = if self.options.requires_orientation() {
            self.goal.as_ref().unwrap().pose.orientation.yaw()
        } else {
            self.odometry.as_ref().unwrap().pose.orientation.yaw()
        };
        self.terminal_phase = if self.options.requires_orientation() {
            TerminalPhase::Turning
        } else {
            TerminalPhase::Translating
        };
    }
    fn state(&mut self) -> MotionState {
        let pose = self.odometry.as_ref().unwrap();
        let boundary =
            crate::motion_feedback::boundary_for_pose(pose, self.measured_motion.as_ref());
        let measured = boundary.velocity.unwrap_or([0.; 3]);
        if let Some(previous) = self.motion_sample_time {
            if pose.stamp.nanos > previous {
                let weight = -(-((pose.stamp.nanos - previous) as f64) * 1e-9).exp_m1();
                for i in 0..2 {
                    self.planning_velocity[i] += weight * (measured[i] - self.planning_velocity[i]);
                }
                self.planning_velocity[2] = measured[2];
            }
        } else {
            self.planning_velocity = measured;
        }
        self.motion_sample_time = Some(pose.stamp.nanos);
        let state = MotionState {
            position: Self::position(pose),
            velocity: measured,
            planning_velocity: self.planning_velocity,
            yaw: pose.pose.orientation.yaw(),
            yaw_rate: boundary.yaw_rate.unwrap_or(0.),
            route_progress: self.progress,
            timestamp_ns: pose.stamp.nanos,
            measured_velocity: u32::from(boundary.velocity.is_some()),
            measured_yaw_rate: u32::from(boundary.yaw_rate.is_some()),
        };
        self.motion_boundary = Some(boundary);
        state
    }
    fn compute(&mut self) -> Result<Twist2, String> {
        self.update_progress();
        let state = self.state();
        if let Some(result) = self.map_worker.take() {
            match result {
                Ok(map) if map.epoch == self.epoch => {
                    self.needs_plan |= map.changed
                        && map
                            .checked_curve
                            .as_ref()
                            .zip(self.active.as_ref())
                            .is_some_and(|(old, active)| old.same(active));
                    self.prepared = Some(map)
                }
                Ok(_) => {}
                Err(reason) => {
                    self.prepared = None;
                    self.wait_reason = Some(reason);
                }
            }
        }
        if let Some((epoch, result)) = self.plan_worker.take() {
            if epoch == self.epoch {
                self.planning = false;
                match result {
                    Ok(curve) => {
                        self.tracker.install(&curve);
                        self.pending_curve = Some((curve, self.plan_snapshot.take().unwrap()));
                    }
                    Err(reason) => {
                        self.wait_reason = Some(reason);
                        self.needs_plan = true;
                    }
                }
            }
        }
        if !self.position_reached()
            && (self.reference.len() == 1
                || nuwax_nav_types_messages::same_finite_position(
                    state.position,
                    *self.reference.last().unwrap(),
                ))
        {
            self.execution_cycle_complete = true;
            return Err("全局参考已结束，原业务目标未到，等待后续路线".into());
        }
        let Some(map) = self.prepared.clone() else {
            self.submit_map();
            return Ok(Twist2::ZERO);
        };
        let config = self.native_config(&map.source);
        self.prepare_terminal(&map, &config);
        let at_turn_point = self.at_turn_point(&state, config.resolution);
        if self.terminal_phase == TerminalPhase::Turning && !at_turn_point {
            self.terminal_phase = TerminalPhase::Travel;
            self.needs_plan = true;
        } else if self.terminal_phase == TerminalPhase::Travel
            && at_turn_point && ((self.active.is_none() && self.pending_curve.is_none()) || state.stopped()) {
            self.begin_terminal_turn();
        }
        if self.terminal_phase == TerminalPhase::Turning {
            let goal_yaw = self.goal.as_ref().unwrap().pose.orientation.yaw();
            let aligned = if self.options.yaw_tolerance_rad == 0. {
                nuwax_nav_types_messages::same_finite_yaw(state.yaw, goal_yaw)
            } else {
                nuwax_nav_astar::controllers::angle_diff(goal_yaw, state.yaw).abs() < self.options.yaw_tolerance_rad
            };
            if aligned { self.terminal_phase = TerminalPhase::Translating; }
        }
        let terminal_yaw = (self.terminal_phase != TerminalPhase::Travel)
            .then_some(self.terminal_heading);
        let stop = self.terminal_approach.as_ref().map_or_else(
            || Self::position(self.goal.as_ref().unwrap()), |a| a.points.stop);
        if self.terminal_phase == TerminalPhase::Translating && !self.terminal_translation_finished {
            let distance = (state.position[0] - stop[0]).hypot(state.position[1] - stop[1]);
            let progress = self.terminal_translation_progress.get_or_insert((distance, state.timestamp_ns));
            if distance < progress.0 { *progress = (distance, state.timestamp_ns); }
            self.terminal_translation_finished = distance <= 0.02 || state.timestamp_ns - progress.1 >= 1_000_000_000;
        }
        let terminal_position = (self.terminal_phase == TerminalPhase::Translating
            && !self.terminal_translation_finished).then_some(stop);
        let mut tracking_failure = None;
        let checked = match self.tracker.track_terminal_position(
            &config,
            &map.snapshot,
            &state,
            [
                self.applied.linear.x,
                self.applied.linear.y,
                self.applied.angular_z,
            ],
            terminal_yaw,
            terminal_position,
        ) {
            Ok(checked) => checked,
            Err(failure) => {
                tracking_failure = Some(failure.message);
                nuwax_nav_scan_native::TrackResult {
                    reason: failure.reason,
                    ..Default::default()
                }
            }
        };
        if checked.installed {
            let (curve, snapshot) = self.pending_curve.take().unwrap();
            self.local_path = curve
                .samples(0., self.config.scan.sample_dt)
                .iter()
                .map(|p| p.position)
                .collect();
            self.local_body_path = self.path.as_ref().map(|path| Path3D {
                poses: self
                    .local_path
                    .iter()
                    .map(|p| {
                        let mut pose = self.odometry.as_ref().unwrap().clone();
                        pose.pose.position =
                            nuwax_nav_types_messages::Point3::new(p[0], p[1], p[2]);
                        pose
                    })
                    .collect(),
                ..path.clone()
            });
            self.active = Some(curve);
            self.active_snapshot = Some(snapshot);
            self.early_requested = false;
            self.plans += 1;
        }
        if checked.rejected_trajectory() {
            self.pending_curve = None;
            self.needs_plan = true;
        }
        self.phase = checked.phase;
        if checked.replan_early() && !self.early_requested && !at_turn_point {
            self.needs_plan = true;
            self.early_requested = true;
        }
        if checked.invalid() {
            self.active = None;
            self.active_snapshot = None;
            self.needs_plan = true;
        }
        if self.terminal_phase == TerminalPhase::Translating {
            self.terminal_translation_finished |= (checked.no_candidate() || !checked.invalid())
                && checked.command[0] == 0. && checked.command[1] == 0.;
            if self.terminal_translation_finished && !self.position_reached() {
                self.terminal_phase = TerminalPhase::Travel;
                self.terminal_translation_finished = false;
                self.terminal_translation_progress = None;
                self.needs_plan = true;
            }
        } else if checked.no_candidate() {
            self.terminal_phase = TerminalPhase::Travel;
        }
        // 已有计算时合并请求；这次计算交付后只由后续新事件再次请求。
        if self.needs_plan {
            if !self.planning && self.terminal_phase == TerminalPhase::Travel && !at_turn_point {
                self.plan_snapshot = Some(map.snapshot.clone());
                self.plan_worker.submit(PlanInput {
                    epoch: self.epoch,
                    map: map.clone(),
                    config,
                    reference: self.terminal_approach.as_ref().map_or_else(|| self.reference.clone(), |a| a.reference.clone()),
                    state,
                    active: self.active.clone(),
                    phase: self.phase,
                });
                self.planning = true;
            }
            self.needs_plan = false;
        }
        self.wait_reason = if checked.invalid() {
            Some(
                if checked.reason & nuwax_nav_scan_native::track_reason::RECOVERY_FAILED != 0 {
                    format!("机身与障碍重叠、脱困失败：{}", tracking_failure.as_deref().unwrap())
                } else if checked.no_candidate() {
                    "命令检查没有合法候选，等待重规划".into()
                } else if checked.deviated() {
                    "机身偏离轨迹，等待重规划".into()
                } else if checked.finished() {
                    "局部轨迹结束，等待重规划".into()
                } else {
                    "等待局部轨迹".into()
                },
            )
        } else {
            None
        };
        self.submit_map();
        Ok(Twist2 {
            linear: Point2::new(checked.command[0], checked.command[1]),
            angular_z: checked.command[2],
        })
    }
}
impl NavPlanner for SurfaceFollower {
    fn set_control_period(&mut self, period: std::time::Duration) {
        self.config.controller.control_frequency = 1. / period.as_secs_f64();
    }
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: GoalOptions,
    ) -> Result<(), RuntimeError> {
        self.retire_route();
        self.tracker.clear();
        self.goal = Some(goal.clone());
        self.options = options;
        self.path = None;
        self.reference = Arc::new(Vec::new());
        self.reference_start = 0;
        self.progress = 0.;
        Ok(())
    }
    fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
        self.boundary = Arc::new(boundary.to_vec());
        self.submit_map();
    }
    fn replace_path(&mut self, _path: Path2D) {
        self.wait_reason = Some("三维执行需要完整机体参考".into());
    }
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        _yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        let ExecutionPath::Surface(path) = path else {
            return Err(RuntimeError::PathRejected {
                reason: "三维执行需要完整机体参考".into(),
            });
        };
        for pose in &path.poses {
            crate::validate_map_pose(pose, "path.pose")?;
        }
        if path.poses.is_empty() {
            return Err(RuntimeError::PathRejected {
                reason: "参考路径为空".into(),
            });
        }
        self.retire_route();
        self.reference = Arc::new(path.poses.iter().map(Self::position).collect());
        self.reference_start = 0;
        self.path = Some(path);
        self.goal = Some(goal);
        self.progress = 0.;
        self.update_progress();
        self.submit_map();
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        self.odometry = Some(pose);
        self.update_progress();
        self.submit_map();
    }
    fn update_costmap(&mut self, _map: CostMap2D) {
        self.invalidate_execution_map();
    }
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        self.map = Some(match map {
            ExecutionMap::Surface(map) => map,
            ExecutionMap::SurfaceUpdate(update) => update.snapshot,
            ExecutionMap::Planar(_) => {
                return Err(RuntimeError::InvalidInput {
                    field: "execution_map",
                    reason: "三维执行需要三维感知".into(),
                })
            }
        });
        self.submit_map();
        Ok(())
    }
    fn invalidate_execution_map(&mut self) {
        self.retire_route();
        self.map_worker.retire(self.epoch, true);
        self.prepared = None;
        self.map = None;
        self.registered_frame = None;
    }
    fn observe_applied_command(&mut self, command: Twist2) {
        self.applied = command;
    }
    fn observe_registered_frame(&mut self, frame: Option<Arc<crate::RegisteredFrame>>) {
        if self.registered_frame.as_ref().map(|f| &f.identity)
            == frame.as_ref().map(|f| &f.identity)
        {
            return;
        }
        if self.registered_frame.as_ref().map(|f| &f.identity.domain)
            != frame.as_ref().map(|f| &f.identity.domain)
        {
            self.retire_route();
            self.map_worker.retire(self.epoch, true);
            self.prepared = None;
        }
        self.registered_frame = frame;
        self.submit_map();
    }
    fn observe_measured_motion(&mut self, motion: Option<crate::MeasuredMotion>) {
        self.measured_motion = motion.filter(crate::MeasuredMotion::is_current);
    }
    fn refresh_execution_context(
        &mut self,
        applied: Twist2,
    ) -> Option<Arc<crate::replan_feedback::ActualExecutionContext>> {
        Some(Arc::new(crate::replan_feedback::ActualExecutionContext {
            source: self.map.clone()?,
            odometry: self.odometry.clone()?,
            applied,
            measured_motion: self
                .measured_motion
                .clone()
                .filter(crate::MeasuredMotion::is_current),
        }))
    }
    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }
    fn terminal_phase(&self) -> TerminalPhase {
        self.terminal_phase
    }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        self.local_path.clone()
    }
    fn execution_cycle_complete(&self) -> bool {
        self.execution_cycle_complete
    }
    fn step_at_goal(&mut self) -> Option<PlannerStep> {
        self.position_reached().then(|| self.step())
    }
    fn step(&mut self) -> PlannerStep {
        self.execution_cycle_complete = false;
        if self.map.is_none() || self.odometry.is_none()
            || (self.reference.is_empty() && !self.position_reached()) {
            return PlannerStep::Pending(Vec::new());
        }
        if self.reached() {
            return PlannerStep::Succeeded;
        }
        match self.compute() {
            Ok(_) if self.reached() => PlannerStep::Succeeded,
            Ok(command) => PlannerStep::Command(command),
            Err(reason) => {
                self.wait_reason = Some(reason.clone());
                if self.execution_cycle_complete {
                    PlannerStep::ReplanRequired(reason)
                } else {
                    self.needs_plan = true;
                    PlannerStep::Command(Twist2::ZERO)
                }
            }
        }
    }
    fn stop(&mut self) {
        self.retire_route();
        self.path = None;
        self.reference = Arc::new(Vec::new());
        self.reference_start = 0;
        self.progress = 0.;
    }
}

#[cfg(test)]
pub(crate) fn test_snapshot(slope: f64, blocked_strip: bool) -> Arc<SurfaceSnapshot> {
    use nuwax_nav_mls_planner::mls_planner::Config;
    let config = Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.6,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.4,
        wall_clearance_m: 0.2,
        wall_buffer_m: 0.2,
        wall_buffer_weight: 0.0,
        step_threshold_m: 0.2,
        max_slope_deg: 35.0,
        min_step_tread_depth_m: 0.3,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.12,
        viz_publish_hz: 0.0,
    };
    let mut points = Vec::new();
    for ix in 0..65 {
        if blocked_strip && (10..=13).contains(&ix) {
            continue;
        }
        for iy in 0..35 {
            let x = (ix as f64 + 0.5) * 0.1;
            points.push((x as f32, (iy as f32 + 0.5) * 0.1, (0.05 + slope * x) as f32));
        }
    }
    Arc::new(SurfaceSnapshot::from_points(&points, config, 0.1).expect("真实合成表面"))
}

#[cfg(test)]
pub(crate) fn prepared_test_surface(map: Arc<SurfaceSnapshot>) -> ExecutionMap {
    ExecutionMap::Surface(map)
}


#[cfg(test)]
mod native_execution_tests {

    use super::*;
    use nuwax_nav_types_messages::{NavFrameId, Point3, Pose3, Quaternion, Timestamp};

    #[test]
    fn position_tolerance_uses_the_plane_and_keeps_height_and_exact_position_constraints() {
        let (config, source) = crate::native_test_helpers::fixture();
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        let mut goal = pose(0.148, 0., 0.);
        goal.pose.position.z += 0.05;
        follower.configure_goal(&goal, GoalOptions { position_tolerance_m: 0.15, ..Default::default() }).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
        follower.update_odometry(pose(0., 0., 0.));
        assert!(follower.position_reached());
        follower.goal.as_mut().unwrap().pose.position.z = 0.71;
        assert!(!follower.position_reached());
        follower.options.position_tolerance_m = 0.;
        follower.goal.as_mut().unwrap().pose.position = Point3::new(0., 0., 0.55);
        assert!(!follower.position_reached());
        follower.goal.as_mut().unwrap().pose.position.z = 0.5;
        assert!(follower.position_reached());
    }

    #[test]
    fn terminal_turn_leaving_the_same_position_region_requests_a_travel_plan() {
        let (config, source) = crate::native_test_helpers::fixture();
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        let start = pose(0.,0.,0.);
        let goal = pose(0.1,0.,0.6);
        follower.configure_goal(&goal, GoalOptions {position_tolerance_m:0.15,yaw_tolerance_rad:0.1,..Default::default()}).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
        follower.update_odometry(start.clone());
        follower.observe_measured_motion(Some(crate::motion_feedback::tests::measured([0.;3],[0.;3])));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start,goal.clone()]}),None,goal).unwrap();
        for _ in 0..1000 {
            follower.step();
            if follower.terminal_phase()==TerminalPhase::Turning {break;}
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert_eq!(follower.terminal_phase(),TerminalPhase::Turning);
        let mut drifted=pose(0.3,0.,0.);
        drifted.stamp.nanos+=50_000_000;
        follower.update_odometry(drifted);
        follower.step();
        assert_eq!(follower.terminal_phase(),TerminalPhase::Travel);
        assert!(follower.planning,"漂出后须请求原行进规划");
    }

    #[test]
    fn tolerance_edge_finishes_translation_before_success_with_or_without_goal_yaw() {
        for requires_yaw in [true, false] {
            let (mut config, source) = crate::native_test_helpers::fixture();
            config.motion_limits.as_mut().unwrap().model = nuwax_nav_types_messages::MotionModel::Omnidirectional;
            config.motion_limits.as_mut().unwrap().max_lateral_mps = 0.3;
            let dt = 1. / config.controller.control_frequency;
            let mut follower = crate::native_test_helpers::author_static_map_follower(config);
            let start = pose(1., 1., 0.);
            let goal = pose(1.14, 1., 1.);
            follower.configure_goal(&goal, GoalOptions {position_tolerance_m: 0.15,
                yaw_tolerance_rad: if requires_yaw { 0.1 } else { std::f64::consts::PI },
                ..Default::default()}).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
            follower.update_odometry(start.clone());
            follower.observe_measured_motion(Some(crate::motion_feedback::tests::measured([0.;3], [0.;3])));
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start, goal.clone()]}), None, goal.clone()).unwrap();
            let mut moving = false;
            let mut succeeded = false;
            for _ in 0..1000 {
                let step = follower.step();
                if matches!(step, PlannerStep::Succeeded) {
                    let p = follower.odometry.as_ref().unwrap().pose.position;
                    assert!((p.x-goal.pose.position.x).hypot(p.y-goal.pose.position.y) <= 0.02);
                    assert!(moving, "不能在容差边缘提前成功");
                    if !requires_yaw {
                        assert!(follower.odometry.as_ref().unwrap().pose.orientation.yaw().abs() < 1e-8,
                            "无朝向要求的平移保持进入时朝向");
                    }
                    succeeded = true; break;
                }
                let command = match step {PlannerStep::Command(v) => v, _ => Twist2::ZERO};
                moving |= command.linear.x.hypot(command.linear.y) > 0.;
                let mut p = follower.odometry.as_ref().unwrap().clone();
                let yaw = p.pose.orientation.yaw();
                p.pose.position.x += (yaw.cos()*command.linear.x-yaw.sin()*command.linear.y)*dt;
                p.pose.position.y += (yaw.sin()*command.linear.x+yaw.cos()*command.linear.y)*dt;
                p.pose.orientation = Quaternion::from_yaw(yaw+command.angular_z*dt);
                p.stamp.nanos += (dt*1e9) as i64;
                follower.update_odometry(p);
                follower.observe_applied_command(command);
                follower.observe_measured_motion(Some(crate::motion_feedback::tests::measured(
                    [command.linear.x, command.linear.y, 0.], [0.,0.,command.angular_z])));
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
            assert!(succeeded, "末段应完成平移与实测停车，朝向要求={requires_yaw}");
        }
    }

    #[test]
    fn terminal_translation_finishes_after_one_source_second_without_a_new_minimum() {
        let (config, source) = crate::native_test_helpers::fixture();
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        let start = pose(1., 1., 0.);
        let goal = pose(1.1, 1., 0.);
        follower.configure_goal(&goal, GoalOptions {position_tolerance_m: 0.15,
            yaw_tolerance_rad: std::f64::consts::PI, ..Default::default()}).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
        follower.update_odometry(start.clone());
        follower.observe_measured_motion(Some(crate::motion_feedback::tests::measured([0.;3], [0.;3])));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses: vec![start.clone(),goal.clone()]}), None, goal.clone()).unwrap();
        crate::native_test_helpers::prepare_terminal_map(&mut follower);
        assert!(!follower.terminal_translation_finished);
        // 刷新最小距离重置来源计时；同源调用不能积成1秒。
        let mut closer = start;
        closer.pose.position.x += 0.02;
        closer.stamp.nanos += 500_000_000;
        follower.update_odometry(closer.clone());
        for _ in 0..30 {
            assert_ne!(follower.step(), PlannerStep::Succeeded);
            assert!(!follower.terminal_translation_finished);
        }
        closer.stamp.nanos += 999_999_999;
        follower.update_odometry(closer.clone());
        assert_ne!(follower.step(), PlannerStep::Succeeded);
        assert!(!follower.terminal_translation_finished);
        closer.stamp.nanos += 1;
        follower.update_odometry(closer);
        assert_eq!(follower.step(), PlannerStep::Succeeded, "距S仍8厘米但1来源秒无新最小值、在原容差内并实测停稳，应结束平移后完成");
        assert!(follower.terminal_translation_finished);
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        assert!(follower.terminal_translation_progress.is_none());
    }

    #[test]
    fn blocked_terminal_translation_succeeds_in_tolerance_or_returns_to_travel() {
        for within in [true, false] {
            let (config, source) = crate::native_test_helpers::fixture();
            let mut follower = crate::native_test_helpers::author_static_map_follower(config);
            // 设备不能横移，保持零朝向时侧向目标的检查输出为零。
            let start = pose(1.,1.,0.);
            let goal = pose(1., if within {1.1} else {1.3}, 0.);
            follower.configure_goal(&goal, GoalOptions {position_tolerance_m:0.15,
                yaw_tolerance_rad:std::f64::consts::PI,..Default::default()}).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
            follower.update_odometry(start.clone());
            follower.observe_measured_motion(Some(crate::motion_feedback::tests::measured([0.;3],[0.;3])));
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start,goal.clone()]}),None,goal).unwrap();
            for _ in 0..1000 {
                follower.compute().unwrap();
                if follower.prepared.is_some() {break;}
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
            // 容差外变体模拟进入平移后发生位置漂移，规则只在平移结束时回查位置。
            follower.begin_terminal_turn();
            follower.compute().unwrap();
            if within {
                assert!(follower.terminal_translation_finished);
                assert!(matches!(follower.step(),PlannerStep::Succeeded));
            } else {
                assert_eq!(follower.terminal_phase(),TerminalPhase::Travel);
                assert!(!follower.terminal_translation_finished);
                assert!(follower.planning, "容差外受阻须请求行进规划");
            }
        }
    }

    fn pose(x: f64, y: f64, yaw: f64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(x, y, 0.5),
                orientation: Quaternion::from_yaw(yaw),
            },
        }
    }
    fn follower(start: PoseStamped, source: Arc<SurfaceSnapshot>) -> SurfaceFollower {
        let (config, _) = crate::native_test_helpers::fixture();
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        let goal = pose(3., 1., 0.);
        follower
            .configure_goal(&goal, GoalOptions::default())
            .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(source))
            .unwrap();
        follower.update_odometry(start.clone());
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![start, goal.clone()],
                }),
                None,
                goal,
            )
            .unwrap();
        for _ in 0..1000{let _=follower.step();if follower.active.is_some(){break;}std::thread::sleep(std::time::Duration::from_millis(1));}
        follower
    }
    fn unknown() -> Arc<SurfaceSnapshot> {
        let (_, known) = crate::native_test_helpers::fixture();
        Arc::new(
            SurfaceSnapshot::from_points(&[], known.config().clone(), known.source_voxel_size_m())
                .unwrap()
                .with_body_envelope(known.body_envelope().unwrap().clone())
                .unwrap(),
        )
    }
    fn next_sample(follower: &mut SurfaceFollower) -> PoseStamped {
        let mut current = follower.odometry.as_ref().unwrap().clone();
        current.stamp.nanos +=
            (1_000_000_000. / follower.config.controller.control_frequency) as i64;
        follower.update_odometry(current.clone());
        current
    }

    #[test]
    fn n6_20_same_goal_route_keeps_same_source_stop_and_new_goal_reopens_escape() {
        let (config, source) = crate::native_test_helpers::fixture();
        let mut follower = SurfaceFollower::new(config).unwrap();
        let goal = pose(3., 1., 0.);
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        let c = follower.native_config(&source);
        let initial = [c.body_length / 2. - c.body_center_x - 0.005, 0.025, 0.5];
        let mut motion = MotionState {
            position: initial, measured_velocity: 1, measured_yaw_rate: 1,
            timestamp_ns: 1_000_000_000, ..Default::default()
        };
        let mut map = nuwax_nav_scan_native::MapOwner::new(&c).unwrap();
        map.seed_occupied(&[], initial);
        map.update_surface(&[[-0.025, 0.025, initial[2] + c.body_z_max - c.resolution / 2.]], &[], &[], &[]).unwrap();
        let snapshot = map.snapshot();
        assert!(!follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap().recovery_samples.is_empty());
        motion.position[0] -= 0.001;
        motion.timestamp_ns += 50_000_000;
        let continued = follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap();
        assert_ne!(continued.command,[0.;3],"无实测向内的定位增深不能锁止");
        follower.replace_execution_path(
            ExecutionPath::Surface(Path3D { poses: vec![pose(initial[0], initial[1], 0.), goal.clone()] }),
            None, goal.clone(),
        ).unwrap();
        motion.position = initial;
        assert_eq!(follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap().command, [0.; 3]);
        follower.stop();
        assert_eq!(follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap().command, [0.; 3],
            "同目标请求重规划也会调用stop，同源仍只停车");
        follower.invalidate_execution_map();
        assert_eq!(follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap().command, [0.; 3],
            "同目标地图退休不能借重建跟踪器重复输出动作");
        follower.observe_registered_frame(Some(crate::registered_frame::test_frame(1)));
        assert_eq!(follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap().command, [0.; 3],
            "同目标注册帧来源变化后同源仍只停车");
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        assert!(!follower.tracker.track_and_check(&c, &snapshot, &motion, [0.; 3], None).unwrap().recovery_samples.is_empty());
    }
    fn advance_to_motion(follower: &mut SurfaceFollower) -> Twist2 {
        for _ in 0..20 {
            next_sample(follower);
            std::thread::sleep(std::time::Duration::from_millis(1));
            let step = follower.step();
            let PlannerStep::Command(command) = step else {
                panic!("原生初始化后应交付运动：{step:?}");
            };
            if command != Twist2::ZERO {
                return command;
            }
        }
        panic!("一秒新采样内未生成可执行运动");
    }
    #[test]
    fn actual_scan_commands_fit_device_limits_after_reference_point_transform() {
        let (_,source)=crate::native_test_helpers::fixture();
        let mut local=follower(pose(0.5,1.5,0.3),source);
        let motion=local.config.motion_limits.as_mut().unwrap();
        motion.model=nuwax_nav_types_messages::MotionModel::Omnidirectional;
        motion.max_lateral_mps=0.5;
        local.config.command_limits=Some(nuwax_motion_safety_guard::wire::MotionCommandLimits {
            max_linear_x_mps:1.,max_linear_y_mps:0.2,max_angular_z_rps:1.,
            source:"Go2测试设备声明".into(),input_frame_id:"imu".into(),limit_frame_id:"base_link".into(),
            command_from_input_rotation:vec![1.,0.,0.,1.],input_origin_in_command_x_m:-0.0256,
            ..Default::default()
        });
        let mut moving=0;
        for _ in 0..40 {
            next_sample(&mut local);
            match local.step() {
                PlannerStep::Command(command)=>{
                    let x=command.linear.x;
                    let y=command.linear.y+0.0256*command.angular_z;
                    assert!(x.abs()<=1. && y.abs()<=0.2 && command.angular_z.abs()<=1.,"{command:?}");
                    moving+=usize::from(command!=Twist2::ZERO);
                    local.observe_applied_command(command);
                }
                other=>panic!("有效平地必须实际输出受约束命令：{other:?}"),
            }
        }
        assert!(moving>0,"不能用持续零速通过设备约束验收");
    }
    #[test]
    fn spatial_progress_does_not_cut_or_replace_author_a_full_reference() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        let reference=follower.reference.clone();
        follower.update_odometry(pose(2.5,1.,0.));
        assert!(follower.progress()>0.);
        assert_eq!(follower.reference,reference);
        assert_eq!(SurfaceFollowerConfig::new(0.4, 0.15).scan.planning_horizon,3.5);
    }
    #[test]
    fn profile_reverse_permission_never_invents_unknown_hardware_channels() {
        let config = SurfaceFollowerConfig::new(0.5, 0.15);
        assert!(config.kinematics().is_none());
        assert_eq!(config.scan.max_reverse_velocity, 0.);
        assert_eq!(config.scan.max_lateral_velocity, 0.);
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.configure_goal(&pose(1., 0., 0.), GoalOptions {
            allow_reverse: GoalOptions::resolve_reverse_request(false, true, true).unwrap(),
            ..GoalOptions::default()
        }).unwrap();
        assert!(follower.config.kinematics().is_none());
        assert_eq!(follower.config.scan.max_reverse_velocity, 0.);
        assert_eq!(follower.config.scan.max_lateral_velocity, 0.);
    }
    #[test]
    fn missing_ground_advances_without_creating_odometry_or_progress() {
        let mut follower = follower(pose(0.5, 1., 0.), unknown());
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        assert!(advance_to_motion(&mut follower).linear.x > 0.);
        let current = follower.odometry.as_ref().unwrap().clone();
        let plans = follower.local_plans();
        let mut previous = None;
        for _ in 0..3 {
            let PlannerStep::Command(command) = follower.step() else {
                panic!("缺测地面应允许原生局部执行");
            };
            assert!(command.linear.x > 0.);
            if let Some(old) = previous {
                assert_eq!(command, old, "同采样时刻重复计算不能推进控制时钟");
            }
            previous = Some(command);
            assert_eq!(
                follower.local_plans(),
                plans,
                "同一路线与采样的重复计算应沿用持续轨迹"
            );
            assert_eq!(follower.progress_m(), 0., "控制时钟不能充当实际移动");
            assert_eq!(
                follower
                    .refresh_execution_context(Twist2::ZERO)
                    .unwrap()
                    .odometry,
                current
            );
        }
        let mut moved = current;
        moved.pose.position.x = 0.7;
        moved.stamp.nanos += 400_000_000;
        follower.update_odometry(moved);
        assert!((follower.progress_m() - 0.2).abs() < 1e-9);
    }
    #[test]
    fn applied_command_never_becomes_measured_velocity_or_acceleration() {
        let mut follower = follower(pose(0.5,1.,0.),unknown());
        let applied=Twist2{linear:Point2::new(0.2,0.),angular_z:0.3};
        follower.observe_applied_command(applied);
        let _=follower.step();
        let boundary=follower.motion_boundary.as_ref().unwrap();
        assert_eq!(boundary.velocity,None);assert_eq!(boundary.yaw_rate,None);
        let context=follower.refresh_execution_context(applied).unwrap();
        assert_eq!(context.applied,applied);assert!(context.measured_motion.is_none());
        assert_eq!(follower.applied,applied);
    }
    #[test]
    fn measured_zero_and_motion_loss_recovery_stay_separate_from_nonzero_command() {
        let mut follower=follower(pose(0.5,1.,0.),unknown());
        follower.observe_applied_command(Twist2{linear:Point2::new(0.2,0.),angular_z:0.3});
        let mut measured=crate::motion_feedback::tests::measured([0.;3],[0.;3]);
        follower.observe_measured_motion(Some(measured.clone()));let _=follower.step();
        assert_eq!(follower.motion_boundary.as_ref().unwrap().velocity,Some([0.;3]));
        follower.observe_measured_motion(None);let _=follower.step();
        assert_eq!(follower.motion_boundary.as_ref().unwrap().velocity,None);
        measured.source_timestamp_ns+=1;measured.body_linear_xyz=[0.1,0.,0.];
        follower.observe_measured_motion(Some(measured));let _=follower.step();
        assert_eq!(follower.motion_boundary.as_ref().unwrap().velocity,Some([0.1,0.,0.]));
    }
    #[test]
    fn measured_loop_progress_preserves_author_full_reference() {
        let mut follower = follower(pose(0., 0., 0.), unknown());
        follower.config.scan.planning_horizon = 0.6;
        let goal = pose(1., -1., 0.);
        let points = [[0., 0.], [1., 0.], [1., 1.], [0., 1.], [0., 0.], [1., -1.]];
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: points.into_iter().map(|p| pose(p[0], p[1], 0.)).collect(),
                }),
                None,
                goal,
            )
            .unwrap();
        for (index, position) in [[1., 0.5], [0.5, 1.], [0., 0.5], [0., 0.]]
            .into_iter()
            .enumerate()
        {
            let mut actual = pose(position[0], position[1], 0.);
            actual.stamp.nanos += (index as i64 + 1) * 4_000_000_000;
            follower.update_odometry(actual);
        }
        let progress = follower.progress_m();
        let reference = follower.reference.clone();
        assert_eq!(progress, 4., "重叠路口的实测进度不能退回首段");
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        next_sample(&mut follower);
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        assert_eq!(follower.reference, reference, "实测进度不得切尾重建作者A的完整时间参考");
        assert_eq!(
            follower.progress_m(),
            progress,
            "计算局部目标不能推进真实进度"
        );
    }
    #[test]
    fn recent_obstacle_inside_real_body_rejects_the_actual_command() {
        let source = unknown();
        let source = Arc::new(
            source
                .with_recent_obstacle_points(1, 1, 0, &[], [(0, [0.5, 1., 0.5])])
                .unwrap(),
        );
        let mut follower = follower(pose(0.5, 1., 0.), source);
        assert!(matches!(follower.step(), PlannerStep::Command(Twist2::ZERO)|PlannerStep::Failed(_)));
    }
    #[test]
    fn moving_actual_pose_refreshes_the_static_window_before_entering_a_known_obstacle() {
        let (_, base) = crate::native_test_helpers::fixture();
        let points: Vec<_> = (0..20)
            .map(|z| (12.05_f32, 1.05_f32, z as f32 * 0.05))
            .collect();
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap(),
        );
        let mut follower = follower(pose(0.5, 1., 0.), map);
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        assert!(advance_to_motion(&mut follower).linear.x > 0.);
        let mut actual = pose(12.05, 1.05, 0.);
        actual.stamp.nanos += 30_000_000_000;
        follower.update_odometry(actual);
        assert!(
            matches!(follower.step(), PlannerStep::Command(Twist2::ZERO)|PlannerStep::Failed(_)),
            "地图内容不变时，真实位置进入新窗口仍须读到原有实体"
        );
    }
    #[test]
    fn initial_turn_uses_native_checked_command_and_never_moves_progress_by_time() {
        let mut follower = follower(pose(0.5, 1., std::f64::consts::FRAC_PI_2), unknown());
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        let command = advance_to_motion(&mut follower);
        assert!(command.angular_z < 0.);
        assert!(command.linear.x >= 0.);
        assert_eq!(command.linear.y, 0.);
        assert_eq!(follower.progress_m(), 0.);
    }
}

#[cfg(test)]
#[path="plan_track_tests.rs"]
mod plan_track_tests;
