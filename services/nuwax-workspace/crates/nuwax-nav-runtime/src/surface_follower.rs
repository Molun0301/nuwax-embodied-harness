//! 原生 SCAN 执行适配。位姿与实测运动独立输入，命令回执不充当测速。
use crate::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceSnapshot};
use nuwax_nav_astar::{local_planner::LocalPlannerConfig, navigation_map::CostMap2D};
use nuwax_nav_types_messages::{
    GoalOptions, MotionKinematics, MotionLimits, Path2D, Path3D, Point2, PoseStamped, Twist2,
};
use std::sync::Arc;

/// 跟随器与全局路线消费共用的位置容差口径。
/// 到达位置判定：非零容差时平面距离严格小于容差，高度差不超过台阶阈值与容差的较大者；
/// 台阶阈值取配置合成给出的 `step_threshold_m`。
pub fn position_within_tolerance(position: [f64; 3], goal: [f64; 3], tolerance: f64, map: &SurfaceSnapshot) -> bool {
    let step_height = f64::from(map.config().step_threshold_m);
    if tolerance == 0. {
        nuwax_nav_types_messages::same_finite_position(position, goal)
    } else {
        (position[0] - goal[0]).hypot(position[1] - goal[1]) < tolerance
            && (position[2] - goal[2]).abs() <= step_height.max(tolerance)
    }
}

pub(super) fn orientation_within_tolerance(pose: &PoseStamped, goal: &PoseStamped, options: GoalOptions) -> bool {
    if !options.requires_orientation() {
        true
    } else if options.yaw_tolerance_rad == 0. {
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
            < options.yaw_tolerance_rad
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
    /// 启动时选定的三维局部执行器。
    pub local_planner: nuwax_config::sections::SurfaceLocalPlanner,
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
    /// 终点转向或平移允许的连续无进展时间。
    pub terminal_stall_timeout_sec: f64,
}
impl SurfaceFollowerConfig {
    /// 创建物理标定明确的三维执行配置。
    pub fn new(body_to_support_z_m: f64, obstacle_clearance_m: f64) -> Self {
        Self {
            local_planner: Default::default(),
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
                min_executable_linear_mps: 0.,
                min_executable_angular_rps: 0.,
                terminal_translation_done_m: nuwax_nav_scan_native::TERMINAL_TRANSLATION_DONE_M,
                stopped_linear_mps: nuwax_nav_scan_native::STOPPED_LINEAR_MPS,
                stopped_yaw_rps: nuwax_nav_scan_native::STOPPED_YAW_RPS,
            },
            body_to_support_z_m,
            motion_limits: None,
            basic_kinematics: None,
            command_limits: None,
            max_observation_age_sec: None,
            controller: LocalPlannerConfig::default_config(),
            max_angular_speed_rps: 0.55,
            terminal_stall_timeout_sec: 5.,
        }
    }
    /// SCAN直接保留实际运动声明，不套用旧二维后端的前向策略。
    pub fn kinematics(&self) -> Option<MotionKinematics> {
        self.motion_limits
            .as_ref()
            .map(MotionLimits::kinematics)
            .or(self.basic_kinematics)
    }
    pub(super) fn native_config(&self, map: &SurfaceSnapshot, options: GoalOptions, lateral: bool) -> nuwax_nav_scan_native::Config {
        let mut config = self.scan;
        config.resolution = map.navigation_source_columns(None).0;
        config.body_to_ground = self.body_to_support_z_m;
        config.sample_dt = 1. / self.controller.control_frequency;
        config.max_velocity =
            self.controller.speed * self.controller.nerf_speed.min(1.);
        config.max_yaw_rate = self.max_angular_speed_rps;
        config.max_yaw_acceleration = config.max_yaw_rate / config.sample_dt;
        config.max_step_up = f64::from(map.config().step_threshold_m);
        config.max_step_down = config.max_step_up;
        config.max_slope = f64::from(map.config().max_slope_deg).to_radians();
        if let Some(motion) = self.kinematics() {
            config.min_executable_linear_mps = motion.min_executable_linear_mps;
            config.min_executable_angular_rps = motion.min_executable_angular_rps;
            config.max_velocity = config.max_velocity.min(motion.max_forward_mps);
            config.max_reverse_velocity = if options.allow_reverse {
                motion.max_reverse_mps.min(config.max_velocity)
            } else {
                0.
            };
            config.max_lateral_velocity = motion.max_lateral_mps.min(config.max_velocity);
            config.max_yaw_rate = config.max_yaw_rate.min(motion.max_angular_rps);
        }
        if let Some(limits) = &self.motion_limits {
            config.max_acceleration = limits
                .linear_acceleration_mps2
                .min(limits.guaranteed_braking_mps2);
            config.max_yaw_acceleration = limits
                .angular_acceleration_rps2
                .min(limits.guaranteed_angular_braking_rps2);
        }
        if !lateral {
            config.max_lateral_velocity = 0.;
        }
        if let Some(limits) = &self.command_limits {
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
            let bounds = crate::native_map::body_rectangle(body);
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
    boundary: Arc<Vec<[f64; 2]>>,
    options: GoalOptions,
    applied: Twist2,
    measured_motion: Option<crate::MeasuredMotion>,
    registered_frame: Option<Arc<crate::RegisteredFrame>>,
    motion_boundary: Option<crate::motion_feedback::MotionBoundary>,
    planning_velocity: [f64; 3],
    motion_sample_time: Option<i64>,
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
    last_local_failure: Option<String>,
    correction_replan_pending: bool,
    terminal_phase: TerminalPhase,
    terminal_approach: Option<TerminalApproach>,
    terminal_heading: f64,
    terminal_progress: Option<(f64, i64)>,
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
            boundary: Arc::new(Vec::new()),
            options: GoalOptions::default(),
            applied: Twist2::ZERO,
            measured_motion: None,
            registered_frame: None,
            motion_boundary: None,
            planning_velocity: [0.; 3],
            motion_sample_time: None,
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
            last_local_failure: None,
            correction_replan_pending: false,
            terminal_phase: TerminalPhase::Travel,
            terminal_approach: None,
            terminal_heading: 0.,
            terminal_progress: None,
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

    fn position(pose: &PoseStamped) -> [f64; 3] {
        let p = pose.pose.position;
        [p.x, p.y, p.z]
    }
    fn position_reached(&self) -> bool {
        let (Some(pose), Some(goal), Some(map)) = (&self.odometry, &self.goal, &self.map) else {
            return false;
        };
        position_within_tolerance(Self::position(pose), Self::position(goal),
            self.options.position_tolerance_m, map)
    }
    fn orientation_reached(&self) -> bool {
        let (Some(pose), Some(goal)) = (&self.odometry, &self.goal) else {
            return false;
        };
        orientation_within_tolerance(pose, goal, self.options)
    }
    fn reached(&self) -> bool {
        self.position_reached() && self.orientation_reached()
    }
    fn native_config(&self, map: &SurfaceSnapshot) -> nuwax_nav_scan_native::Config {
        self.config.native_config(map, self.options, true)
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
        self.last_local_failure = None;
        self.wait_reason = None;
        self.motion_sample_time = None;
        self.planning_velocity = [0.; 3];
        self.terminal_phase = TerminalPhase::Travel;
        self.terminal_approach = None;
        self.terminal_heading = 0.;
        self.terminal_progress = None;
    }
    pub(super) fn reference_to_stop(reference: &[[f64; 3]], stop: [f64; 3], half_width: f64) -> Vec<[f64; 3]> {
        let keep = reference.iter().position(|point|
            (point[0] - stop[0]).hypot(point[1] - stop[1]) <= half_width
        ).map_or(reference.len(), |index| index.max(1));
        reference[..keep].iter().copied().chain((!reference.is_empty()).then_some(stop)).collect()
    }
    fn prepare_terminal(&mut self, map: &PreparedMap, config: &nuwax_nav_scan_native::Config) {
        if self.terminal_phase != TerminalPhase::Travel || !self.options.requires_orientation()
            || self.terminal_approach.as_ref().is_some_and(|old| old.snapshot.same(&map.snapshot)) {
            return;
        }
        let goal = self.goal.as_ref().unwrap();
        let points = map.snapshot.terminal_points(config, Self::position(goal),
            goal.pose.orientation.yaw(), self.options.position_tolerance_m);
        let reference = Self::reference_to_stop(&self.reference, points.stop, config.body_width / 2.);
        self.needs_plan |= self.terminal_approach.as_ref().is_some_and(|old| *old.reference != reference);
        self.terminal_approach = Some(TerminalApproach { points, snapshot: map.snapshot.clone(), reference: Arc::new(reference) });
    }
    fn at_terminal_stop(&self, state: &MotionState) -> bool {
        let stop = self.terminal_approach.as_ref().map_or_else(
            || Self::position(self.goal.as_ref().unwrap()), |a| a.points.stop);
        if self.options.position_tolerance_m == 0. {
            self.position_reached()
        } else {
            (state.position[0] - stop[0]).hypot(state.position[1] - stop[1]) < self.options.position_tolerance_m
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
        self.terminal_progress = None;
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
            timestamp_ns: pose.stamp.nanos,
            measured_velocity: u32::from(boundary.velocity.is_some()),
            measured_yaw_rate: u32::from(boundary.yaw_rate.is_some()),
        };
        self.motion_boundary = Some(boundary);
        state
    }
    fn compute(&mut self) -> Result<Twist2, PlannerStep> {
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
                        self.last_local_failure = Some(reason.clone());
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
            let reason = "全局参考已结束，原业务目标未到，等待后续路线".to_owned();
            self.wait_reason = Some(reason.clone());
            return Err(PlannerStep::ReplanRequired(reason));
        }
        let Some(map) = self.prepared.clone() else {
            self.submit_map();
            return Ok(Twist2::ZERO);
        };
        let config = self.native_config(&map.source);
        self.prepare_terminal(&map, &config);
        let at_terminal_stop = self.at_terminal_stop(&state);
        if self.terminal_phase == TerminalPhase::Turning && !at_terminal_stop {
            self.terminal_phase = TerminalPhase::Travel;
            self.terminal_progress = None;
            self.needs_plan = true;
        } else if self.terminal_phase == TerminalPhase::Travel
            && at_terminal_stop && ((self.active.is_none() && self.pending_curve.is_none()) || state.stopped()) {
            self.begin_terminal_turn();
        }
        if self.terminal_phase == TerminalPhase::Turning && self.orientation_reached() {
            self.terminal_phase = TerminalPhase::Translating;
            self.terminal_progress = None;
        }
        let terminal_yaw = (self.terminal_phase != TerminalPhase::Travel)
            .then_some(self.terminal_heading);
        let stop = self.terminal_approach.as_ref().map_or_else(
            || Self::position(self.goal.as_ref().unwrap()), |a| a.points.stop);
        let terminal_position = (self.terminal_phase == TerminalPhase::Translating).then_some(stop);
        if terminal_yaw.is_some() {
            let (error, reason) = if self.terminal_phase == TerminalPhase::Turning {
                (nuwax_nav_astar::controllers::angle_diff(self.terminal_heading, state.yaw).abs(),
                    "终点转向受阻")
            } else {
                ((state.position[0] - stop[0]).hypot(state.position[1] - stop[1]), "终点平移受阻")
            };
            // 用实测位姿的来源时间计时，只有误差刷新最小值才算有进展。
            let progress = self.terminal_progress.get_or_insert((error, state.timestamp_ns));
            if error < progress.0 {
                *progress = (error, state.timestamp_ns);
            }
            if (state.timestamp_ns - progress.1) as f64 * 1e-9 >= self.config.terminal_stall_timeout_sec {
                return Err(PlannerStep::Failed(reason.into()));
            }
        }
        let mut tracking_failure = None;
        let goal_pose = self.goal.as_ref().unwrap();
        let q = goal_pose.pose.orientation;
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
            Some((Self::position(goal_pose), [q.x, q.y, q.z, q.w])),
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
        if checked.replan_early() && !self.early_requested && !at_terminal_stop {
            self.needs_plan = true;
            self.early_requested = true;
        }
        if checked.invalid() {
            self.active = None;
            self.active_snapshot = None;
            self.needs_plan = true;
        }
        // 已有计算时合并请求；这次计算交付后只由后续新事件再次请求。
        if self.needs_plan {
            if !self.planning && self.terminal_phase == TerminalPhase::Travel && !at_terminal_stop {
                self.plan_snapshot = Some(map.snapshot.clone());
                self.plan_worker.submit(PlanInput {
                    epoch: self.epoch,
                    map: map.clone(),
                    config,
                    reference: self.terminal_approach.as_ref().map_or_else(|| self.reference.clone(), |a| a.reference.clone()),
                    state,
                    active: self.active.clone(),
                    phase: self.phase,
                    goal_yaw: self.options.requires_orientation().then(|| self.goal.as_ref().unwrap().pose.orientation.yaw()),
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
        self.correction_replan_pending = false;
        self.options = options;
        self.path = None;
        self.reference = Arc::new(Vec::new());
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
        self.path = Some(path);
        self.correction_replan_pending = false;
        self.goal = Some(goal);
        self.submit_map();
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        self.odometry = Some(pose);
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
        if frame.is_none() && self.path.is_some() {
            self.correction_replan_pending = true;
        }
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
        match (&self.wait_reason, &self.last_local_failure) {
            (Some(wait), Some(failure)) if failure.starts_with("原参考路线未被接纳") =>
                Some(format!("{wait}；{failure}")),
            _ => self.wait_reason.clone(),
        }
    }
    fn terminal_phase(&self) -> TerminalPhase {
        self.terminal_phase
    }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        self.local_path.clone()
    }
    fn has_pending_replan(&self) -> bool { self.correction_replan_pending }
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
        if std::mem::take(&mut self.correction_replan_pending) {
            return PlannerStep::ReplanRequired("地图已校正，重新规划路线".into());
        }
        match self.compute() {
            Ok(command) => PlannerStep::Command(command),
            Err(step) => step,
        }
    }
    fn stop(&mut self) {
        self.retire_route();
        self.path = None;
        self.reference = Arc::new(Vec::new());
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
    fn go2_frontier_succeeds_in_tolerance_without_terminal_motion() {
        for (yaw, yaw_tolerance_rad) in [(-40_f64.to_radians(), std::f64::consts::PI), (2.82, 0.1)] {
            let (config, source) = crate::native_test_helpers::fixture();
            let mut follower = crate::native_test_helpers::author_static_map_follower(config);
            let start = pose(-0.404, -0.324, yaw);
            let goal = pose(-0.35, -0.25, 2.82);
            follower.configure_goal(&goal, GoalOptions {
                position_tolerance_m: 0.15,
                yaw_tolerance_rad,
                ..Default::default()
            }).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
            follower.update_odometry(start.clone());
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses: vec![start, goal.clone()],
            }), None, goal).unwrap();

            assert_eq!(follower.step(), PlannerStep::Succeeded);
            assert_eq!(follower.terminal_phase(), TerminalPhase::Travel);
        }
    }

    #[test]
    fn terminal_translation_times_out_from_last_distance_progress() {
        let (mut config, source) = crate::native_test_helpers::fixture();
        config.terminal_stall_timeout_sec = 2.;
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        let start = pose(1., 1., 0.);
        let mut goal = pose(1.1, 1., 0.);
        goal.pose.position.z += 0.3;
        follower.configure_goal(&goal, GoalOptions { position_tolerance_m: 0.15,
            yaw_tolerance_rad: std::f64::consts::PI, ..Default::default() }).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
        follower.update_odometry(start.clone());
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![start.clone(), goal.clone()],
        }), None, goal.clone()).unwrap();
        crate::native_test_helpers::prepare_terminal_map(&mut follower);
        assert_eq!(follower.terminal_phase(), TerminalPhase::Translating);

        let mut closer = start;
        closer.pose.position.x += 0.02;
        closer.stamp.nanos += 500_000_000;
        follower.update_odometry(closer.clone());
        for _ in 0..30 {
            assert!(matches!(follower.step(), PlannerStep::Command(_)));
        }
        closer.stamp.nanos += 1_999_999_999;
        follower.update_odometry(closer.clone());
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        closer.stamp.nanos += 1;
        follower.update_odometry(closer);
        assert_eq!(follower.step(), PlannerStep::Failed("终点平移受阻".into()));

        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        assert!(follower.terminal_progress.is_none());
    }

    #[test]
    fn terminal_turn_times_out_without_yaw_progress() {
        let (config, source) = crate::native_test_helpers::fixture();
        let mut follower = crate::native_test_helpers::author_static_map_follower(config);
        let mut start = pose(1., 1., 0.);
        let goal = pose(1.1, 1., 1.);
        follower.configure_goal(&goal, GoalOptions { position_tolerance_m: 0.15,
            yaw_tolerance_rad: 0.1, ..Default::default() }).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
        follower.update_odometry(start.clone());
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![start.clone(), goal.clone()],
        }), None, goal).unwrap();
        crate::native_test_helpers::prepare_terminal_map(&mut follower);
        assert_eq!(follower.terminal_phase(), TerminalPhase::Turning);

        start.stamp.nanos += 4_999_999_999;
        follower.update_odometry(start.clone());
        assert!(matches!(follower.step(), PlannerStep::Command(_)));
        start.stamp.nanos += 1;
        follower.update_odometry(start);
        assert_eq!(follower.step(), PlannerStep::Failed("终点转向受阻".into()));
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
    fn n4_unaccepted_reference_cause_is_kept_in_public_wait_diagnostics() {
        let (config, _) = crate::native_test_helpers::fixture();
        let mut follower = SurfaceFollower::new(config).unwrap();
        let rejected = "原参考路线未被接纳：Original final target is occupied";
        follower.last_local_failure = Some(rejected.into());
        follower.wait_reason = Some("等待局部轨迹".into());
        assert_eq!(follower.execution_wait_reason(), Some(format!("等待局部轨迹；{rejected}")));
        follower.wait_reason = None;
        assert_eq!(follower.execution_wait_reason(), None, "正常执行不显示等待原因");
        follower.configure_goal(&pose(3., 1., 0.), GoalOptions::default()).unwrap();
        follower.wait_reason = Some("等待局部轨迹".into());
        assert_eq!(follower.execution_wait_reason(), Some("等待局部轨迹".into()),
            "新目标不附带上一参考的拒绝原因");
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
            assert_eq!(
                follower
                    .refresh_execution_context(Twist2::ZERO)
                    .unwrap()
                    .odometry,
                current
            );
        }
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
    }
}

#[cfg(test)]
#[path="plan_track_tests.rs"]
mod plan_track_tests;
