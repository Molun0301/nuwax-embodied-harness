//! 原生 SCAN 执行适配。位姿与实测运动独立输入，命令回执不充当测速。
use crate::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceSnapshot};
use nuwax_nav_astar::{local_planner::LocalPlannerConfig, navigation_map::CostMap2D};
use nuwax_nav_types_messages::{
    GoalOptions, MotionKinematics, MotionLimits, Path2D, Path3D, Point2, PoseStamped,
    Twist2,
};
use std::sync::Arc;

/// 旧二维后端的前向兼容策略；原生 SCAN 不调用此函数。
pub fn forward_navigation_kinematics(mut motion: MotionKinematics) -> MotionKinematics {
    if motion.model==nuwax_nav_types_messages::MotionModel::Omnidirectional {
        motion.model=nuwax_nav_types_messages::MotionModel::Differential;
    }
    motion.max_reverse_mps=0.;motion.max_lateral_mps=0.;motion
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
    /// 感知拥有方配置的来源时效。
    pub max_observation_age_sec: Option<f64>,
    /// 原速度、周期和目标容差。
    pub controller: LocalPlannerConfig,
    /// 实际角速度上限。
    pub max_angular_speed_rps: f64,
    /// 导航实验台对照开关，默认开启；生产配置不提供关闭项。
    /// 关闭后直接跟随全局参考，控制器、实测障碍与实际机身检查不变。
    pub local_planning: bool,
}
impl SurfaceFollowerConfig {
    /// 创建物理标定明确的三维执行配置。
    pub fn new(body_to_support_z_m: f64) -> Self {
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
            },
            body_to_support_z_m,
            motion_limits: None,
            basic_kinematics: None,
            max_observation_age_sec: None,
            controller: LocalPlannerConfig::default_config(),
            max_angular_speed_rps: 0.55,
            local_planning: true,
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
/// 唯一三维局部计算拥有者，实际运动端仍属于调度器。
pub struct SurfaceFollower {
    config: SurfaceFollowerConfig,
    map: Option<Arc<SurfaceSnapshot>>,
    native_map: Option<crate::native_map::NativeMap>,
    native_window: Option<([i32; 2], u64)>,
    odometry: Option<PoseStamped>,
    goal: Option<PoseStamped>,
    path: Option<Path3D>,
    reference: Vec<[f64; 3]>,
    reference_start: usize,
    native_planner: nuwax_nav_scan_native::Planner,
    boundary: Vec<[f64; 2]>,
    options: GoalOptions,
    applied: Twist2,
    measured_motion: Option<crate::MeasuredMotion>,
    registered_frame: Option<Arc<crate::RegisteredFrame>>,
    ingested_frame: Option<crate::RegisteredFrameIdentity>,
    #[cfg(test)]
    frame_ingestion_count:usize,
    motion_dispatch_bound: Option<crate::CommandDispatchBound>,
    motion_boundary: Option<crate::motion_feedback::MotionBoundary>,
    motion_boundary_sources: Option<[nuwax_nav_scan_native::MotionBoundarySource;3]>,
    progress: f64,
    local_path: Vec<[f64; 3]>,
    local_body_path: Option<Path3D>,
    wait_reason: Option<String>,
    execution_cycle_complete:bool,
    execution_failed:bool,
}
impl SurfaceFollower {
    /// 仅供显式模拟/静态地图夹具使用作者 A 的静态占据输入。
    /// 生产默认关闭，不能用静态表面快照伪装注册点云原帧。
    pub fn enable_author_static_map_fixture(&mut self, enabled: bool) {
        self.native_planner.set_static_map_fixture(enabled);
    }
    /// 最近一次边界来源：实测、历史轨迹或首次边界假设，不与 APPLIED 混用。
    pub fn motion_boundary_sources(&self)->Option<[nuwax_nav_scan_native::MotionBoundarySource;3]> { self.motion_boundary_sources }
    /// 最近的测量来源和两个独立采样时点。
    pub fn measured_boundary(&self)->Option<&crate::motion_feedback::MotionBoundary> {self.motion_boundary.as_ref()}
    /// 构造纯计算后端。
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
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
        let mut native_planner = nuwax_nav_scan_native::Planner::new();
        native_planner.set_local_planning(config.local_planning);
        Ok(Self {
            config,
            map: None,
            native_map: None,
            native_window: None,
            odometry: None,
            goal: None,
            path: None,
            reference: Vec::new(),
            reference_start: 0,
            native_planner,
            boundary: Vec::new(),
            options: GoalOptions::default(),
            applied: Twist2::ZERO,
            measured_motion: None,
            registered_frame: None,
            ingested_frame: None,
            #[cfg(test)]
            frame_ingestion_count:0,
            motion_dispatch_bound: None,
            motion_boundary: None,
            motion_boundary_sources: None,
            progress: 0.,
            local_path: Vec::new(),
            local_body_path: None,
            wait_reason: None,
            execution_cycle_complete:false,
            execution_failed:false,
        })
    }
    /// 实际位置投影到全局参考的累计进度。
    pub fn progress_m(&self) -> f64 {
        self.progress
    }
    /// 已执行的原生局部规划次数。
    pub fn local_plans(&self) -> u64 {
        self.native_planner.plan_count()
    }
    /// 原生局部规划调用次数。
    pub fn local_planning_calls(&self) -> u64 {
        self.native_planner.plan_count()
    }
    /// 当前局部机体轨迹，供状态显示。
    pub fn local_path(&self) -> Option<&Path3D> {
        self.local_body_path.as_ref()
    }
    /// 当前局部机体轨迹的独立显示副本。
    pub fn local_body_path_for_display(&self) -> Option<Path3D> {
        self.local_body_path.clone()
    }
    /// 原接口中的路线进度观测。
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
        if self.options.position_tolerance_m == 0. {
            nuwax_nav_types_messages::same_finite_position(
                Self::position(pose),
                Self::position(goal),
            )
        } else {
            pose.pose.position.distance(goal.pose.position) < self.options.position_tolerance_m
        }
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
        self.position_reached() && yaw
    }
    fn computed_failure_step(&self, reason: String) -> PlannerStep {
        if self.execution_failed { PlannerStep::Failed(reason) }
        else { PlannerStep::ReplanRequired(reason) }
    }
    fn compute(&mut self, robot_width_m: f64) -> Result<Twist2, String> {
        self.update_progress();
        let map = self.map.as_ref().ok_or("等待局部感知")?;
        let pose = self.odometry.as_ref().ok_or("等待实际位姿")?;
        if self.reference.is_empty() {
            return Err("等待全局参考路线".into());
        }
        if !self.position_reached()
            && (self.reference.len() == 1
                || nuwax_nav_types_messages::same_finite_position(
                    Self::position(pose), *self.reference.last().unwrap()))
        {
            // 参考已无后续位移但原三维目标尚未到达；不能把零速等待当作到达。
            // 与作者正常周期结束一致，这不是可缓存的几何拒绝证据。
            self.execution_cycle_complete = true;
            self.local_path.clear();
            self.local_body_path = None;
            return Err("全局参考已结束，原业务目标未到，等待后续路线".into());
        }
        let mut config = self.config.scan;
        config.body_width = robot_width_m;
        config.body_length = robot_width_m;
        config.body_height = f64::from(map.config().robot_height);
        config.body_center_x = 0.;
        config.body_center_y = 0.;
        config.body_z_min = -self.config.body_to_support_z_m;
        config.body_z_max = config.body_height - self.config.body_to_support_z_m;
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
            config.max_reverse_velocity = if self.options.allow_reverse { motion.max_reverse_mps.min(config.max_velocity) } else { 0. };
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
        let yaw = pose.pose.orientation.yaw();
        let measured=self.measured_motion.as_ref().filter(|m|m.is_current());
        let boundary=crate::motion_feedback::boundary_for_pose(pose,measured);
        // 同步模拟状态在下一物理事件前有效；MAX 没有有限墙钟期限。
        self.motion_dispatch_bound=measured.filter(|m|m.valid_until_monotonic_ns<u64::MAX)
            .map(|m|crate::CommandDispatchBound::from_same_host_monotonic_deadline(m.valid_until_monotonic_ns));
        self.motion_boundary=Some(boundary.clone());
        let position = Self::position(pose);
        let radius = config.planning_horizon
            + (config.body_length * 0.5 + config.body_center_x.abs())
                .hypot(config.body_width * 0.5 + config.body_center_y.abs());
        let cell = [
            (position[0] / config.resolution).floor() as i32,
            (position[1] / config.resolution).floor() as i32,
        ];
        let key = (cell, radius.to_bits());
        if self.native_window != Some(key) {
            let center = cell.map(|value| (f64::from(value) + 0.5) * config.resolution);
            self.native_map = Some(crate::native_map::from_surface_window(
                map,
                Some((center, radius + config.resolution)),
            )?);
            self.native_window = Some(key);
        }
        let input = self.native_map.as_ref().expect("当前局部窗口已准备");
        let nearby = |p: &[f64; 3]| {
            (p[0] - position[0]).abs() <= radius && (p[1] - position[1]).abs() <= radius
        };
        let (recent_ground, recent_obstacles) = map.navigation_recent_samples();
        let obstacles: Vec<_> = input
            .obstacles
            .iter()
            .chain(recent_obstacles.iter())
            .filter(|p| nearby(p))
            .copied()
            .collect();
        let ground: Vec<_> = input
            .static_collision_ground
            .iter()
            .chain(recent_ground.iter())
            .filter(|p| nearby(p))
            .copied()
            .collect();
        if self.registered_frame.as_ref().is_some_and(|frame| !frame.is_current()) {
            self.native_planner.reset();
            self.ingested_frame=None;
            return Err("registered cloud source deadline expired".into());
        }
        // Execution retirement preserves the same-source probability map. Only
        // a real map-configuration rebuild permits re-ingesting its current frame.
        if self.native_planner.configure(&config)? {
            self.ingested_frame=None;
        }
        if let Some(frame)=self.registered_frame.as_ref() {
            if let Some(bound)=frame.dispatch_bound {
                self.motion_dispatch_bound=Some(self.motion_dispatch_bound.map_or(bound,|old|old.earliest(bound)));
            }
            if self.ingested_frame.as_ref()!=Some(&frame.identity) {
                self.native_planner.ingest_registered_frame(&config,&frame.points_world,
                    frame.sensor_origin_world,frame.body_position_world,frame.identity.timestamp_ns)?;
                tracing::debug!(target:"navigation.registered_frame", source=?frame.identity,
                    paired_odometry_sequence=frame.paired_odometry_sequence,
                    paired_odometry_timestamp_ns=frame.paired_odometry_timestamp_ns,
                    body_orientation_xyzw=?frame.body_orientation_xyzw,
                    "原始world注册点仅注入一次；同帧sensor origin和body pose，不二次变换");
                self.ingested_frame=Some(frame.identity.clone());
                #[cfg(test)] {self.frame_ingestion_count+=1;}
            }
        }
        let position_reached = self.position_reached();
        let result = self
            .native_planner
            .step(&nuwax_nav_scan_native::PlanRequest {
                obstacles: &obstacles,
                ground: &ground,
                forbidden_columns: &input.forbidden_columns,
                reference: if position_reached {
                    &[]
                } else {
                    // 原版 mode3 接收完整 PCT 路径；空间进度只供外层观测，
                    // 不能每拍切尾伪装成新路径而重建全局多项式。
                    &self.reference
                },
                boundary: &self.boundary,
                goal_yaw: (position_reached && self.options.requires_orientation())
                    .then(|| self.goal.as_ref().unwrap().pose.orientation.yaw()),
                start_position: Self::position(pose),
                start_velocity: boundary.velocity,
                // 没有可信加速度观测，历史接续由原 SCAN 显式处理。
                start_acceleration: None,
                start_yaw: yaw,
                start_yaw_rate: boundary.yaw_rate,
                odometry_timestamp_ns: pose.stamp.nanos,
                config,
            })?;
        if self.motion_boundary_sources != Some(result.motion_boundary_sources) {
            tracing::info!(target:"navigation.motion_feedback", source=?boundary.source_id,
                motion_timestamp_ns=?boundary.motion_timestamp_ns,pose_timestamp_ns=boundary.pose_timestamp_ns,
                boundary_sources=?result.motion_boundary_sources,
                "SCAN运动边界来源变化；测量和位姿为独立采样，回执不代表测速");
        }
        tracing::debug!(target:"navigation.motion_feedback", source=?boundary.source_id,
            motion_timestamp_ns=?boundary.motion_timestamp_ns,pose_timestamp_ns=boundary.pose_timestamp_ns,
            pose_clock=?boundary.pose_clock,sample_time_delta_ns=?boundary.sample_time_delta_ns,
            boundary_sources=?result.motion_boundary_sources,"SCAN运动输入来源与异步采样时间");
        self.motion_boundary_sources=Some(result.motion_boundary_sources);
        // A's exhausted emergency recovery is a terminal execution failure,
        // unlike a normally completed execution cycle needing a following route.
        if result.execution_failed && !position_reached {
            self.execution_failed=true;
            self.local_path.clear();
            self.local_body_path=None;
            return Err("作者A原生恢复重试已耗尽，执行失败并停止".into());
        }
        // A owns the EXEC→WAIT transition. The original business goal may
        // still need a following global route; this is not a geometry refusal.
        if result.execution_cycle_complete && !position_reached {
            self.execution_cycle_complete=true;
            self.local_path.clear();
            self.local_body_path=None;
            return Err("作者A本执行周期结束，原业务目标未到，等待后续路线".into());
        }
        self.local_path = result.samples.iter().map(|s| s.position).collect();
        self.local_body_path = self.path.as_ref().map(|path| Path3D {
            poses: result
                .samples
                .iter()
                .map(|sample| {
                    let mut point = pose.clone();
                    point.pose.position = nuwax_nav_types_messages::Point3::new(
                        sample.position[0],
                        sample.position[1],
                        sample.position[2],
                    );
                    point.pose.orientation =
                        nuwax_nav_types_messages::Quaternion::from_yaw(sample.yaw);
                    point
                })
                .collect(),
            ..path.clone()
        });
        Ok(Twist2 {
            linear: Point2::new(result.command_forward, result.command_lateral),
            angular_z: result.command_yaw_rate,
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
        options
            .validate()
            .map_err(|reason| RuntimeError::InvalidInput {
                field: "goal_options",
                reason,
            })?;
        crate::validate_map_pose(goal, "goal_pose")?;
        self.goal = Some(goal.clone());
        self.options = options;
        self.path = None;
        self.reference.clear();
        self.reference_start = 0;
        self.native_planner.reset_execution();
        self.execution_cycle_complete=false;
        self.execution_failed=false;
        // Keep the same-source map and its once-only fusion watermark.
        self.measured_motion=None;
        self.motion_boundary=None;
        self.motion_boundary_sources=None;
        self.motion_dispatch_bound=None;
        self.local_path.clear();
        self.local_body_path = None;
        self.progress = 0.;
        self.wait_reason = None;
        Ok(())
    }
    fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
        self.boundary = boundary.to_vec();
        // A business source event is independent of the latest odometry time.
        self.native_planner.notify_source_changed();
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
        self.reference = path.poses.iter().map(Self::position).collect();
        self.reference_start = 0;
        self.native_planner.reset_execution();
        self.execution_cycle_complete=false;
        self.execution_failed=false;
        // Keep the same-source map and its once-only fusion watermark.
        self.path = Some(path);
        self.goal = Some(goal);
        self.progress = 0.;
        self.update_progress();
        Ok(())
    }
    fn update_odometry(&mut self, odometry: PoseStamped) {
        self.odometry = Some(odometry);
        self.update_progress();
    }
    fn update_costmap(&mut self, _map: CostMap2D) {
        self.invalidate_execution_map();
    }
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        let source = match map {
            ExecutionMap::Surface(map) => map,
            ExecutionMap::SurfaceUpdate(update) => update.snapshot,
            ExecutionMap::Planar(_) => {
                return Err(RuntimeError::InvalidInput {
                    field: "execution_map",
                    reason: "三维执行需要三维感知".into(),
                })
            }
        };
        if self
            .map
            .as_ref()
            .is_none_or(|old| !crate::native_map::same_source_geometry(old, &source))
        {
            self.native_map = None;
            self.native_window = None;
        }
        self.map = Some(source);
        // Recheck the installed map/mask once even if no new pose has arrived.
        // Ordinary repeated compute calls do not generate this event.
        self.native_planner.notify_source_changed();
        Ok(())
    }
    fn invalidate_execution_map(&mut self) {
        self.observe_registered_frame(None);
        self.map = None;
        self.native_map = None;
        self.native_window = None;
        self.local_path.clear();
    }
    fn observe_applied_command(&mut self, command: Twist2) {
        self.applied = command;
    }
    fn observe_registered_frame(&mut self, frame: Option<Arc<crate::RegisteredFrame>>) {
        let same_domain=match (&self.registered_frame,&frame) {
            (Some(old),Some(new))=>old.identity.domain==new.identity.domain,
            (None,None)=>true,
            _=>false,
        };
        if !same_domain {
            self.native_planner.reset();
            self.ingested_frame=None;
        }
        // Duplicate identity is not a new sample and cannot refresh its deadline.
        if matches!((&self.registered_frame,&frame),(Some(old),Some(new)) if old.identity==new.identity) { return; }
        self.registered_frame=frame;
    }
    fn observe_measured_motion(&mut self, motion: Option<crate::MeasuredMotion>) {
        self.measured_motion=motion.filter(crate::MeasuredMotion::is_current);
    }
    fn take_command_dispatch_bound(&mut self)->Option<crate::CommandDispatchBound> { self.motion_dispatch_bound.take() }
    fn refresh_execution_context(
        &mut self,
        applied: Twist2,
    ) -> Option<Arc<crate::replan_feedback::ActualExecutionContext>> {
        Some(Arc::new(crate::replan_feedback::ActualExecutionContext {
            source: self.map.clone()?,
            odometry: self.odometry.clone()?,
            applied,
            measured_motion: self.measured_motion.clone().filter(crate::MeasuredMotion::is_current),
        }))
    }
    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        self.local_path.clone()
    }
    fn execution_cycle_complete(&self)->bool { self.execution_cycle_complete }
    fn step(&mut self, robot_width_m: f64) -> PlannerStep {
        self.execution_cycle_complete=false;
        self.execution_failed=false;
        self.motion_dispatch_bound=None;
        if self.reached() {
            return PlannerStep::Succeeded;
        }
        if self.map.is_none() || self.odometry.is_none() || self.reference.is_empty() {
            return PlannerStep::Pending(Vec::new());
        }
        match self.compute(robot_width_m) {
            Ok(command) => {
                self.wait_reason = None;
                PlannerStep::Command(command)
            }
            Err(reason) => {
                self.wait_reason = Some(reason.clone());
                self.computed_failure_step(reason)
            }
        }
    }
    fn stop(&mut self) {
        self.path = None;
        self.reference.clear();
        self.reference_start = 0;
        self.native_planner.reset_execution();
        self.execution_cycle_complete=false;
        self.execution_failed=false;
        // Keep the same-source map and its once-only fusion watermark.
        self.local_path.clear();
        self.local_body_path = None;
        self.progress = 0.;
        self.wait_reason = None;
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
    // 作者A按真实采样推进初始化；等待限于一秒，不能只重复同一采样让时钟前进。
    fn advance_to_motion(follower: &mut SurfaceFollower) -> Twist2 {
        for _ in 0..20 {
            next_sample(follower);
            let step = follower.step(0.3);
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
    fn production_static_surface_does_not_impersonate_a_registered_scan() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        follower.enable_author_static_map_fixture(false);
        let error=follower.compute(0.3).unwrap_err();
        assert!(error.contains("registered source frame"),"{error}");
        assert_eq!(follower.frame_ingestion_count,0);
    }
    #[test]
    fn registered_frame_injected_once_per_native_session_and_domain_change() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        follower.enable_author_static_map_fixture(false);
        let frame=crate::registered_frame::test_frame(1);
        follower.observe_registered_frame(Some(frame.clone()));
        let _=follower.compute(0.3); // This test asserts input delivery, not path feasibility.
        assert_eq!(follower.frame_ingestion_count,1);
        let _=follower.compute(0.3);
        assert_eq!(follower.frame_ingestion_count,1);
        assert_eq!(follower.registered_frame.as_ref().unwrap().points_world.as_ref(),frame.points_world.as_ref());
        let mut next=(*crate::registered_frame::test_frame(2)).clone();
        next.identity.domain.runtime_generation+=1;
        follower.observe_registered_frame(Some(Arc::new(next)));
        assert!(follower.ingested_frame.is_none());
        let _=follower.compute(0.3);
        assert_eq!(follower.frame_ingestion_count,2);
        follower.observe_registered_frame(None);
        assert!(follower.registered_frame.is_none());
        assert!(follower.compute(0.3).unwrap_err().contains("registered source frame"));
    }
    #[test]
    fn execution_goal_path_and_stop_preserve_same_frame_fusion_watermark() {
        let start=pose(0.5,1.,0.);let goal=pose(3.,1.,0.);
        let mut follower=follower(start.clone(),test_snapshot(0.,false));
        follower.enable_author_static_map_fixture(false);
        let frame=crate::registered_frame::test_frame(1);
        follower.observe_registered_frame(Some(frame.clone()));
        let _=follower.compute(0.3);
        assert_eq!(follower.frame_ingestion_count,1);
        let key=follower.ingested_frame.clone();
        for retire in 0..3 {
            match retire {
                0=>follower.configure_goal(&goal,GoalOptions::default()).unwrap(),
                1=>follower.stop(),
                _=>{},
            }
            assert_eq!(follower.ingested_frame,key,"execution retirement must not clear fusion watermark");
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses:vec![start.clone(),goal.clone()],
            }),None,goal.clone()).unwrap();
            assert_eq!(follower.ingested_frame,key);
            next_sample(&mut follower);
            let _=follower.compute(0.3);
            assert_eq!(follower.frame_ingestion_count,1,"a new goal/path must not hit the same scan twice");
            assert!(Arc::ptr_eq(follower.registered_frame.as_ref().unwrap(),&frame));
        }
    }
    #[test]
    fn map_configuration_rebuild_reingests_once_without_renewing_frame_deadline() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        follower.enable_author_static_map_fixture(false);
        let mut frame=(*crate::registered_frame::test_frame(1)).clone();
        let deadline=crate::motion_feedback::monotonic_ns()+5_000_000_000;
        frame.dispatch_bound=Some(crate::CommandDispatchBound::from_same_host_monotonic_deadline(deadline));
        let frame=Arc::new(frame);
        follower.observe_registered_frame(Some(frame.clone()));
        let _=follower.compute(0.3);assert_eq!(follower.frame_ingestion_count,1);
        // A real map geometry field, not a new observation or a new goal.
        follower.config.body_to_support_z_m+=0.01;
        next_sample(&mut follower);
        let error=follower.compute(0.3).unwrap_err();
        assert!(error.contains("unretired trajectory"),"{error}");
        assert_eq!(follower.frame_ingestion_count,1,"active curves cannot silently adopt new map geometry");
        follower.stop();
        let start=follower.odometry.as_ref().unwrap().clone();let goal=pose(3.,1.,0.);
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses:vec![start,goal.clone()],
        }),None,goal).unwrap();
        let _=follower.compute(0.3);assert_eq!(follower.frame_ingestion_count,2);
        let _=follower.compute(0.3);assert_eq!(follower.frame_ingestion_count,2);
        assert!(Arc::ptr_eq(follower.registered_frame.as_ref().unwrap(),&frame));
        assert_eq!(follower.registered_frame.as_ref().unwrap().dispatch_bound,frame.dispatch_bound);
        assert_eq!(follower.ingested_frame.as_ref(),Some(&frame.identity));
    }
    #[test]
    fn spatial_progress_does_not_cut_or_replace_author_a_full_reference() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        let reference=follower.reference.clone();
        follower.update_odometry(pose(2.5,1.,0.));
        assert!(follower.progress()>0.);
        assert_eq!(follower.reference,reference);
        assert_eq!(SurfaceFollowerConfig::new(0.4).scan.planning_horizon,3.5);
    }
    #[test]
    fn author_cycle_completion_cannot_override_arrival_or_survive_stop() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        follower.execution_cycle_complete=true;
        follower.update_odometry(pose(3.,1.,0.));
        assert!(matches!(follower.step(0.3),PlannerStep::Succeeded));
        assert!(!follower.execution_cycle_complete());
        follower.execution_cycle_complete=true;
        follower.stop();
        assert!(!follower.execution_cycle_complete());
        follower.update_odometry(pose(0.5,1.,0.));
        assert!(matches!(follower.step(0.3),PlannerStep::Pending(_)));
    }
    #[test]
    fn author_exhausted_recovery_maps_to_terminal_failure_not_following_route() {
        let mut follower=follower(pose(0.5,1.,0.),test_snapshot(0.,false));
        // Unit-level classification of the same-call native cause; native FSM
        // transition tests separately prove when the cause may be emitted.
        follower.execution_failed=true;
        assert!(matches!(follower.computed_failure_step("exhausted".into()),PlannerStep::Failed(_)));
        follower.execution_failed=false;
        assert!(matches!(follower.computed_failure_step("ordinary refusal".into()),PlannerStep::ReplanRequired(_)));
        follower.execution_failed=true;
        follower.update_odometry(pose(3.,1.,0.));
        assert!(matches!(follower.step(0.3),PlannerStep::Succeeded));
        assert!(!follower.execution_failed);
        follower.execution_failed=true;follower.stop();
        assert!(!follower.execution_failed);
        follower.update_odometry(pose(0.5,1.,0.));
        assert!(matches!(follower.step(0.3),PlannerStep::Pending(_)));
    }
    #[test]
    fn profile_reverse_permission_never_invents_unknown_hardware_channels() {
        let config = SurfaceFollowerConfig::new(0.5);
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
        assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
        assert!(advance_to_motion(&mut follower).linear.x > 0.);
        let current = follower.odometry.as_ref().unwrap().clone();
        let plans = follower.local_plans();
        let mut previous = None;
        for _ in 0..3 {
            let PlannerStep::Command(command) = follower.step(0.3) else {
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
        let _=follower.step(0.3);
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
        follower.observe_measured_motion(Some(measured.clone()));let _=follower.step(0.3);
        assert_eq!(follower.motion_boundary.as_ref().unwrap().velocity,Some([0.;3]));
        follower.observe_measured_motion(None);let _=follower.step(0.3);
        assert_eq!(follower.motion_boundary.as_ref().unwrap().velocity,None);
        measured.source_timestamp_ns+=1;measured.body_linear_xyz=[0.1,0.,0.];
        follower.observe_measured_motion(Some(measured));let _=follower.step(0.3);
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
        assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
        next_sample(&mut follower);
        assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
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
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(_)));
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
        assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
        assert!(advance_to_motion(&mut follower).linear.x > 0.);
        let mut actual = pose(12.05, 1.05, 0.);
        actual.stamp.nanos += 30_000_000_000;
        follower.update_odometry(actual);
        assert!(
            matches!(follower.step(0.3), PlannerStep::ReplanRequired(_)),
            "地图内容不变时，真实位置进入新窗口仍须读到原有实体"
        );
    }

    #[test]
    fn declared_height_without_full_envelope_keeps_a_real_low_ceiling_blocking() {
        let source = test_snapshot(0., false);
        assert!(source.body_envelope().is_none());
        let source = Arc::new(
            source
                .with_recent_obstacle_points(1, 1, 0, &[0], [(0, [0.5, 1., 0.68])])
                .unwrap(),
        );
        let mut follower = follower(pose(0.5, 1., 0.), source);
        assert!(
            matches!(follower.step(0.3), PlannerStep::ReplanRequired(_)),
            "原声明0.6米高度的机身顶达到0.7米，0.68米楼板不能因默认矮机身而放行"
        );
    }

    #[test]
    fn initial_turn_uses_native_checked_command_and_never_moves_progress_by_time() {
        let mut follower = follower(pose(0.5, 1., std::f64::consts::FRAC_PI_2), unknown());
        assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
        let command = advance_to_motion(&mut follower);
        assert!(command.angular_z < 0.);
        assert!(command.linear.x >= 0.);
        assert_eq!(command.linear.y, 0.);
        assert_eq!(follower.progress_m(), 0.);
    }
}
