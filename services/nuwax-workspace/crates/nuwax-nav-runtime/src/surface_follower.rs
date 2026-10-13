//! 三维表面路径的局部跟随器，只生成平面速度意图，不持有运动端。

use std::sync::Arc;
#[path="surface_body_control.rs"]
mod body_control;

use nuwax_nav_astar::controllers::{angle_diff, Controller, PController};
use nuwax_nav_astar::global_planner::GlobalPlannerConfig;
use nuwax_nav_astar::local_planner::LocalPlannerConfig;
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{MotionKinematics, MotionLimits, MotionModel, Path2D, Path3D, Point2, Point3, PoseStamped, Quaternion, Twist2};

use crate::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceSnapshot};

/// 从真实执行器声明派生自主导航/探索的前进与转向通道，不改写手动运动能力。
#[must_use]
pub fn forward_navigation_kinematics(mut motion: MotionKinematics) -> MotionKinematics {
    if motion.model == MotionModel::Omnidirectional {
        motion.model = MotionModel::Differential;
    }
    motion.max_reverse_mps = 0.0;
    motion.max_lateral_mps = 0.0;
    motion
}

/// 完整标定的前向副本供两个生产执行后端共用，保留原加减速和反应参数。
pub(crate) fn forward_navigation_limits(physical: &MotionLimits) -> MotionLimits {
    let motion = forward_navigation_kinematics(physical.kinematics());
    let mut limits = physical.clone();
    limits.model = motion.model;
    limits.max_reverse_mps = motion.max_reverse_mps;
    limits.max_lateral_mps = motion.max_lateral_mps;
    limits
}

/// 三维跟随必须明确标定，控制参数复用已有局部控制器配置。
#[derive(Debug, Clone)]
pub struct SurfaceFollowerConfig {
    /// Startup-only route refinement switch; collision, observation and braking checks stay enabled.
    pub local_planning_enabled: bool,
    /// 同一导航拥有方的全局地图与局部反馈缓存。
    pub pct: Arc<nuwax_nav_pct::PctPlanner>,
    /// 路线引导的近场轨迹参数，生产RouteFollower消费。
    pub scan: nuwax_nav_scan::ScanConfig,
    /// 机体局部负Z方向到支撑参考点的正距离，单位米；不提供猜测默认值。
    pub body_to_support_z_m: f64,
    /// 显式运动及保证制动标定。
    pub motion_limits: Option<nuwax_nav_types_messages::MotionLimits>,
    /// 执行器声明的基本速度通道；完整制动标定缺失时仍使用原低速几何检查。
    pub basic_kinematics: Option<nuwax_nav_types_messages::MotionKinematics>,
    /// 已配置的近期感知源年龄上界；不再授予或撤销逐地面格许可。
    pub max_observation_age_sec: Option<f64>,
    /// 已有速度、控制频率及位置/朝向容差。
    pub controller: LocalPlannerConfig,
    /// 独立的角速度命令上限，不能拿米/秒的线速度代替弧度/秒。
    pub max_angular_speed_rps: f64,
    /// 沿连续三维路线前进的前瞻距离，单位米。
    pub lookahead_m: f64,
    /// 机体允许偏离当前连续路径的最大距离，单位米。
    pub max_path_deviation_m: f64,
}

impl SurfaceFollowerConfig {
    /// 以明确物理标定创建配置，其余参数沿现有二维控制默认值。
    #[must_use]
    pub fn new(body_to_support_z_m: f64) -> Self {
        Self {
            local_planning_enabled: true,
            pct: Default::default(),
            scan: nuwax_nav_scan::ScanConfig::default(),
            body_to_support_z_m,
            motion_limits: None,
            basic_kinematics: None,
            max_observation_age_sec: None,
            controller: LocalPlannerConfig::default_config(),
            max_angular_speed_rps: 0.55,
            lookahead_m: 0.5,
            max_path_deviation_m: GlobalPlannerConfig::default().max_path_deviation,
        }
    }

    fn validate(&self) -> Result<(), RuntimeError> {
        // 先验证真实声明，策略收窄不能掩盖原始标定中无效的速度通道。
        if let Some(motion)=self.motion_limits.as_ref().map(|limits|limits.kinematics()).or(self.basic_kinematics) {
            motion.validate().map_err(|reason|RuntimeError::InvalidInput{field:"surface.kinematics",reason})?;
        }
        if let Some(limits)=&self.motion_limits {limits.validate().map_err(|reason|RuntimeError::InvalidInput{field:"surface.motion_limits",reason})?;}
        if self.max_observation_age_sec.is_some_and(|v|!v.is_finite() || v<=0.0){return Err(RuntimeError::InvalidInput {field:"surface.max_observation_age_sec",reason:"观察时效必须为有限正值".into()});}
        for (field, value) in [
            ("body_to_support_z_m", self.body_to_support_z_m),
            ("surface.speed", self.controller.speed),
            ("surface.max_angular_speed_rps", self.max_angular_speed_rps),
            (
                "surface.control_frequency",
                self.controller.control_frequency,
            ),
            ("surface.goal_tolerance", self.controller.goal_tolerance),
            (
                "surface.orientation_tolerance",
                self.controller.orientation_tolerance,
            ),
            ("surface.lookahead_m", self.lookahead_m),
            ("surface.max_path_deviation_m", self.max_path_deviation_m),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(RuntimeError::InvalidInput {
                    field,
                    reason: "必须提供有限正值".into(),
                });
            }
        }
        if !self.controller.nerf_speed.is_finite() || self.controller.nerf_speed <= 0.0 {
            return Err(RuntimeError::InvalidInput {
                field: "surface.nerf_speed",
                reason: "速度缩放必须是有限正值".into(),
            });
        }
        Ok(())
    }

    /// 完整标定优先，导航只使用从真实声明派生的前进与转向通道。
    pub fn kinematics(&self) -> Option<MotionKinematics> {
        self.motion_limits.as_ref().map(|limits|limits.kinematics()).or(self.basic_kinematics)
            .map(forward_navigation_kinematics)
    }
}

/// 跟随器的进度是三维累计弧长，跨地图修订不保存可复用的表面单元编号。
pub struct SurfaceFollower {
    boundary: Vec<[f64;2]>,
    config: SurfaceFollowerConfig,
    controller: PController,
    map: Option<Arc<SurfaceSnapshot>>,
    source_map: Option<Arc<SurfaceSnapshot>>,
    odometry: Option<PoseStamped>,
    path: Option<Path3D>,
    goal: Option<PoseStamped>,
    cumulative: Vec<f64>,
    progress_m: f64,
    last_tracked_pose: Option<PoseStamped>,
    invalid_input: Option<String>,
    path_index: usize,
    alignment_target_index: Option<usize>,
    options: nuwax_nav_types_messages::GoalOptions,
    last_command: Twist2,
    last_command_dispatch_bound: Option<crate::CommandDispatchBound>,
    selected_body_command: Option<body_control::SelectedBodyCommand>,
    braking_deadline: Option<std::time::Instant>,
    simulation_braking_deadline_ns: Option<i64>,
    control_target: Option<nuwax_nav_control::PlanarPose>,
    body_view_dirty: bool,
}

impl SurfaceFollower {
    /// 构造纯计算后端，不连接运动服务。
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
        config.validate()?;
        let speed = config.controller.speed * config.controller.nerf_speed.min(1.0);
        let mut controller = PController::new(speed, config.controller.control_frequency);
        controller.set_simulation(config.controller.simulation);
        Ok(Self {
            boundary: Vec::new(),
            config,
            controller,
            map: None,
            source_map: None,
            odometry: None,
            path: None,
            goal: None,
            cumulative: Vec::new(),
            progress_m: 0.0,
            last_tracked_pose: None,
            invalid_input: None,
            path_index: 1,
            alignment_target_index: None,
            options:Default::default(),
            last_command:Twist2::ZERO,
            last_command_dispatch_bound:None,
            selected_body_command:None,
            braking_deadline:None,
            simulation_braking_deadline_ns:None,
            control_target:None,
            body_view_dirty: true,
        })
    }

    /// Consume only the most recent successful nonzero step's observation-time
    /// qualification. This neither validates a newer pose nor grants a lease.
    pub(crate) fn take_command_dispatch_bound(&mut self) -> Option<crate::CommandDispatchBound> {
        self.last_command_dispatch_bound.take()
    }

    /// 当前累计三维进度，供状态诊断与行为回归使用。
    #[must_use]
    pub fn progress_m(&self) -> f64 {
        self.progress_m
    }

    fn refresh_current_body_view(&mut self) {
        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::CurrentViewRefresh);
        if !self.body_view_dirty { return; }
        self.body_view_dirty = false;
        self.map = self.source_map.clone();
        if let (Some(map), Some(pose)) = (self.source_map.as_ref(), self.odometry.as_ref()) {
            let p = pose.pose.position;
            let q = pose.pose.orientation;
            // 每次从环境快照派生；旧足迹清除不能随实际运动累积到未知区域。
            if let Ok(view) = map.with_current_body_support([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]) {
                self.map = Some(Arc::new(view));
            }
        }
    }

    /// 当前站姿自身被拒绝时，全局改道不能恢复运动；保留目标等待实际输入恢复。
    pub(crate) fn current_body_error(&mut self) -> Option<String> {
        self.refresh_current_body_view();
        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::CurrentBodyValidation);
        let map = self.map.as_ref()?;
        map.body_envelope()?;
        let pose = self.odometry.as_ref()?;
        let q = pose.pose.orientation;
        // A measured ground/pose contradiction cannot be repaired by replanning.
        // Unknown ground is not a collision or a per-cell support permission.
        let source=self.source_map.as_ref().unwrap_or(map);
        if matches!(source.body_ground_relation(xyz(pose.pose.position), [q.x,q.y,q.z,q.w]),
            Ok(nuwax_nav_mls_planner::surface::BodyGroundRelation::KnownIncompatible)) {
            return Some(nuwax_nav_mls_planner::surface::SurfaceError::MissingSupport.to_string());
        }
        let mut witness=None;
        map.validate_body_pose_with_witness(xyz(pose.pose.position), [q.x, q.y, q.z, q.w], &mut witness)
            .err().map(|error| match witness {
                Some(sample)=>format!("{error}; {sample}"),
                None=>error.to_string(),
            })
    }

    fn support(map: &SurfaceSnapshot, pose: &PoseStamped, offset: f64) -> Result<[f64; 3], String> {
        let q = pose.pose.orientation;
        map.locate_support_from_body(xyz(pose.pose.position), [q.x, q.y, q.z, q.w], offset)
            .map(|location| location.support_xyz)
            .map_err(|error| error.to_string())
    }

    fn admit_path(&mut self, path: Path3D, goal: PoseStamped) -> Result<(), RuntimeError> {
        self.last_command_dispatch_bound = None;
        self.refresh_current_body_view();
        if path.poses.is_empty() {
            return Err(rejected("三维路径为空"));
        }
        crate::validate_map_pose(&goal, "goal_pose")?;
        for pose in &path.poses {
            crate::validate_map_pose(pose, "path.payload")?;
        }
        let cumulative = cumulative_lengths(&path);
        if cumulative.iter().any(|length| !length.is_finite()) {
            return Err(rejected("三维路径弧长无效"));
        }
        let Some(map) = self.map.as_ref() else {
            return Err(rejected("三维路径缺少已消费表面快照"));
        };
        let Some(odometry) = self.odometry.as_ref() else {
            return Err(rejected("三维路径缺少机体位姿"));
        };
        crate::validate_map_pose(odometry, "odometry")?;
        // 完整规划属于后台MLS拥有方；调度器只验证当前接续与有界前向段，
        // 避免长路线的全图遍历阻塞优先取消和续租。
        let (progress, projected) = if path.poses[0].pose.position == odometry.pose.position {
            // 原始首点就是当前机体时，不让零弧长的旋转点或归一化余量改变接续站姿。
            (0.0, path.poses[0].clone())
        } else {
            closest_in_window(&path, &cumulative, odometry.pose.position, 0.0,
                self.config.max_path_deviation_m)
        };
        if projected.pose.position.distance(odometry.pose.position)
            > self.config.max_path_deviation_m
        {
            return Err(rejected("新路径首段不能与当前三维机体位置连续接续"));
        }
        if map.body_envelope().is_some() {
            let q = odometry.pose.orientation;
            let from = (xyz(odometry.pose.position), [q.x, q.y, q.z, q.w]);
            // 完整机体已沿实际地形验证；粗格中心连边不能否决精细层上的合法接续。
            if body_control::connection_endpoint(map, from, &projected).is_none() {
                return Err(rejected("新路径不能从实际机体站姿连续接续"));
            }
        } else {
            let current_support = Self::support(map, odometry, self.config.body_to_support_z_m)
                .map_err(rejected)?;
            let projected_support = Self::support(map, &projected, self.config.body_to_support_z_m)
                .map_err(rejected)?;
            map.validate_segment(current_support, projected_support)
                .map_err(|error| rejected(error.to_string()))?;
        }
        self.progress_m = progress;
        self.last_tracked_pose = Some(odometry.clone());
        self.path_index=1;
        self.alignment_target_index=None;
        self.path = Some(path);
        self.goal = Some(goal);
        self.cumulative = cumulative;
        self.invalid_input = None;
        self.controller.reset_errors();
        Ok(())
    }

    fn follow_step(&mut self, robot_width_m: f64) -> PlannerStep {
        if self.map.as_ref().is_some_and(|m|m.body_envelope().is_some()) {
            return self.follow_body_step(robot_width_m);
        }
        if let Some(reason) = self.invalid_input.as_ref() {
            return PlannerStep::Failed(reason.clone());
        }
        let (Some(map), Some(odometry), Some(path), Some(goal)) = (
            self.map.as_ref(),
            self.odometry.as_ref(),
            self.path.as_ref(),
            self.goal.as_ref(),
        ) else {
            return PlannerStep::ReplanRequired("三维路径、位姿或表面输入缺失".into());
        };
        if let Err(error) = crate::validate_map_pose(odometry, "odometry") {
            return PlannerStep::Failed(error.to_string());
        }
        if !robot_width_m.is_finite()
            || robot_width_m <= 0.0
            || f64::from(map.config().wall_clearance_m) + 1e-6 < robot_width_m * 0.5
        {
            return PlannerStep::Failed("表面图硬净空小于当前机器人半宽".into());
        }
        let position = odometry.pose.position;
        let current_support = match Self::support(map, odometry, self.config.body_to_support_z_m) {
            Ok(support) => support,
            Err(reason) => {
                return PlannerStep::ReplanRequired(format!("当前机体没有有效支撑: {reason}"))
            }
        };
        let displacement = self
            .last_tracked_pose
            .as_ref()
            .map_or(0.0, |previous| previous.pose.position.distance(position));
        if let Some(previous) = self.last_tracked_pose.as_ref() {
            let elapsed = ((odometry.stamp.nanos - previous.stamp.nanos).max(0) as f64 * 1e-9)
                .min(crate::DEFAULT_CONTROL_HORIZON.as_secs_f64());
            let plausible =
                self.config.controller.speed * elapsed + self.config.controller.goal_tolerance;
            if displacement > plausible {
                let horizontal_delta = (position.x - previous.pose.position.x)
                    .hypot(position.y - previous.pose.position.y);
                let vertical_delta = (position.z - previous.pose.position.z).abs();
                let step_limit = f64::from(map.config().step_threshold_m + map.config().voxel_size);
                let supported_step = horizontal_delta <= plausible
                    && vertical_delta <= step_limit
                    && Self::support(map, previous, self.config.body_to_support_z_m)
                        .is_ok_and(|from| map.validate_segment(from, current_support).is_ok());
                if !supported_step {
                    return PlannerStep::ReplanRequired(
                        "三维位姿跳变没有可确认的连续支撑，需要重新接续路线".into(),
                    );
                }
            }
        }

        let end = *self.cumulative.last().expect("已接纳的非空路径");
        let window_end =
            (self.progress_m + displacement + self.config.controller.goal_tolerance).min(end);
        let (progress, projected) = closest_in_window(
            path,
            &self.cumulative,
            position,
            self.progress_m,
            window_end,
        );
        if projected.pose.position.distance(position) > self.config.max_path_deviation_m {
            return PlannerStep::ReplanRequired("机体偏离当前连续三维路线".into());
        }
        self.progress_m = progress;
        self.last_tracked_pose = Some(odometry.clone());
        let endpoint = path.poses.last().expect("已接纳的非空路径");
        let at_path_end = end - progress <= self.config.controller.goal_tolerance
            && position.distance(endpoint.pose.position) < self.config.controller.goal_tolerance;
        if at_path_end {
            if endpoint.pose.position.distance(goal.pose.position)
                >= self.config.controller.goal_tolerance
                || position.distance(goal.pose.position) >= self.config.controller.goal_tolerance
            {
                return PlannerStep::ReplanRequired(
                    "安全截断路径已结束，原三维目标尚未到达".into(),
                );
            }
            let yaw_error =
                angle_diff(goal.pose.orientation.yaw(), odometry.pose.orientation.yaw());
            if !self.options.requires_orientation() || yaw_error.abs() < self.config.controller.orientation_tolerance {
                return PlannerStep::Succeeded;
            }
            let twist = self.controller.rotate(yaw_error);
            return self.bounded_command(twist);
        }
        // 前瞻不跨越未经过的折点，避免平面控制用直线切掉楼梯转角。
        let segment_end_index = self
            .cumulative
            .partition_point(|distance| *distance <= progress + 1e-9)
            .min(path.poses.len() - 1);
        let segment_end = self.cumulative[segment_end_index];
        let target_distance = (progress + self.config.lookahead_m).min(segment_end);
        let mut target = pose_at_distance(path, &self.cumulative, target_distance);
        let endpoint_pose = &path.poses[segment_end_index];
        let end_q = endpoint_pose.pose.orientation;
        let end_reference = match map.support_position_from_body(
            xyz(endpoint_pose.pose.position),
            [end_q.x, end_q.y, end_q.z, end_q.w],
            self.config.body_to_support_z_m,
        ) {
            Ok(reference) => reference,
            Err(error) => return PlannerStep::Failed(error.to_string()),
        };
        let remaining = segment_end - progress;
        let ratio = if remaining > f64::EPSILON {
            ((target_distance - progress) / remaining).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let sampled = match map.sample_segment_support(current_support, end_reference, ratio) {
            Ok(location) => location,
            Err(reason) => {
                return PlannerStep::ReplanRequired(format!("前方表面连接失效: {reason}"))
            }
        };
        let q = target.pose.orientation;
        let body_target = match map.body_position_from_support(
            sampled.support_xyz,
            [q.x, q.y, q.z, q.w],
            self.config.body_to_support_z_m,
        ) {
            Ok(point) => point,
            Err(reason) => {
                return PlannerStep::ReplanRequired(format!("前方机体目标没有有效支撑: {reason}"))
            }
        };
        target.pose.position = Point3::new(body_target[0], body_target[1], body_target[2]);
        let dx = target.pose.position.x - position.x;
        let dy = target.pose.position.y - position.y;
        if dx.hypot(dy) < 1e-6 {
            return PlannerStep::ReplanRequired(
                "路线要求原地改变高度，平面运动不能执行该连接".into(),
            );
        }
        let twist = self.controller.advance(
            (target.pose.position.x, target.pose.position.y),
            odometry.clone(),
        );
        self.bounded_command(twist)
    }

    fn bounded_command(&mut self, twist: Twist2) -> PlannerStep {
        self.bounded_command_with_speed_limit(twist,None)
    }

    fn bounded_command_with_speed_limit(&mut self, twist: Twist2, speed_limit: Option<f64>) -> PlannerStep {
        // Calibrated body braking is selected by the physical model, never by
        // an observation-age switch. Removing ground TTL cannot weaken this path.
        if self.config.motion_limits.is_some() && self.map.as_ref().is_some_and(|m|m.body_envelope().is_some()) {
            return self.bounded_body_command(twist,speed_limit);
        }
        // 原控制器最小速度不能突破三维执行显式配置的低速上限。
        let speed = self.config.controller.speed * self.config.controller.nerf_speed.min(1.0);
        let mut requested = twist;
        if twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.angular_z != 0.0
            && self.map.as_ref().is_some_and(|map| map.body_envelope().is_some())
        {
            if let Some((pose, target)) = self.odometry.as_ref().zip(self.control_target) {
                // 基础通道按已核验短时段完成剩余转角；不把角度直接当每秒速度，
                // 也不引入未标定的最低角速度。完整制动通道仍走上方动态窗口。
                let error = angle_diff(target.yaw, pose.pose.orientation.yaw());
                if error * twist.angular_z > 0.0 {
                    requested.angular_z = error / crate::DEFAULT_CONTROL_HORIZON.as_secs_f64();
                }
            }
        }
        let mut command = requested.clamp(speed, self.config.max_angular_speed_rps);
        if let Some(motion)=self.config.kinematics() {
            command.linear.x=command.linear.x.clamp(
                if self.options.allow_reverse {-motion.max_reverse_mps} else {0.0},motion.max_forward_mps);
            command.linear.y=command.linear.y.clamp(-motion.max_lateral_mps,motion.max_lateral_mps);
            command.angular_z=command.angular_z.clamp(-motion.max_angular_rps,motion.max_angular_rps);
            if motion.minimum_turning_radius_m>0.0 {
                let maximum=command.linear.x.hypot(command.linear.y)/motion.minimum_turning_radius_m;
                command.angular_z=command.angular_z.clamp(-maximum,maximum);
            } else if !motion.can_rotate_in_place && command.linear.x.hypot(command.linear.y)<=1e-9 {
                command.angular_z=0.0;
            }
            if command.linear.x.hypot(command.linear.y)<=1e-9 && command.angular_z.abs()<=1e-9
                && (twist.linear.x.hypot(twist.linear.y)>1e-9 || twist.angular_z.abs()>1e-9) {
                return PlannerStep::ReplanRequired("当前平面意图超出已声明的执行器通道，需要重新规划".into());
            }
        }
        if self.motion_is_traversable(command) {
            return PlannerStep::Command(command);
        }
        // 降低转弯半径，不能让原控制器的最小前进速度把机体带出有效净空。
        for scale in [0.5, 0.25, 0.125] {
            let candidate = Twist2 {
                linear: Point2::new(command.linear.x * scale, command.linear.y * scale),
                angular_z: command.angular_z,
            };
            if self.motion_is_traversable(candidate) {
                return PlannerStep::Command(candidate);
            }
        }
        let rotation = Twist2 {
            linear: Point2::new(0.0, 0.0),
            angular_z: command.angular_z,
        };
        let needs_rotation=self.rotation_can_restore_translation(command,command.angular_z);
        // 没有转向恢复通行的依据时不反复发微小角速度；必要的小角转向仍沿真实扫掠执行。
        if needs_rotation && self.config.kinematics().is_none_or(|m|m.can_rotate_in_place)
            && rotation.angular_z.abs() > 1e-9 && self.motion_is_traversable(rotation) {
            return PlannerStep::Command(rotation);
        }
        PlannerStep::ReplanRequired("短时速度轨迹没有连续支撑或足够净空".into())
    }

    /// 微转向只有能恢复后续平移时才属于接续动作，避免在已对齐堵点渐近旋转。
    pub(super) fn rotation_can_restore_translation(&self, command: Twist2, candidate_angular_z:f64) -> bool {
        self.odometry.as_ref().zip(self.control_target).is_none_or(|(pose,target)| {
            let yaw=pose.pose.orientation.yaw();
            let omni=self.config.kinematics().is_some_and(|m|m.model==nuwax_nav_types_messages::MotionModel::Omnidirectional && m.max_lateral_mps>0.0);
            let desired=if !omni && command.linear.x.hypot(command.linear.y)>1e-9 {
                (target.y-pose.pose.position.y).atan2(target.x-pose.pose.position.x)
                    + if command.linear.x<0.0 {std::f64::consts::PI} else {0.0}
            } else {target.yaw};
            let error=angle_diff(desired,yaw);
            if !candidate_angular_z.is_finite() {return false;}
            // 恢复依据属于这个候选的方向；另一个方向可能有路，不能授权反向微转。
            // 已有实际角速度的减速仍是必要制动，后续照常核验真实速度窗口与完整停车。
            let braking=candidate_angular_z*self.last_command.angular_z>=0.0
                && candidate_angular_z.abs()<self.last_command.angular_z.abs();
            if braking {return true;}
            if candidate_angular_z*error<=0.0 {return false;}
            if error.abs()>self.config.controller.orientation_tolerance.min(0.05).max(1e-6) {return true;}
            if error.abs()<=1e-6 {return false;}
            let Some(map)=self.map.as_ref() else {return false;};
            let distance=(target.x-pose.pose.position.x).hypot(target.y-pose.pose.position.y);
            let travel=(command.linear.x.hypot(command.linear.y)
                *crate::DEFAULT_CONTROL_HORIZON.as_secs_f64()).min(distance);
            if travel<=1e-9 || distance<=1e-9 {return false;}
            let q=pose.pose.orientation;
            let actual=(xyz(pose.pose.position),[q.x,q.y,q.z,q.w]);
            let mut aligned_target=pose.clone();aligned_target.pose.orientation=Quaternion::from_yaw(desired);
            let Some(aligned)=body_control::connection_endpoint(map,actual,&aligned_target) else {return false;};
            let mut next=aligned_target;
            next.pose.position.x=actual.0[0]+(target.x-actual.0[0])*travel/distance;
            next.pose.position.y=actual.0[1]+(target.y-actual.0[1])*travel/distance;
            // 必要小转向与后续平移都使用同一实际地形接续，不能借旧路径站姿取得许可。
            body_control::translation_connects(map,aligned,&next)
        })
    }

    fn motion_is_traversable(&self, command: Twist2) -> bool {
        let (Some(map), Some(pose)) = (self.map.as_ref(), self.odometry.as_ref()) else {
            return false;
        };
        if ![command.linear.x, command.linear.y, command.angular_z]
            .into_iter()
            .all(f64::is_finite)
        {
            return false;
        }
        if map.body_envelope().is_some() {
            return self.predicted_body_motion_is_traversable(command,crate::DEFAULT_CONTROL_HORIZON.as_secs_f64());
        }
        let Ok(initial_support) = Self::support(map, pose, self.config.body_to_support_z_m) else {
            return false;
        };
        let yaw = pose.pose.orientation.yaw();
        let horizon = crate::DEFAULT_CONTROL_HORIZON.as_secs_f64();
        let offset = [
            initial_support[0] - pose.pose.position.x,
            initial_support[1] - pose.pose.position.y,
        ];
        let travel = (command.linear.x.hypot(command.linear.y)
            + offset[0].hypot(offset[1]) * command.angular_z.abs())
            * horizon;
        let steps = (travel / (f64::from(map.config().voxel_size) * 0.5))
            .max(command.angular_z.abs() * horizon / 0.1)
            .ceil()
            .max(2.0);
        if !steps.is_finite() || steps > 512.0 {
            return false;
        }
        let mut support = initial_support;
        let mut delayed_support = initial_support;
        for sample in 1..=steps as usize {
            let time = horizon * sample as f64 / steps;
            let angle = command.angular_z * time;
            let (dx, dy) = if command.angular_z.abs() < 1e-8 {
                (
                    (command.linear.x * yaw.cos() - command.linear.y * yaw.sin()) * time,
                    (command.linear.x * yaw.sin() + command.linear.y * yaw.cos()) * time,
                )
            } else {
                (
                    (command.linear.x * ((yaw + angle).sin() - yaw.sin())
                        + command.linear.y * ((yaw + angle).cos() - yaw.cos()))
                        / command.angular_z,
                    (command.linear.x * (yaw.cos() - (yaw + angle).cos())
                        + command.linear.y * ((yaw + angle).sin() - yaw.sin()))
                        / command.angular_z,
                )
            };
            // 短时保持当前倾斜姿态，偏航同时旋转经完整姿态标定的水平支撑偏移。
            let desired = [
                pose.pose.position.x + dx + offset[0] * angle.cos() - offset[1] * angle.sin(),
                pose.pose.position.y + dy + offset[0] * angle.sin() + offset[1] * angle.cos(),
                support[2],
            ];
            let Ok(next) = map.sample_segment_support(support, desired, 1.0) else {
                return false;
            };
            // 不假定转向立即生效：同时检查保持当前航向的边界，以及两者间的支撑带。
            let delayed = [
                initial_support[0]
                    + (command.linear.x * yaw.cos() - command.linear.y * yaw.sin()) * time,
                initial_support[1]
                    + (command.linear.x * yaw.sin() + command.linear.y * yaw.cos()) * time,
                delayed_support[2],
            ];
            let Ok(next_delayed) = map.sample_segment_support(delayed_support, delayed, 1.0) else {
                return false;
            };
            if map
                .validate_segment(next.support_xyz, next_delayed.support_xyz)
                .is_err()
            {
                return false;
            }
            support = next.support_xyz;
            delayed_support = next_delayed.support_xyz;
        }
        true
    }
}

impl NavPlanner for SurfaceFollower {
    fn revalidate_selected_command(&mut self,command:Twist2)->Option<PlannerStep> {
        self.recheck_selected_body_command(command)
    }
    fn revalidate_selected_command_at_current_state(&mut self,command:Twist2)->Option<PlannerStep> {
        self.recheck_selected_body_command_at_current_state(command)
    }
    fn take_command_dispatch_bound(&mut self)->Option<crate::CommandDispatchBound> {SurfaceFollower::take_command_dispatch_bound(self)}
    fn configure_boundary(&mut self,boundary:&[[f64;2]]) {
        self.last_command_dispatch_bound = None;
        self.boundary=boundary.to_vec();
    }
    fn configure_goal(&mut self, _goal:&PoseStamped, options:nuwax_nav_types_messages::GoalOptions)->Result<(),RuntimeError> {
        self.last_command_dispatch_bound = None;
        options.validate().map_err(|reason|RuntimeError::InvalidInput {field:"goal_options",reason})?;
        // 新目标执行仍须取得守卫准入；旧规划器内部状态不能代替该证明。
        self.braking_deadline = None;
        self.last_command = Twist2::ZERO;
        // 目标授权是上限，旧目标允许倒车也不能恢复自主导航已禁用的运动方向。
        self.options=nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options};
        self.config.controller.goal_tolerance=options.position_tolerance_m.max(1e-6);
        self.config.controller.orientation_tolerance=options.yaw_tolerance_rad.max(1e-6);
        Ok(())
    }

    fn replace_path(&mut self, _path: Path2D) {
        self.stop();
        self.invalid_input = Some("三维后端拒绝二维路径".into());
    }

    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        self.last_command_dispatch_bound = None;
        match path {
            ExecutionPath::Surface(path) if yaws.is_none() => self.admit_path(path, goal),
            _ => Err(rejected(
                "三维后端必须接收完整三维路径，不能使用二维朝向来源",
            )),
        }
    }

    fn update_odometry(&mut self, odometry: PoseStamped) {
        self.last_command_dispatch_bound = None;
        // 新时间戳仍交给观察时效检查；几何完全相同的重复位姿不重建足迹。
        if self.odometry.as_ref().is_none_or(|previous|
            previous.pose != odometry.pose || previous.frame_id != odometry.frame_id)
        {
            self.body_view_dirty = true;
        }
        self.odometry = Some(odometry);
    }

    fn update_costmap(&mut self, _costmap: CostMap2D) {
        self.last_command_dispatch_bound = None;
        self.map = None;
        self.source_map = None;
        self.invalid_input = Some("三维后端拒绝二维占据图".into());
    }

    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        self.last_command_dispatch_bound = None;
        match map {
            ExecutionMap::Surface(map) | ExecutionMap::SurfaceUpdate(crate::surface_changes::SurfaceMapUpdate {snapshot:map,..}) => {
                if map.config().world_frame != "map" {
                    return Err(rejected("三维表面快照必须使用当前机体路径的map坐标系"));
                }
                if self.source_map.as_ref().is_none_or(|previous| !previous.same_body_query_view(&map)) {
                    self.map = Some(Arc::clone(&map));
                    self.body_view_dirty = true;
                }
                self.source_map = Some(map);
                Ok(())
            }
            ExecutionMap::Planar(_) => Err(rejected("三维后端不能用二维占据图恢复就绪")),
        }
    }

    fn invalidate_execution_map(&mut self) {
        self.stop();
        self.map = None;
        self.source_map = None;
    }

    fn observe_applied_command(&mut self, command:Twist2) {
        self.last_command_dispatch_bound = None;
        self.last_command=command;
    }

    fn step(&mut self, robot_width_m: f64) -> PlannerStep {
        self.last_command_dispatch_bound = None;
        self.refresh_current_body_view();
        self.follow_step(robot_width_m)
    }

    fn stop(&mut self) {
        self.begin_braking();
        self.path = None;
        self.goal = None;
        self.cumulative.clear();
        self.progress_m = 0.0;
        self.last_tracked_pose = None;
        self.invalid_input = None;
        self.controller.reset_errors();
    }
}

fn rejected(reason: impl Into<String>) -> RuntimeError {
    RuntimeError::PathRejected {
        reason: reason.into(),
    }
}
fn xyz(point: Point3) -> [f64; 3] {
    [point.x, point.y, point.z]
}

fn cumulative_lengths(path: &Path3D) -> Vec<f64> {
    let mut distances = Vec::with_capacity(path.poses.len());
    let mut total = 0.0;
    distances.push(total);
    for pair in path.poses.windows(2) {
        total += pair[0].pose.position.distance(pair[1].pose.position);
        distances.push(total);
    }
    distances
}

fn interpolate_pose(from: &PoseStamped, to: &PoseStamped, ratio: f64) -> PoseStamped {
    let mut pose = from.clone();
    let a = from.pose.position;
    let b = to.pose.position;
    pose.pose.position = Point3::new(
        a.x + (b.x - a.x) * ratio,
        a.y + (b.y - a.y) * ratio,
        a.z + (b.z - a.z) * ratio,
    );
    let a = normalized_quaternion(from.pose.orientation);
    let mut b = normalized_quaternion(to.pose.orientation);
    if a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>() < 0.0 {
        for value in &mut b {
            *value = -*value;
        }
    }
    let q: [f64; 4] = std::array::from_fn(|i| a[i] + (b[i] - a[i]) * ratio);
    let q = normalized_quaternion(Quaternion {
        x: q[0],
        y: q[1],
        z: q[2],
        w: q[3],
    });
    pose.pose.orientation = Quaternion {
        x: q[0],
        y: q[1],
        z: q[2],
        w: q[3],
    };
    pose
}

fn normalized_quaternion(q: Quaternion) -> [f64; 4] {
    let mut values = [q.x, q.y, q.z, q.w];
    let scale = values.iter().copied().map(f64::abs).fold(0.0, f64::max);
    for value in &mut values {
        *value /= scale;
    }
    let norm = values.iter().map(|v| v * v).sum::<f64>().sqrt();
    for value in &mut values {
        *value /= norm;
    }
    values
}

fn pose_at_distance(path: &Path3D, cumulative: &[f64], distance: f64) -> PoseStamped {
    let end = cumulative.partition_point(|value| *value <= distance);
    if end >= path.poses.len() {
        return path.poses.last().expect("非空路径").clone();
    }
    let start = end.saturating_sub(1);
    let length = cumulative[end] - cumulative[start];
    if length <= f64::EPSILON {
        return path.poses[end].clone();
    }
    interpolate_pose(
        &path.poses[start],
        &path.poses[end],
        ((distance - cumulative[start]) / length).clamp(0.0, 1.0),
    )
}

fn closest_in_window(
    path: &Path3D,
    cumulative: &[f64],
    position: Point3,
    begin: f64,
    end: f64,
) -> (f64, PoseStamped) {
    let mut best_s = begin;
    let mut best = pose_at_distance(path, cumulative, begin);
    let mut best_distance = best.pose.position.distance(position);
    let first_segment = cumulative
        .partition_point(|distance| *distance < begin)
        .saturating_sub(1);
    for index in first_segment..path.poses.len().saturating_sub(1) {
        let start_s = cumulative[index];
        if start_s > end {
            break;
        }
        let end_s = cumulative[index + 1];
        let length = end_s - start_s;
        if length <= f64::EPSILON || end_s < begin || start_s > end {
            continue;
        }
        let a = path.poses[index].pose.position;
        let b = path.poses[index + 1].pose.position;
        // 完整三维端点已经到达时吸收浮点余量，避免一直指向数值上同位置的折点。
        if b.distance(position) <= 1e-6 && end_s >= begin && end_s <= end + 1e-6 {
            best_s = end_s;
            best = path.poses[index + 1].clone();
            best_distance = b.distance(position);
            continue;
        }
        let delta = [b.x - a.x, b.y - a.y, b.z - a.z];
        let offset = [position.x - a.x, position.y - a.y, position.z - a.z];
        let ratio = (delta.iter().zip(offset).map(|(x, y)| x * y).sum::<f64>() / (length * length))
            .clamp(
                ((begin - start_s) / length).max(0.0),
                ((end - start_s) / length).min(1.0),
            );
        let candidate = interpolate_pose(&path.poses[index], &path.poses[index + 1], ratio);
        let distance = candidate.pose.position.distance(position);
        if distance < best_distance {
            best_distance = distance;
            best_s = start_s + ratio * length;
            best = candidate;
        }
    }
    (best_s, best)
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
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{GoalOptions, NavFrameId, Pose3, Timestamp};

    fn omni_route_follower(poses: Vec<PoseStamped>, calibrated: bool) -> (SurfaceFollower, Arc<SurfaceSnapshot>) {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        use nuwax_nav_types_messages::{MotionLimits, MotionModel};
        let map=Arc::new(test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,max_step_up_m:0.0,
            max_step_down_m:0.0,max_cross_slope_deg:10.0,support_uncertainty_m:0.0,
        }).unwrap());
        let limits=MotionLimits {
            model:MotionModel::Omnidirectional,max_forward_mps:1.0,max_reverse_mps:1.0,
            max_lateral_mps:1.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,
            guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        };
        let mut config=SurfaceFollowerConfig::new(0.3);
        config.controller.speed=1.0;config.controller.control_frequency=20.0;
        config.controller.simulation=true;config.max_angular_speed_rps=0.8;
        config.basic_kinematics=Some(limits.kinematics());
        if calibrated {config.motion_limits=Some(limits);}
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(poses.first().unwrap().clone());
        let goal=poses.last().unwrap().clone();
        follower.configure_goal(&goal,GoalOptions {
            allow_reverse:true,position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,
        }).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses}),None,goal).unwrap();
        (follower,map)
    }

    #[test]
    fn observation_dispatch_bound_is_single_use_and_lifecycle_scoped() {
        let anchor=crate::command_dispatch::ObservationClockAnchor {
            monotonic:std::time::Instant::now(),unix_ns:100,
        };
        let bound=crate::CommandDispatchBound::from_anchor(anchor,200);
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        follower.last_command_dispatch_bound=Some(bound);
        assert_eq!(follower.take_command_dispatch_bound(),Some(bound));
        assert_eq!(follower.take_command_dispatch_bound(),None);
        let pose=body_pose([1.0,1.0,0.4],Quaternion::IDENTITY,0);
        for operation in 0..8 {
            follower.last_command_dispatch_bound=Some(bound);
            match operation {
                0=>follower.configure_boundary(&[]),
                1=>{follower.configure_goal(&pose,GoalOptions::default()).unwrap();},
                2=>follower.update_odometry(pose.clone()),
                3=>follower.observe_applied_command(Twist2::ZERO),
                4=>{follower.update_execution_map(ExecutionMap::Surface(test_snapshot(0.0,false))).unwrap();},
                5=>{assert!(follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![]}),None,pose.clone()).is_err());},
                6=>follower.stop(),
                _=>{let _=follower.step(0.2);},
            }
            assert_eq!(follower.take_command_dispatch_bound(),None,"lifecycle operation {operation}");
        }
    }

    #[test]
    fn continuous_distance_keeps_height_and_degenerate_segment_endpoints() {
        let pose = body_pose([1.0,1.0,0.5],Quaternion::IDENTITY,0);
        let path = Path3D {poses:vec![pose.clone(),pose]};
        let point = Point3::new(1.0,1.0,2.0);
        let (progress, projected) = closest_in_window(&path,&[0.0,0.0],point,0.0,0.0);
        assert_eq!(progress,0.0);
        assert_eq!(projected.pose.position.distance(point),1.5);
    }

    #[test]
    fn forward_navigation_turns_then_reaches_rear_and_side_targets() {
        for calibrated in [false,true] {
            for (heading,prefix) in [(std::f64::consts::PI,true),(-2.6,true),
                (std::f64::consts::PI,false),(std::f64::consts::FRAC_PI_2,false)] {
                let mut current=body_pose([3.05,1.55,0.4],Quaternion::IDENTITY,0);
                let mut poses=vec![current.clone()];
                if prefix {
                    for index in 1..=20 {
                        poses.push(body_pose(xyz(current.pose.position),Quaternion::from_yaw(heading*index as f64/20.0),0));
                    }
                }
                let goal=body_pose([3.05+1.2*heading.cos(),1.55+1.2*heading.sin(),0.4],Quaternion::from_yaw(heading),0);
                poses.push(goal.clone());
                let (mut follower,map)=omni_route_follower(poses,calibrated);
                let mut applied=Twist2::ZERO;let mut reached=false;let mut translated=false;let mut finished=false;
                let mut max_cross_track=0.0_f64;let mut completed_ticks=0;
                for tick in 0..300 {
                    current.stamp.nanos=1_000_000_000+tick*50_000_000;
                    follower.update_odometry(current.clone());
                    let error=angle_diff(heading,current.pose.orientation.yaw());
                    let command=if reached {Twist2::ZERO} else {
                        match follower.step(0.3) {
                            PlannerStep::Command(command)=>command,
                            PlannerStep::Succeeded=>{reached=true;Twist2::ZERO},
                            other=>panic!("明确转向后前进不应丢失路线：完整标定{calibrated}，方向{heading}，{other:?}"),
                        }
                    };
                    let alignment_range=if prefix {0.25}else{0.6};
                    if error.abs()>alignment_range {
                        assert!(command.linear.x.hypot(command.linear.y)<1e-10,
                            "大转向前缀未完成时不可改成倒走或横移：{command:?}");
                    }
                    assert!(command.linear.x>=-1e-10,"前进路线不可被跟随器改成倒退：{command:?}");
                    assert_eq!(command.linear.y,0.0,"自主导航必须保持机体横向速度为零");
                    if !translated && command.linear.x.hypot(command.linear.y)>1e-8 {
                        translated=true;
                        assert!(command.linear.x>0.0 && error.abs()<=alignment_range,
                            "后方路线须先对齐到现有接续范围再向前行走");
                    }
                    // 独立按声明矢量加速度和实际回执积分，不把命令当已瞬时达到的速度。
                    let dx=command.linear.x-applied.linear.x;let dy=command.linear.y-applied.linear.y;
                    let ratio=(0.05/dx.hypot(dy).max(1e-12)).min(1.0);
                    let next_velocity=Twist2 {
                        linear:Point2::new(applied.linear.x+dx*ratio,applied.linear.y+dy*ratio),
                        angular_z:applied.angular_z+(command.angular_z-applied.angular_z).clamp(-0.1,0.1),
                    };
                    let mean=Twist2{linear:Point2::new((applied.linear.x+next_velocity.linear.x)*0.5,
                        (applied.linear.y+next_velocity.linear.y)*0.5),angular_z:(applied.angular_z+next_velocity.angular_z)*0.5};
                    let next=nuwax_nav_control::integrate(nuwax_nav_control::PlanarPose {
                        x:current.pose.position.x,y:current.pose.position.y,yaw:current.pose.orientation.yaw(),
                    },mean,0.05);
                    let q=Quaternion::from_yaw(next.yaw);
                    let advance=(next.x-current.pose.position.x)*heading.cos()
                        +(next.y-current.pose.position.y)*heading.sin();
                    assert!(advance>=-1e-9,"独立实际轨迹不能沿直线反复折返：{advance}");
                    max_cross_track=max_cross_track.max(((next.x-3.05)*heading.sin()
                        -(next.y-1.55)*heading.cos()).abs());
                    let from=(xyz(current.pose.position),[current.pose.orientation.x,current.pose.orientation.y,
                        current.pose.orientation.z,current.pose.orientation.w]);
                    let to=([next.x,next.y,current.pose.position.z],[q.x,q.y,q.z,q.w]);
                    map.with_current_body_support(from.0,from.1).unwrap().validate_body_motion(from,to,&[]).unwrap();
                    current.pose.position=Point3::new(to.0[0],to.0[1],to.0[2]);current.pose.orientation=q;
                    applied=next_velocity;follower.observe_applied_command(applied);
                    assert!(applied.linear.x>=0.0);assert_eq!(applied.linear.y,0.0);
                    if reached && applied.linear.x.hypot(applied.linear.y)<1e-12 && applied.angular_z.abs()<1e-12 {
                        finished=true;completed_ticks=tick+1;break;
                    }
                }
                assert!(finished && translated,"明确前进路线须闭环到达并停稳");
                assert!(current.pose.position.distance(goal.pose.position)<0.12);
                if prefix {assert!(max_cross_track<0.12,"明确直线前缀后不应横摆：{max_cross_track}");}
                eprintln!("完整标定{calibrated}，明确前缀{prefix}，目标朝向{heading:.3}：{}秒，实际最大横偏{max_cross_track:.6}米，零横移/倒退/折返",completed_ticks as f64*0.05);
            }
        }
    }

    #[test]
    fn large_turn_prefix_cannot_be_skipped_by_translation_endpoint_heading() {
        for calibrated in [false,true] {
            let start=body_pose([3.05,1.55,0.4],Quaternion::IDENTITY,0);
            let turn=body_pose(xyz(start.pose.position),Quaternion::from_yaw(std::f64::consts::PI),0);
            // 后续平移还包含姿态变化，不能用它的末朝向替代明确的原地转向终点。
            let end=body_pose([2.65,1.55,0.4],Quaternion::IDENTITY,0);
            let (mut follower,_)=omni_route_follower(vec![start,turn,end],calibrated);
            let PlannerStep::Command(command)=follower.step(0.3) else {panic!("可行的明确转向应开始执行")};
            assert!(command.linear.x.hypot(command.linear.y)<1e-10 && command.angular_z.abs()>0.0,
                "后续平移末端恰好与当前朝向相同，也不能跳过大转向并直接倒走：{command:?}");
        }
    }

    #[test]
    fn autonomous_side_and_rear_goals_turn_without_lateral_or_reverse_motion() {
        for calibrated in [false,true] {
            let start=body_pose([3.05,1.55,0.4],Quaternion::IDENTITY,0);
            for (direction,heading,prefix) in [([-1.0,0.0],0.0,false),([0.0,1.0],0.0,false),
                ([0.12_f64.cos(),0.12_f64.sin()],0.12,true)] {
                let mut poses=vec![start.clone()];
                if prefix {
                    poses.push(body_pose(xyz(start.pose.position),Quaternion::from_yaw(heading),0));
                    // 与生产完整采样路径一致，在有限前瞻内保留紧邻平移点。
                    poses.push(body_pose([start.pose.position.x+direction[0]*0.2,
                        start.pose.position.y+direction[1]*0.2,0.4],Quaternion::from_yaw(heading),0));
                }
                poses.push(body_pose([start.pose.position.x+direction[0],start.pose.position.y+direction[1],0.4],Quaternion::from_yaw(heading),0));
                let (mut follower,_)=omni_route_follower(poses,calibrated);
                let PlannerStep::Command(command)=follower.step(0.3) else {panic!("明确规划的可行平移必须保留")};
                assert!(!follower.options.allow_reverse,"旧目标倒退授权不能覆盖自主导航策略");
                assert_eq!(command.linear.y,0.0,"自主导航不能生成横移");
                assert!(command.linear.x>=0.0,"自主导航不能生成倒退");
                if prefix {
                    assert_eq!(command.linear.x,0.0,"显式小转向也应先完成：{command:?}");
                    assert!(command.angular_z>0.0);
                    follower.observe_applied_command(Twist2::ZERO);
                    follower.update_odometry(body_pose(xyz(start.pose.position),Quaternion::from_yaw(heading),1));
                    assert!(matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x>0.0 && c.linear.y==0.0),
                        "实际完成转向后应立即继续前向移动");
                } else {
                    assert_eq!(command.linear.x,0.0,"侧方或后方先原地转向");
                    assert!(command.angular_z.abs()>0.0,"必须朝新的前进方向转动");
                }
            }
        }
    }

    #[test]
    fn forward_navigation_policy_preserves_physical_calibration_and_curvature_limits() {
        let poses=vec![body_pose([2.05,1.05,0.4],Quaternion::IDENTITY,0),
            body_pose([3.05,1.05,0.4],Quaternion::IDENTITY,0)];
        let (follower,_)=omni_route_follower(poses,true);
        let physical=follower.config.motion_limits.as_ref().unwrap();
        assert_eq!(physical.model,MotionModel::Omnidirectional);
        assert_eq!((physical.max_reverse_mps,physical.max_lateral_mps),(1.0,1.0));
        let strategy=follower.config.kinematics().unwrap();
        assert_eq!(strategy.model,MotionModel::Differential);
        assert_eq!((strategy.max_reverse_mps,strategy.max_lateral_mps),(0.0,0.0));
        let mut expected=physical.clone();expected.model=MotionModel::Differential;
        expected.max_reverse_mps=0.0;expected.max_lateral_mps=0.0;
        assert_eq!(forward_navigation_limits(physical),expected,"响应、加速和制动标定必须原样保留");
        let car=MotionKinematics{model:MotionModel::Ackermann,can_rotate_in_place:false,
            minimum_turning_radius_m:0.8,max_lateral_mps:0.0,..physical.kinematics()};
        let forward_car=forward_navigation_kinematics(car);
        assert_eq!(forward_car.model,MotionModel::Ackermann);
        assert_eq!(forward_car.minimum_turning_radius_m,0.8);
        assert!(!forward_car.can_rotate_in_place);
        let mut invalid=follower.config.clone();invalid.motion_limits=None;
        invalid.basic_kinematics=Some(MotionKinematics{model:MotionModel::Differential,
            max_lateral_mps:1.0,..physical.kinematics()});
        assert!(SurfaceFollower::new(invalid).is_err(),"策略收窄不能掩盖无效物理声明");
    }

    fn apply_forward_feedback(map:&SurfaceSnapshot,pose:&mut PoseStamped,applied:&mut Twist2,command:Twist2) {
        assert!(command.linear.x>=0.0,"前向命令不能倒退：{command:?}");
        assert_eq!(command.linear.y,0.0,"前向命令不能横移");
        let next=Twist2{linear:Point2::new(command.linear.x.clamp(applied.linear.x-0.05,applied.linear.x+0.05),0.0),
            angular_z:command.angular_z.clamp(applied.angular_z-0.1,applied.angular_z+0.1)};
        let mean=Twist2{linear:Point2::new((applied.linear.x+next.linear.x)*0.5,0.0),
            angular_z:(applied.angular_z+next.angular_z)*0.5};
        let q=pose.pose.orientation;let yaw=q.yaw();
        let end_yaw=yaw+mean.angular_z*0.05;
        // 独立积分实际前进圆弧，不能用路径点直接替换实际位置。
        let (dx,dy)=if mean.angular_z.abs()<1e-12 {
            (mean.linear.x*yaw.cos()*0.05,mean.linear.x*yaw.sin()*0.05)
        } else {(mean.linear.x*(end_yaw.sin()-yaw.sin())/mean.angular_z,
            mean.linear.x*(yaw.cos()-end_yaw.cos())/mean.angular_z)};
        let from=(xyz(pose.pose.position),[q.x,q.y,q.z,q.w]);
        let q=Quaternion::from_yaw(end_yaw);
        let to=([from.0[0]+dx,from.0[1]+dy,from.0[2]],[q.x,q.y,q.z,q.w]);
        map.with_current_body_support(from.0,from.1).unwrap().validate_body_motion(from,to,&[]).unwrap();
        pose.pose.position=Point3::new(to.0[0],to.0[1],to.0[2]);pose.pose.orientation=q;*applied=next;
        assert!(applied.linear.x>=0.0);assert_eq!(applied.linear.y,0.0);
    }

    #[test]
    fn forward_navigation_follows_a_continuous_arc_and_cancels_with_real_braking() {
        for calibrated in [false,true] {
            let poses:Vec<_>=(0..=32).map(|index| {
                let angle=std::f64::consts::FRAC_PI_2*index as f64/32.0;
                body_pose([2.05+angle.sin(),1.55-angle.cos(),0.4],Quaternion::from_yaw(angle),0)
            }).collect();
            for cancel_at in [None,Some(20)] {
                let goal=poses.last().unwrap().clone();let mut pose=poses[0].clone();
                let (mut follower,map)=omni_route_follower(poses.clone(),calibrated);
                let mut applied=Twist2::ZERO;let mut completed=false;let mut stopping=false;
                let mut curved=0;let mut moving=0;let mut zero_between=0;let mut peak=0.0_f64;
                for tick in 0..400 {
                    pose.stamp.nanos=1_000_000_000+tick*50_000_000;
                    follower.update_odometry(pose.clone());
                    if cancel_at==Some(tick) {follower.stop();stopping=true;}
                    let command=if stopping {
                        match follower.step(0.3) {
                            PlannerStep::Command(command)=>{assert_eq!(command,Twist2::ZERO);command},
                            PlannerStep::ReplanRequired(_)=>Twist2::ZERO,
                            other=>panic!("取消后的旧路径不能恢复：{other:?}"),
                        }
                    } else {
                        match follower.step(0.3) {
                            PlannerStep::Command(command)=>command,
                            PlannerStep::Succeeded=>{stopping=true;follower.stop();Twist2::ZERO},
                            other=>panic!("前进圆弧应连续执行：完整标定{calibrated}，拍{tick}，{other:?}"),
                        }
                    };
                    if !stopping {
                        if command.linear.x>1e-8 {moving+=1;if command.angular_z.abs()>1e-8 {curved+=1;}}
                        else if moving>0 {zero_between+=1;}
                    }
                    apply_forward_feedback(&map,&mut pose,&mut applied,command);
                    peak=peak.max(applied.linear.x);follower.observe_applied_command(applied);
                    if stopping && applied.linear.x<1e-12 && applied.angular_z.abs()<1e-12 {
                        completed=true;
                        eprintln!("前向圆弧：完整标定{calibrated}，取消{cancel_at:?}，{}秒，峰速{peak:.3}，运动拍{moving}，曲线拍{curved}，中途零平移{zero_between}",(tick+1)as f64*0.05);
                        break;
                    }
                }
                assert!(completed,"真实反馈闭环必须有限时间内停稳");
                if cancel_at.is_none() {
                    assert!(pose.pose.position.distance(goal.pose.position)<0.12,"圆弧终点仍按原容差验收");
                    assert_eq!(zero_between,0,"连续可行圆弧不应每个采样点停转");
                    assert!(curved*2>moving,"必须实际同时前进转弯，不能用平移/原地转向拼接冒充圆弧");
                } else {
                    assert!(follower.path.is_none(),"取消后路径不能重新出现");
                }
            }
        }
    }

    #[test]
    fn forward_navigation_brakes_before_tight_arcs_and_keeps_driving() {
        for radius in [0.2,0.5] {
            let mut poses:Vec<_>=(0..=30).map(|i|
                body_pose([1.05+i as f64*0.05,0.55,0.4],Quaternion::IDENTITY,0)).collect();
            for i in 1..=32 {
                let angle=std::f64::consts::FRAC_PI_2*i as f64/32.0;
                poses.push(body_pose([2.55+radius*angle.sin(),0.55+radius*(1.0-angle.cos()),0.4],Quaternion::from_yaw(angle),0));
            }
            for i in 1..=20 {
                poses.push(body_pose([2.55+radius,0.55+radius+i as f64*0.05,0.4],Quaternion::from_yaw(std::f64::consts::FRAC_PI_2),0));
            }
            let goal=poses.last().unwrap().clone();let mut pose=poses[0].clone();
            let (mut follower,map)=omni_route_follower(poses,true);
            let mut applied=Twist2::ZERO;let mut reached=false;let mut completed=false;
            let mut entry_speed=None;let mut peak_before=0.0_f64;let mut stopped_in_arc=false;
            for tick in 0..500 {
                pose.stamp.nanos=1_000_000_000+tick*50_000_000;follower.update_odometry(pose.clone());
                let command=if reached {Twist2::ZERO} else {
                    match follower.step(0.3) {
                        PlannerStep::Command(command)=>command,
                        PlannerStep::Succeeded=>{reached=true;Twist2::ZERO},
                        other=>panic!("合法前向小圆弧不能中断：半径{radius}，拍{tick}，位置{:?}，朝向{}，目标{:?}，{other:?}",xyz(pose.pose.position),pose.pose.orientation.yaw(),follower.control_target),
                    }
                };
                apply_forward_feedback(&map,&mut pose,&mut applied,command);follower.observe_applied_command(applied);
                if pose.pose.position.x<2.55 {peak_before=peak_before.max(applied.linear.x);}
                else if entry_speed.is_none() {entry_speed=Some(applied.linear.x);}
                let angle=(pose.pose.position.x-2.55).atan2(0.55+radius-pose.pose.position.y);
                if angle>0.1 && angle<std::f64::consts::FRAC_PI_2-0.1 && applied.linear.x<1e-8 {stopped_in_arc=true;}
                if reached && applied.linear.x<1e-12 && applied.angular_z.abs()<1e-12 {
                    completed=true;
                    eprintln!("小圆弧半径{radius}：{}秒，直段峰速{peak_before:.3}，入弯速度{:.3}，原角速对应上限{:.3}",(tick+1)as f64*0.05,entry_speed.unwrap_or(0.0),radius*0.8);
                    break;
                }
            }
            assert!(completed && pose.pose.position.distance(goal.pose.position)<0.12);
            assert!(peak_before>0.8,"必须先真实巡航再验证提前减速，不能统一低速完成");
            assert!(entry_speed.unwrap()<=radius*0.8+1e-6,"入弯前应按路径曲率与原角速能力减速");
            assert!(!stopped_in_arc,"可执行相切小圆弧不能退化为中途停车转向");
        }
    }

    #[test]
    fn moving_current_body_view_does_not_accumulate_old_footprint_clearing() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        let mut points: Vec<_> = (0..40).flat_map(|x| (0..20).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        points.extend([(0.55,0.55,0.45),(2.55,0.55,0.45)]);
        let source=Arc::new(SurfaceSnapshot::from_points(
            &points,test_snapshot(0.0,false).config().clone(),0.1,
        ).unwrap().with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,
            max_step_up_m:0.15,max_step_down_m:0.15,max_cross_slope_deg:15.0,
            support_uncertainty_m:0.0,
        }).unwrap());
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&source))).unwrap();
        let first=[0.55,0.55,0.4];let next=[2.55,0.55,0.4];let q=[0.0,0.0,0.0,1.0];
        follower.update_odometry(body_pose(first,Quaternion::IDENTITY,1));
        follower.step(0.3);
        follower.map.as_ref().unwrap().validate_body_pose(first,q).unwrap();
        let first_view = Arc::clone(follower.map.as_ref().unwrap());
        follower.update_odometry(body_pose(first,Quaternion::IDENTITY,2));
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&source))).unwrap();
        follower.step(0.3);
        assert!(Arc::ptr_eq(&first_view, follower.map.as_ref().unwrap()),
            "同站姿新时间戳及同源地图心跳应复用足迹，不能每拍重建");
        let observations = Arc::new(nuwax_nav_mls_planner::SurfaceObservations::new(0.1, 32).unwrap());
        let changed = Arc::new(source.with_observations(Some(observations)));
        follower.update_execution_map(ExecutionMap::Surface(changed)).unwrap();
        follower.step(0.3);
        assert!(!Arc::ptr_eq(&first_view, follower.map.as_ref().unwrap()),
            "新观察必须撤销旧查询视图，不能只比较粗图几何");
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&source))).unwrap();
        follower.update_odometry(body_pose(next,Quaternion::IDENTITY,2));
        follower.step(0.3);
        let view=follower.map.as_ref().unwrap();
        view.validate_body_pose(next,q).unwrap();
        assert!(view.validate_body_pose(first,q).is_err(),"旧足迹外障碍必须恢复");
        assert!(source.validate_body_pose(first,q).is_err(),"原始环境不能被改写");
        assert!(Arc::ptr_eq(follower.source_map.as_ref().unwrap(),&source));
    }

    #[test]
    fn slow_translation_keeps_independent_in_place_turn_limit() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        let map=Arc::new(test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,max_step_up_m:0.0,
            max_step_down_m:0.0,max_cross_slope_deg:10.0,support_uncertainty_m:0.0,
        }).unwrap());
        let mut config=SurfaceFollowerConfig::new(0.3);
        config.controller.speed=0.05;config.max_angular_speed_rps=0.8;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        let start=body_pose([1.55,0.55,0.4],Quaternion::IDENTITY,1);
        let turn=body_pose([1.55,0.55,0.4],Quaternion::from_yaw(std::f64::consts::PI),2);
        let goal=body_pose([0.55,0.55,0.4],turn.pose.orientation,3);
        follower.update_odometry(start.clone());
        follower.configure_goal(&goal,nuwax_nav_types_messages::GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start,turn,goal.clone()]}),None,goal).unwrap();
        assert!(matches!(follower.step(0.3),PlannerStep::Command(v)
            if v.linear.x.abs()<1e-8 && v.angular_z.abs()>0.5 && v.angular_z.abs()<=0.8),
            "低速前进设置不能把后方目标的原地转向压到0.05弧度/秒");
        follower.config.max_angular_speed_rps=0.2;
        assert!(matches!(follower.step(0.3),PlannerStep::Command(v) if v.angular_z.abs()<=0.2));
    }

    #[test]
    fn localization_jitter_does_not_create_a_second_stop_gate() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        let map = Arc::new(test_snapshot(0.0, false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m: [-0.05,0.2], body_to_support_z_m:0.3,
            max_step_up_m:0.0, max_step_down_m:0.0, max_cross_slope_deg:10.0,
            support_uncertainty_m:0.0,
        }).unwrap());
        let mut config = SurfaceFollowerConfig::new(0.3);
        config.controller.speed = 0.15;
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        // 没有发过运动命令；定位微小校正的差分却远大于旧静止阈值。
        for tick in 0..20 {
            follower.update_odometry(body_pose([1.0 + if tick%2==0 {0.004} else {-0.004},0.55,0.4],
                Quaternion::from_yaw(if tick%2==0 {0.01} else {-0.01}),tick));
        }
        follower.stop();
        let start = follower.odometry.clone().unwrap();
        let goal = body_pose([1.5,0.55,0.4],Quaternion::IDENTITY,20);
        let options = nuwax_nav_types_messages::GoalOptions {position_tolerance_m:0.04,..Default::default()};
        follower.configure_goal(&goal,options).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start,goal.clone()]}),None,goal.clone()).unwrap();
        assert!(matches!(follower.step(0.3),PlannerStep::Command(v) if v.linear.x>0.0),
            "未运动时的地图校正不能把起步锁在零速");
        follower.update_odometry(goal);
        assert!(matches!(follower.step(0.3),PlannerStep::Succeeded),
            "到达只交给调度器发起停车，物理终态由运动守卫确认");
    }

    #[test]
    fn path_admission_uses_fine_body_support_across_a_coarse_step() {
        use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
        let mut points: Vec<_> = (-30..30).flat_map(|x| (-20..20).map(move |y|
            ((x as f32+0.5)*0.05, (y as f32+0.5)*0.05, -0.025))).collect();
        for z in [0.025, 0.075, 0.125] { points.push((0.075, 0.025, z)); }
        let mut cfg = test_snapshot(0.0, false).config().clone();
        cfg.wall_clearance_m=0.0; cfg.robot_height=0.25; cfg.step_threshold_m=0.21;
        cfg.min_step_tread_depth_m=0.2; cfg.max_slope_deg=25.0;
        let mut blocks=std::collections::BTreeMap::<[i32;3], std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation { observed_ns:0, hits:0 });
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks { observations.replace_block(key,values).unwrap(); }
        let map=Arc::new(SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m:vec![[-0.55,-0.14],[0.55,-0.14],[0.55,0.14],[-0.55,0.14]],
                z_bounds_m:[-0.1,0.25],body_to_support_z_m:0.4,max_step_up_m:0.2,
                max_step_down_m:0.1,max_cross_slope_deg:10.0,support_uncertainty_m:0.0,
            }).unwrap().with_observations(Some(Arc::new(observations))));
        let current=body_pose([0.025,-0.0001,0.4],Quaternion::IDENTITY,1);
        let first=body_pose([0.025,0.0001,0.4],Quaternion::IDENTITY,1);
        let from=(xyz(current.pose.position),[0.0,0.0,0.0,1.0]);
        let to=(xyz(first.pose.position),from.1);
        let a=map.locate_support_from_body(from.0,from.1,0.4).unwrap();
        let b=map.locate_support_from_body(to.0,to.1,0.4).unwrap();
        assert!(map.validate_segment(a.support_xyz,b.support_xyz).is_err(),
            "粗格中心的台阶来自另一精细半列，不能否决实际经过的低平地面");
        map.validate_body_motion(from,to,&[]).unwrap();
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.4)).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(current);
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses:vec![first.clone()],
        }),None,first).expect("路径从实际机体沿精细连续支撑接入");
    }

    #[test]
    fn short_velocity_prediction_does_not_lift_body_over_flat_ground_obstacle() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        let mut geometry=test_snapshot(0.0,false).config().clone();
        geometry.wall_clearance_m=0.0;geometry.robot_height=0.1;
        let mut points:Vec<_>=(0..40).flat_map(|x|(0..20).flat_map(move |y|
            [0.025,0.075,0.125].into_iter().map(move |z|((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,z)))).collect();
        points.push((1.65,0.55,0.45));
        let map=Arc::new(SurfaceSnapshot::from_points(&points,geometry,0.05).unwrap().with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.3,-0.12],[0.3,-0.12],[0.3,0.12],[-0.3,0.12]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,
            max_step_up_m:0.2,max_step_down_m:0.2,max_cross_slope_deg:15.0,support_uncertainty_m:0.0,
        }).unwrap());
        let start=body_pose([1.29,0.55,0.46],Quaternion::IDENTITY,1);
        let next=([1.415,0.55,0.46],[0.0,0.0,0.0,1.0]);
        map.validate_body_pose(xyz(start.pose.position),next.1).unwrap();
        assert!(map.validate_body_motion((xyz(start.pose.position),next.1),next,&[]).is_err(),
            "实际恒高轨迹确实会碰到低障碍");
        let support=map.locate_support_from_body(xyz(start.pose.position),next.1,0.4).unwrap();
        let raised=map.body_pose_on_surface([next.0[0],next.0[1]],0.0,support.support_xyz).unwrap();
        assert!(raised.0[2]>next.0[2]+0.05,"旧格顶投影会擅自抬高当前机体");
        let old_waypoint=body_pose([1.31,0.55,raised.0[2]],Quaternion::IDENTITY,1);
        assert!(map.validate_body_motion((xyz(start.pose.position),next.1),
            (xyz(old_waypoint.pose.position),next.1),&[]).is_ok(),
            "旧路径的抬高站姿可在几何上越过低障碍，必须独立验证实际地形接续");
        assert!(!body_control::translation_connects(&map,(xyz(start.pose.position),next.1),&old_waypoint),
            "局部不能提前消费逃逸点，再借旧路径末姿态抬高实际机体");
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.4)).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();follower.update_odometry(start);
        follower.refresh_current_body_view();
        assert!(!follower.motion_is_traversable(Twist2{linear:Point2::new(0.5,0.0),angular_z:0.0}),
            "基础短时预测不能凭空抬高机体获得通过");
    }

    #[test]
    fn calibrated_rotation_changes_direction_without_discarding_the_route() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        use nuwax_nav_types_messages::{MotionLimits,MotionModel,GoalOptions};
        let map=Arc::new(test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,
            max_step_up_m:0.0,max_step_down_m:0.0,max_cross_slope_deg:10.0,support_uncertainty_m:0.0,
        }).unwrap());
        let mut config=SurfaceFollowerConfig::new(0.3);
        config.controller.control_frequency=20.0;
        config.motion_limits=Some(MotionLimits {
            model:MotionModel::Differential,max_forward_mps:0.5,max_reverse_mps:0.3,
            max_lateral_mps:0.0,max_angular_rps:0.7,linear_acceleration_mps2:1.0,
            guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.1,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        });
        let mut follower=SurfaceFollower::new(config).unwrap();
        let start=body_pose([1.0,0.55,0.4],Quaternion::IDENTITY,1);
        let goal=body_pose([1.0,0.55,0.4],Quaternion::from_yaw(0.8),1);
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(start.clone());
        follower.configure_goal(&goal,GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start,goal.clone()]}),None,goal).unwrap();
        follower.observe_applied_command(Twist2 {linear:Point2::new(0.0,0.0),angular_z:-0.05});
        assert_eq!(follower.step(0.3),PlannerStep::Command(Twist2::ZERO),
            "换向时已验证的过零不能触发完整重规划");
        assert!(follower.path.is_some());
        assert!(follower.braking_deadline.is_some());
        follower.observe_applied_command(Twist2::ZERO);
        follower.braking_deadline=Some(std::time::Instant::now()-std::time::Duration::from_millis(1));
        assert!(matches!(follower.step(0.3),PlannerStep::Command(v) if v.angular_z>0.0),
            "制动结束应沿原路线开始反向转动");
    }

    #[test]
    fn simulation_braking_uses_input_time_and_current_confirmed_speed() {
        use nuwax_nav_types_messages::{MotionLimits,MotionModel};
        let mut config=SurfaceFollowerConfig::new(0.4);config.controller.control_frequency=20.0;
        config.motion_limits=Some(MotionLimits {
            model:MotionModel::Omnidirectional,max_forward_mps:1.0,max_reverse_mps:1.0,max_lateral_mps:1.0,
            max_angular_rps:0.8,linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
            angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        });
        for simulation in [false,true] {
            config.controller.simulation=simulation;
            let mut follower=SurfaceFollower::new(config.clone()).unwrap();
            follower.update_odometry(body_pose([0.0,0.0,0.5],Quaternion::IDENTITY,0));
            follower.observe_applied_command(Twist2{linear:Point2::new(0.1,0.0),angular_z:0.0});
            follower.begin_braking();
            if simulation {
                assert_eq!(follower.simulation_braking_deadline_ns,Some(1_150_000_000),
                    "0.1米/秒只按自身150毫秒停止时间等待，不能按最高速度等一秒");
                assert!(follower.braking_deadline.is_none());
                follower.stop();
                assert_eq!(follower.simulation_braking_deadline_ns,Some(1_150_000_000));
                follower.update_odometry(body_pose([0.0,0.0,0.5],Quaternion::IDENTITY,2));
                assert_eq!(follower.follow_body_step(0.3),PlannerStep::Command(Twist2::ZERO));
                follower.update_odometry(body_pose([0.0,0.0,0.5],Quaternion::IDENTITY,4));
                assert!(!matches!(follower.follow_body_step(0.3),PlannerStep::Command(_)),
                    "模拟时间到期应回到正常输入检查，不依赖墙钟等待");
            } else {
                assert!(follower.simulation_braking_deadline_ns.is_none());
                follower.update_odometry(body_pose([0.0,0.0,0.5],Quaternion::IDENTITY,400));
                assert_eq!(follower.follow_body_step(0.3),PlannerStep::Command(Twist2::ZERO),
                    "生产制动等待不能被输入时间跳变提前结束");
            }
        }
    }

    #[test]
    fn simulation_flag_does_not_boost_complete_body_rotation_commands() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
        use nuwax_nav_types_messages::GoalOptions;
        let map=Arc::new(test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,
            max_step_up_m:0.0,max_step_down_m:0.0,max_cross_slope_deg:10.0,support_uncertainty_m:0.0,
        }).unwrap());
        let mut results=Vec::new();
        for simulation in [false,true] {
            let mut config=SurfaceFollowerConfig::new(0.3);config.controller.simulation=simulation;
            let mut follower=SurfaceFollower::new(config).unwrap();
            let start=body_pose([1.0,0.55,0.4],Quaternion::IDENTITY,1);
            let goal=body_pose([1.0,0.55,0.4],Quaternion::from_yaw(0.1),1);
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
            follower.update_odometry(start.clone());
            follower.configure_goal(&goal,GoalOptions{yaw_tolerance_rad:0.01,..Default::default()}).unwrap();
            follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![start,goal.clone()]}),None,goal).unwrap();
            results.push(follower.step(0.3));
        }
        assert_eq!(results[0],results[1],"完整机体模拟只换时钟，不采用旧PController的0.8弧度/秒补偿");
        assert!(matches!(results[0],PlannerStep::Command(v)
            if (v.angular_z * crate::DEFAULT_CONTROL_HORIZON.as_secs_f64() - 0.1).abs() < 1e-9),
            "剩余转角应在已核验窗口内完成，不依赖模拟开关或固定最低角速度");
    }

    #[test]
    fn old_ground_remains_usable_but_missing_support_brakes_the_complete_tail() {
        use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
        use nuwax_nav_types_messages::{MotionLimits, MotionModel};
        use std::collections::BTreeMap;
        let base = test_snapshot(0.0, false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m: [-0.05, 0.2], body_to_support_z_m: 0.3,
            max_step_up_m: 0.0, max_step_down_m: 0.0,
            max_cross_slope_deg: 10.0, support_uncertainty_m: 0.0,
        }).unwrap();
        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
        for (fresh,missing_support,age) in [(true,false,Some(1.0)),(false,false,Some(1.0)),
            (false,false,None),(false,true,Some(1.0)),(false,true,None)] {
            let mut blocks: BTreeMap<_, BTreeMap<_, _>> = BTreeMap::new();
            for (x,y,z) in base.cell_keys() {
                let key = [x,y,z];
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                    SurfaceObservation { observed_ns: if fresh || x < 12 {now} else {1}, hits: 2 });
            }
            let mut observations = SurfaceObservations::new(base.source_voxel_size_m(), 32).unwrap();
            for (key, block) in blocks { observations.replace_block(key,block).unwrap(); }
            let geometry=if missing_support {
                test_snapshot(0.0,true).with_body_envelope(base.body_envelope().unwrap().clone()).unwrap()
            } else {base.clone()};
            let map = Arc::new(geometry.with_observations(Some(Arc::new(observations))));
            let mut config = SurfaceFollowerConfig::new(0.3);
            config.max_observation_age_sec = age;
            config.motion_limits = Some(MotionLimits {
                model: MotionModel::Differential, max_forward_mps: 0.5, max_reverse_mps: 0.3,
                max_lateral_mps: 0.0, max_angular_rps: 0.7, linear_acceleration_mps2: 1.0,
                guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
                guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.1,
                minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
            });
            let mut follower = SurfaceFollower::new(config).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
            follower.update_odometry(body_pose([1.0, 0.55, 0.4], Quaternion::IDENTITY, 0));
            let speed = Twist2 { linear: Point2::new(0.5, 0.0), angular_z: 0.0 };
            follower.last_command = speed;
            follower.control_target = Some(nuwax_nav_control::PlanarPose { x: 2.0, y: 0.55, yaw: 0.0 });
            let result = follower.bounded_command(speed);
            if !missing_support { assert!(matches!(result, PlannerStep::Command(v) if v.linear.x > 0.0), "old ground={}: {result:?}",!fresh); }
            else {
                // Missing support remains a geometric failure; the physical stop interval is retained.
                assert!(matches!(result, PlannerStep::ReplanRequired(_)), "{result:?}");
                assert_eq!(follower.last_command, Twist2::ZERO);
                let deadline = follower.braking_deadline.expect("已运动后须保留制动上界");
                follower.stop();
                assert_eq!(follower.braking_deadline,Some(deadline),"重复停止不能延长等待");
                assert!(matches!(follower.follow_body_step(0.3),PlannerStep::Command(v) if v==Twist2::ZERO));
                follower.braking_deadline=Some(std::time::Instant::now()-std::time::Duration::from_millis(1));
                assert!(!matches!(follower.follow_body_step(0.3),PlannerStep::Command(_)),"期限结束须回到输入/路径检查");
            }
        }
    }

    #[test]
    fn unsafe_turning_arc_is_limited_before_leaving_clear_surface() {
        let mut follower = SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(test_snapshot(0.0, false)))
            .unwrap();
        follower.update_odometry(body_pose(
            [2.0, 0.205, 0.4],
            Quaternion::from_yaw(-std::f64::consts::FRAC_PI_2),
            0,
        ));
        let unsafe_arc = Twist2 {
            linear: Point2::new(0.55, 0.0),
            angular_z: 0.55,
        };
        assert!(!follower.motion_is_traversable(unsafe_arc));
        let PlannerStep::Command(accepted) = follower.bounded_command(unsafe_arc) else {
            panic!("原地转向应可行");
        };
        assert_eq!(accepted.linear, Point2::new(0.0, 0.0));
        assert!(accepted.angular_z > 0.0);
        assert!(follower.motion_is_traversable(accepted));
    }

    #[test]
    fn near_edge_heading_change_closes_without_entering_unclear_cells() {
        let map = test_snapshot(0.0, false);
        let mut follower = SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        let mut position = [1.0, 0.25, 0.4];
        let mut yaw = -std::f64::consts::FRAC_PI_2;
        let start = body_pose(position, Quaternion::from_yaw(yaw), 0);
        let goal = body_pose([4.5, 0.25, 0.4], Quaternion::IDENTITY, 0);
        follower
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
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
        let mut reached = false;
        for tick in 1..=600 {
            follower.update_odometry(body_pose(position, Quaternion::from_yaw(yaw), tick));
            match follower.step(0.3) {
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                PlannerStep::Command(command) => {
                    position[0] +=
                        (command.linear.x * yaw.cos() - command.linear.y * yaw.sin()) * 0.05;
                    position[1] +=
                        (command.linear.x * yaw.sin() + command.linear.y * yaw.cos()) * 0.05;
                    yaw += command.angular_z * 0.05;
                    assert!(
                        map.locate_support([position[0], position[1], 0.1]).is_ok(),
                        "实际运动离开已验证净空: {position:?}"
                    );
                }
                other => panic!("平地近边缘路线不应失效: {other:?}"),
            }
        }
        assert!(reached, "限制圆弧后仍须完成目标");
    }

    fn body_pose(position: [f64; 3], orientation: Quaternion, tick: i64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp {
                nanos: 1_000_000_000 + tick * 50_000_000,
                ..Timestamp::default()
            },
            pose: Pose3 {
                position: Point3::new(position[0], position[1], position[2]),
                orientation,
            },
        }
    }

    fn follower_config() -> SurfaceFollowerConfig {
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.speed = 0.2;
        config.controller.control_frequency = 20.0;
        config.controller.goal_tolerance = 0.12;
        config.controller.orientation_tolerance = 0.15;
        config
    }

    #[test]
    fn invalid_calibration_and_planar_inputs_never_start_surface_execution() {
        for invalid in [0.0, -0.4, f64::NAN, f64::INFINITY] {
            assert!(SurfaceFollower::new(SurfaceFollowerConfig::new(invalid)).is_err());
        }
        let mut follower = SurfaceFollower::new(follower_config()).unwrap();
        assert!(follower
            .update_execution_map(ExecutionMap::Planar(CostMap2D::new_all_free(
                "map", 10, 10, 0.1
            )))
            .is_err());
        assert!(follower
            .replace_execution_path(
                ExecutionPath::Planar(Path2D::empty()),
                None,
                body_pose([1.0, 1.0, 0.5], Quaternion::IDENTITY, 0)
            )
            .is_err());
    }

    #[test]
    fn progress_window_cannot_jump_to_a_later_overlapping_floor() {
        let path = Path3D {
            poses: vec![
                body_pose([1.0, 1.0, 0.5], Quaternion::IDENTITY, 0),
                body_pose([4.0, 1.0, 0.5], Quaternion::IDENTITY, 0),
                body_pose([4.0, 4.0, 3.5], Quaternion::IDENTITY, 0),
                body_pose([1.0, 1.0, 3.5], Quaternion::IDENTITY, 0),
            ],
        };
        let cumulative = cumulative_lengths(&path);
        let (progress, projected) =
            closest_in_window(&path, &cumulative, Point3::new(1.0, 1.0, 3.5), 0.0, 0.4);
        assert!(progress <= 0.4);
        assert_eq!(projected.pose.position.z, 0.5);
        assert!(projected.pose.position.distance(Point3::new(1.0, 1.0, 3.5)) > 2.9);
    }

    #[test]
    fn tilted_body_reference_uses_the_shared_physical_support_transform() {
        let map = test_snapshot(0.0, false);
        let pitch = 0.6_f64;
        let orientation = Quaternion {
            x: 0.0,
            y: (pitch * 0.5).sin(),
            z: 0.0,
            w: (pitch * 0.5).cos(),
        };
        let q = [orientation.x, orientation.y, orientation.z, orientation.w];
        let start = map
            .body_position_from_support([1.05, 1.55, 0.1], q, 0.4)
            .unwrap();
        let end = map
            .body_position_from_support([2.05, 1.55, 0.1], q, 0.4)
            .unwrap();
        assert!(
            (start[0] - 1.05).abs() > 0.2,
            "倾斜机体的支撑偏移必须包含水平分量"
        );
        let mut follower = SurfaceFollower::new(follower_config()).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
            .unwrap();
        follower.update_odometry(body_pose(start, orientation, 0));
        let goal = body_pose(end, orientation, 0);
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![body_pose(start, orientation, 0), goal.clone()],
                }),
                None,
                goal.clone(),
            )
            .unwrap();
        assert!(matches!(follower.step(0.3),PlannerStep::Command(twist) if twist.linear.x > 0.0));
        // 正确平面坐标但高度不同，必须要求重规划，不能只凭二维终点成功。
        let mut wrong_floor = end;
        wrong_floor[2] += 3.0;
        follower.update_odometry(body_pose(wrong_floor, orientation, 1));
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(_)));
    }

    #[test]
    fn shared_surface_planning_and_planar_commands_close_a_real_ramp_route() {
        let slope = 0.15_f64;
        let map = test_snapshot(slope, false);
        let start = map
            .locate_support([0.55, 1.55, 0.1 + slope * 0.55])
            .unwrap()
            .surface_xyz;
        let end = map
            .locate_support([4.55, 1.55, 0.1 + slope * 4.55])
            .unwrap()
            .surface_xyz;
        let support_path = map.plan(start, end).expect("坡道存在真实连通路线");
        let poses: Vec<_> = support_path
            .into_iter()
            .map(|point| {
                body_pose(
                    map.body_position_from_support(point, [0.0, 0.0, 0.0, 1.0], 0.4)
                        .unwrap(),
                    Quaternion::IDENTITY,
                    0,
                )
            })
            .collect();
        let goal = poses.last().unwrap().clone();
        let mut current = poses.first().unwrap().clone();
        let mut follower = SurfaceFollower::new(follower_config()).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
            .unwrap();
        follower.update_odometry(current.clone());
        follower
            .replace_execution_path(ExecutionPath::Surface(Path3D { poses }), None, goal.clone())
            .unwrap();
        let initial_z = current.pose.position.z;
        let mut reached = false;
        for tick in 0..1200 {
            current.stamp.nanos = 1_000_000_000 + tick * 50_000_000;
            follower.update_odometry(current.clone());
            match follower.step(0.3) {
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                PlannerStep::Command(twist) => {
                    let yaw = current.pose.orientation.yaw();
                    current.pose.position.x += twist.linear.x * yaw.cos() * 0.05;
                    current.pose.position.y += twist.linear.x * yaw.sin() * 0.05;
                    current.pose.orientation = Quaternion::from_yaw(yaw + twist.angular_z * 0.05);
                    // 合成位姿的高度来自已观察的真实表面，不从速度凭空累加Z。
                    let support = map
                        .locate_support([
                            current.pose.position.x,
                            current.pose.position.y,
                            0.1 + slope * current.pose.position.x,
                        ])
                        .unwrap();
                    current.pose.position.z = support.surface_xyz[2] + 0.4;
                }
                unexpected => panic!(
                    "坡道实际控制提前停止 tick={tick} pose={current:?} result={unexpected:?}"
                ),
            }
        }
        assert!(reached, "坡道导航应在有限控制步骤内到达");
        assert!(current.pose.position.z - initial_z > 0.4);
        assert!(current.pose.position.distance(goal.pose.position) < 0.12);
    }

    #[test]
    fn calibrated_body_closes_real_ramp_route() {
        use nuwax_nav_mls_planner::SurfaceBodyEnvelope;

        // Static geometry acceptance with synthetic 50 ms applied feedback, not
        // observation-TTL, MotionGuard, PlannerWorker, or hardware qualification.
        let slope = 0.15_f64;
        let dt = 0.05;
        let offset = 0.4;
        let normal_scale = (1.0 + slope * slope).sqrt();
        let envelope = SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.30, -0.16], [0.30, -0.16], [0.30, 0.16], [-0.30, 0.16]],
            z_bounds_m: [-0.05, 0.20], body_to_support_z_m: offset,
            max_step_up_m: 0.2, max_step_down_m: 0.2,
            max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
        };
        let map = Arc::new(test_snapshot(slope, false).with_body_envelope(envelope.clone()).unwrap());
        let limits = MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 0.2, max_reverse_mps: 0.0, max_lateral_mps: 0.0,
            max_angular_rps: 0.55, linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.05,
            minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
        };
        // The simulator uses the exact point-generating plane z=.05+slope*x.
        // Voxel tops remain quantized map evidence; they never reset actual Z.
        // Ry(-atan(slope))*Rz(theta) keeps body Z along the plane normal and
        // chooses theta so the forward axis has the requested world XY yaw.
        let actual_pose = |xy: [f64; 2], yaw: f64, tick: i64| {
            let theta = (yaw.sin() / normal_scale).atan2(yaw.cos());
            let (sp, cp) = (-slope.atan() * 0.5).sin_cos();
            let (st, ct) = (theta * 0.5).sin_cos();
            body_pose(
                [xy[0], xy[1], 0.05 + slope * xy[0] + offset * normal_scale],
                Quaternion { x: sp * st, y: sp * ct, z: cp * st, w: cp * ct },
                tick,
            )
        };
        let observed_support = |pose: &PoseStamped| {
            let p = pose.pose.position;
            // Independently undo offset*n, including its horizontal component.
            let support = [p.x + offset * slope / normal_scale, p.y, p.z - offset / normal_scale];
            assert!((support[2] - (0.05 + slope * support[0])).abs() < 1e-10);
            let q = pose.pose.orientation;
            // At a column edge the exact plane can lie slightly over one voxel
            // below its top; use the existing full-body step-contact contract.
            let observed = map.locate_support_from_body(xyz(p), [q.x, q.y, q.z, q.w], offset)
                .expect("actual full-body ramp support must be observed");
            let source_height = 0.05 + slope * observed.surface_xyz[0];
            assert!(observed.surface_xyz[2] >= source_height - 1e-6);
            assert!(observed.surface_xyz[2] - source_height
                <= f64::from(map.source_voxel_size_m()) + 1e-6);
        };
        let start = map.locate_support([0.55, 1.55, 0.1 + slope * 0.55]).unwrap().surface_xyz;
        let end = map.locate_support([4.55, 1.55, 0.1 + slope * 4.55]).unwrap().surface_xyz;
        assert!((end[0] - start[0] - 4.0).abs() < 1e-5);
        let support_path = map.plan(start, end).expect("the real ramp planner must provide a route");
        let poses: Vec<_> = support_path.into_iter().map(|support| {
            actual_pose([support[0] - offset * slope / normal_scale, support[1]], 0.0, 0)
        }).collect();
        assert!(poses.len() >= 2);
        for pose in &poses { observed_support(pose); }
        let goal = actual_pose([end[0] - offset * slope / normal_scale, end[1]], 0.0, 0);
        let mut current = actual_pose([start[0] - offset * slope / normal_scale, start[1]], 0.0, 0);
        assert_eq!(poses.first().unwrap().pose.position, current.pose.position);
        assert_eq!(poses.last().unwrap().pose.position, goal.pose.position,
            "a shortened planned route must not redefine the original goal");
        let initial_z = current.pose.position.z;
        let mut config = follower_config();
        config.controller.simulation = true;
        config.motion_limits = Some(limits.clone());
        assert!(config.max_observation_age_sec.is_none());
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(current.clone());
        follower.configure_goal(&goal, GoalOptions {
            position_tolerance_m: 0.12, yaw_tolerance_rad: 0.15, allow_reverse: false,
        }).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses }), None, goal.clone()).unwrap();
        let mut applied = Twist2::ZERO;
        follower.observe_applied_command(applied);
        let mut reached = false;
        let mut failure = None;
        let mut executed_steps = 0;
        let mut path_length_m = 0.0;
        let mut peak_control_wall = std::time::Duration::ZERO;
        for tick in 0..1200 {
            current.stamp.nanos = 1_000_000_000 + tick * 50_000_000;
            follower.update_odometry(current.clone());
            assert_eq!(follower.last_command, applied, "control must consume actual applied feedback");
            assert_eq!(follower.config.motion_limits.as_ref(), Some(&limits));
            assert_eq!(follower.map.as_ref().unwrap().body_envelope(), Some(&envelope));
            assert_eq!(follower.source_map.as_ref().unwrap().body_envelope(), Some(&envelope));
            let started = std::time::Instant::now();
            let result = follower.step(0.32);
            peak_control_wall = peak_control_wall.max(started.elapsed());
            let command = match result {
                PlannerStep::Succeeded => {
                    assert!(current.pose.position.distance(goal.pose.position) < 0.12,
                        "success must refer to the original 3D goal");
                    reached = true;
                    break;
                }
                PlannerStep::Command(command) => command,
                other => {
                    failure = Some(format!("tick={tick} pose={current:?} result={other:?}"));
                    break; // A replan is a failure, never an opportunity to teleport or retry.
                }
            };
            // Check the raw request before execution. No simulator clamp may
            // sanitize a reverse, lateral, overspeed, or unreachable command.
            assert!([command.linear.x, command.linear.y, command.angular_z].into_iter().all(f64::is_finite));
            assert!(command.linear.x >= 0.0 && command.linear.x <= limits.max_forward_mps + 1e-12,
                "invalid forward command at tick {tick}: {command:?}");
            assert_eq!(command.linear.y, 0.0, "differential command must not move laterally");
            assert!(command.angular_z.abs() <= limits.max_angular_rps + 1e-12);
            let linear_rate = if command.linear.x < applied.linear.x {
                limits.guaranteed_braking_mps2.min(limits.linear_acceleration_mps2)
            } else { limits.linear_acceleration_mps2 };
            let angular_rate = if command.angular_z.abs() < applied.angular_z.abs() {
                limits.guaranteed_angular_braking_rps2
            } else { limits.angular_acceleration_rps2 };
            assert!((command.linear.x - applied.linear.x).abs() <= linear_rate * dt + 1e-12,
                "unreachable linear command at tick {tick}: previous={applied:?}, requested={command:?}");
            assert!((command.angular_z - applied.angular_z).abs() <= angular_rate * dt + 1e-12,
                "unreachable angular command at tick {tick}: previous={applied:?}, requested={command:?}");
            let speed = (applied.linear.x + command.linear.x) * 0.5;
            let angular = (applied.angular_z + command.angular_z) * 0.5;
            let yaw = current.pose.orientation.yaw();
            let next_yaw = yaw + angular * dt;
            // Independent midpoint-speed SE(2) arc integration, with no path
            // snapping or production terrain-projection helper in the simulator.
            let (dx, dy) = if angular.abs() < 1e-12 {
                (speed * yaw.cos() * dt, speed * yaw.sin() * dt)
            } else {
                (speed * (next_yaw.sin() - yaw.sin()) / angular,
                 speed * (yaw.cos() - next_yaw.cos()) / angular)
            };
            let next = actual_pose([current.pose.position.x + dx, current.pose.position.y + dy], next_yaw, tick + 1);
            assert!((next.pose.position.z - current.pose.position.z - slope * dx).abs() < 1e-10,
                "the simulator must not invent a height jump");
            observed_support(&next);
            let a = current.pose.orientation;
            let b = next.pose.orientation;
            map.validate_body_motion(
                (xyz(current.pose.position), [a.x, a.y, a.z, a.w]),
                (xyz(next.pose.position), [b.x, b.y, b.z, b.w]), &[],
            ).unwrap_or_else(|error| panic!("actual full-body interval rejected at tick {tick}: from={current:?}, to={next:?}, error={error:?}"));
            path_length_m += current.pose.position.distance(next.pose.position);
            current = next;
            applied = command;
            follower.observe_applied_command(applied);
            executed_steps += 1;
        }
        let error_m = current.pose.position.distance(goal.pose.position);
        eprintln!("calibrated ramp: arrived={reached}, actual={:?}, goal={:?}, error_m={error_m:.6}, path_m={path_length_m:.6}, steps={executed_steps}, peak_control_wall_ms={:.3}, failure={failure:?}",
            xyz(current.pose.position), xyz(goal.pose.position), peak_control_wall.as_secs_f64() * 1000.0);
        assert!(failure.is_none(), "calibrated ramp must not fail or enter a replan loop: {failure:?}");
        assert!(reached, "calibrated ramp must reach the original goal within 1200 control steps");
        assert!(current.pose.position.z - initial_z > 0.4);
        assert!(error_m < 0.12);
        assert!(executed_steps > 0 && executed_steps <= 1200);
        assert_eq!(follower.config.motion_limits.as_ref(), Some(&limits));
        assert_eq!(follower.map.as_ref().unwrap().body_envelope(), Some(&envelope));
    }

    #[test]
    fn changed_surface_requests_replan_and_partial_endpoint_never_claims_the_goal() {
        let map = test_snapshot(0.0, false);
        let start = body_pose([0.55, 1.55, 0.5], Quaternion::IDENTITY, 0);
        let end = body_pose([4.55, 1.55, 0.5], Quaternion::IDENTITY, 0);
        let mut follower = SurfaceFollower::new(follower_config()).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map))
            .unwrap();
        follower.update_odometry(start.clone());
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![start.clone(), end.clone()],
                }),
                None,
                end.clone(),
            )
            .unwrap();
        assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
        follower
            .update_execution_map(ExecutionMap::Surface(test_snapshot(0.0, true)))
            .unwrap();
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(_)));
        follower.stop();
        follower
            .update_execution_map(ExecutionMap::Surface(test_snapshot(0.0, false)))
            .unwrap();
        // 同一支撑点的单点安全截断路径可以停车，但原目标仍远在4.55米。
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D { poses: vec![start] }),
                None,
                end,
            )
            .unwrap();
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(_)));
    }
}

#[cfg(test)]
mod stair_tests {
    use super::*;
    use nuwax_nav_mls_planner::mls_planner::Config;
    use nuwax_nav_types_messages::{NavFrameId, Pose3, Timestamp};

    #[test]
    fn real_stair_surface_closes_planar_control_in_both_directions() {
        let size = 0.05_f32;
        let heights: Vec<_> = (0..72)
            .map(|x| {
                if x < 12 {
                    0
                } else if x < 48 {
                    ((x - 12) / 6 + 1) * 3
                } else {
                    18
                }
            })
            .collect();
        let points: Vec<_> = heights
            .iter()
            .enumerate()
            .flat_map(|(x, height)| {
                (0..24).map(move |y| {
                    (
                        (x as f32 + 0.5) * size,
                        (y as f32 + 0.5) * size,
                        (*height as f32 + 0.5) * size,
                    )
                })
            })
            .collect();
        let map = Arc::new(
            SurfaceSnapshot::from_points(
                &points,
                Config {
                    world_frame: "map".into(),
                    voxel_size: size,
                    robot_height: 0.6,
                    surface_closing_radius: 0.0,
                    node_spacing_m: 0.3,
                    wall_clearance_m: 0.2,
                    wall_buffer_m: 0.2,
                    wall_buffer_weight: 0.0,
                    step_threshold_m: 0.2,
                    max_slope_deg: 25.0,
                    min_step_tread_depth_m: 0.2,
                    step_penalty_weight: 1.0,
                    goal_tolerance: 0.12,
                    viz_publish_hz: 0.0,
                },
                size,
            )
            .unwrap(),
        );
        let lower = map
            .locate_support([0.325, 0.625, 0.05])
            .unwrap()
            .surface_xyz;
        let upper = map
            .locate_support([3.275, 0.625, 0.95])
            .unwrap()
            .surface_xyz;
        for (start, end, yaw) in [(lower, upper, 0.0), (upper, lower, std::f64::consts::PI)] {
            let orientation = Quaternion::from_yaw(yaw);
            let poses: Vec<_> = map
                .plan(start, end)
                .unwrap()
                .into_iter()
                .map(|support| {
                    let point = map
                        .body_position_from_support(
                            support,
                            [orientation.x, orientation.y, orientation.z, orientation.w],
                            0.4,
                        )
                        .unwrap();
                    PoseStamped::from_pose(
                        Pose3 {
                            position: Point3::new(point[0], point[1], point[2]),
                            orientation,
                        },
                        NavFrameId::Map,
                    )
                })
                .collect();
            let mut pose = poses.first().unwrap().clone();
            pose.stamp = Timestamp::from_unix(1, 0);
            let goal = poses.last().unwrap().clone();
            let mut config = SurfaceFollowerConfig::new(0.4);
            config.controller.speed = 0.2;
            config.controller.control_frequency = 20.0;
            config.controller.goal_tolerance = 0.12;
            let mut follower = SurfaceFollower::new(config).unwrap();
            follower
                .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
                .unwrap();
            follower.update_odometry(pose.clone());
            follower
                .replace_execution_path(
                    ExecutionPath::Surface(Path3D { poses }),
                    None,
                    goal.clone(),
                )
                .unwrap();
            let mut reached = false;
            for tick in 0..1400 {
                pose.stamp.nanos = 1_000_000_000 + tick * 50_000_000;
                follower.update_odometry(pose.clone());
                match follower.step(0.3) {
                    PlannerStep::Succeeded=>{reached=true;break;}
                    PlannerStep::Command(twist)=>{
                        let heading=pose.pose.orientation.yaw();
                        pose.pose.position.x+=twist.linear.x*heading.cos()*0.05;
                        pose.pose.position.y+=twist.linear.x*heading.sin()*0.05;
                        pose.pose.orientation=Quaternion::from_yaw(heading+twist.angular_z*0.05);
                        let column=(pose.pose.position.x/f64::from(size)).floor() as usize;
                        pose.pose.position.z=(f64::from(heights[column])+1.0)*f64::from(size)+0.4;
                    }
                    unexpected=>panic!("真实楼梯闭环停止 heading={yaw} tick={tick} pose={pose:?} result={unexpected:?}"),
                }
            }
            assert!(reached, "上下楼梯均须到达原机体目标");
            assert!(pose.pose.position.distance(goal.pose.position) < 0.12);
        }
    }
}
