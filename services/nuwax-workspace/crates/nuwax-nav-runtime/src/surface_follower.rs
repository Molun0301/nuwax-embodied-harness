//! 三维表面路径的局部跟随器，只生成平面速度意图，不持有运动端。

use std::sync::Arc;
#[path="surface_body_control.rs"]
mod body_control;

use nuwax_nav_astar::controllers::{angle_diff, Controller, PController};
use nuwax_nav_astar::global_planner::GlobalPlannerConfig;
use nuwax_nav_astar::local_planner::LocalPlannerConfig;
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{Path2D, Path3D, Point2, Point3, PoseStamped, Quaternion, Twist2};

use crate::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceSnapshot};

/// 三维跟随必须明确标定，控制参数复用已有局部控制器配置。
#[derive(Debug, Clone)]
pub struct SurfaceFollowerConfig {
    /// 机体局部负Z方向到支撑参考点的正距离，单位米；不提供猜测默认值。
    pub body_to_support_z_m: f64,
    /// 显式运动及保证制动标定。
    pub motion_limits: Option<nuwax_nav_types_messages::MotionLimits>,
    /// 近场观察最长年龄。
    pub max_observation_age_sec: Option<f64>,
    /// 已有速度、控制频率及位置/朝向容差。
    pub controller: LocalPlannerConfig,
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
            body_to_support_z_m,
            motion_limits: None,
            max_observation_age_sec: None,
            controller: LocalPlannerConfig::default_config(),
            lookahead_m: 0.5,
            max_path_deviation_m: GlobalPlannerConfig::default().max_path_deviation,
        }
    }

    fn validate(&self) -> Result<(), RuntimeError> {
        if let Some(limits)=&self.motion_limits {limits.validate().map_err(|reason|RuntimeError::InvalidInput{field:"surface.motion_limits",reason})?;}
        if self.max_observation_age_sec.is_some_and(|v|!v.is_finite() || v<=0.0){return Err(RuntimeError::InvalidInput {field:"surface.max_observation_age_sec",reason:"观察时效必须为有限正值".into()});}
        for (field, value) in [
            ("body_to_support_z_m", self.body_to_support_z_m),
            ("surface.speed", self.controller.speed),
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
}

/// 跟随器的进度是三维累计弧长，跨地图修订不保存可复用的表面单元编号。
pub struct SurfaceFollower {
    config: SurfaceFollowerConfig,
    controller: PController,
    map: Option<Arc<SurfaceSnapshot>>,
    odometry: Option<PoseStamped>,
    path: Option<Path3D>,
    goal: Option<PoseStamped>,
    cumulative: Vec<f64>,
    progress_m: f64,
    last_tracked_pose: Option<PoseStamped>,
    invalid_input: Option<String>,
    path_index: usize,
    options: nuwax_nav_types_messages::GoalOptions,
    last_command: Twist2,
    observed_velocity: Twist2,
    braking: bool,
    settled_samples: usize,
    control_target: Option<nuwax_nav_control::PlanarPose>,
}

impl SurfaceFollower {
    /// 构造纯计算后端，不连接运动服务。
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
        config.validate()?;
        let speed = config.controller.speed * config.controller.nerf_speed.min(1.0);
        let mut controller = PController::new(speed, config.controller.control_frequency);
        controller.set_simulation(config.controller.simulation);
        Ok(Self {
            config,
            controller,
            map: None,
            odometry: None,
            path: None,
            goal: None,
            cumulative: Vec::new(),
            progress_m: 0.0,
            last_tracked_pose: None,
            invalid_input: None,
            path_index: 1,
            options:Default::default(),
            last_command:Twist2::ZERO,
            observed_velocity:Twist2::ZERO,
            braking:false,
            settled_samples:0,
            control_target:None,
        })
    }

    /// 当前累计三维进度，供状态诊断与行为回归使用。
    #[must_use]
    pub fn progress_m(&self) -> f64 {
        self.progress_m
    }

    fn support(map: &SurfaceSnapshot, pose: &PoseStamped, offset: f64) -> Result<[f64; 3], String> {
        let q = pose.pose.orientation;
        map.locate_support_from_body(xyz(pose.pose.position), [q.x, q.y, q.z, q.w], offset)
            .map(|location| location.support_xyz)
            .map_err(|error| error.to_string())
    }

    fn admit_path(&mut self, path: Path3D, goal: PoseStamped) -> Result<(), RuntimeError> {
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
        let current_support =
            Self::support(map, odometry, self.config.body_to_support_z_m).map_err(rejected)?;
        // 完整规划属于后台MLS拥有方；调度器只验证当前接续与有界前向段，
        // 避免长路线的全图遍历阻塞优先取消和续租。
        let (progress, projected) = closest_in_window(
            &path,
            &cumulative,
            odometry.pose.position,
            0.0,
            self.config.max_path_deviation_m,
        );
        if projected.pose.position.distance(odometry.pose.position)
            > self.config.max_path_deviation_m
        {
            return Err(rejected("新路径首段不能与当前三维机体位置连续接续"));
        }
        let projected_support =
            Self::support(map, &projected, self.config.body_to_support_z_m).map_err(rejected)?;
        map.validate_segment(current_support, projected_support)
            .map_err(|error| rejected(error.to_string()))?;
        self.progress_m = progress;
        self.last_tracked_pose = Some(odometry.clone());
        self.path_index=1;
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
                .min(crate::DEFAULT_POSE_TTL.as_secs_f64());
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
            if yaw_error.abs() < self.config.controller.orientation_tolerance {
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
        if self.config.max_observation_age_sec.is_some()
            || self.config.motion_limits.is_some() && self.map.as_ref().is_some_and(|m|m.body_envelope().is_some()) {
            return self.bounded_body_command(twist);
        }
        // 原控制器最小速度不能突破三维执行显式配置的低速上限。
        let speed = self.config.controller.speed * self.config.controller.nerf_speed.min(1.0);
        let command = twist.clamp(speed, speed);
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
        if rotation.angular_z.abs() > 1e-9 && self.motion_is_traversable(rotation) {
            return PlannerStep::Command(rotation);
        }
        PlannerStep::ReplanRequired("短时速度轨迹没有连续支撑或足够净空".into())
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
        let Ok(initial_support) = Self::support(map, pose, self.config.body_to_support_z_m) else {
            return false;
        };
        let yaw = pose.pose.orientation.yaw();
        let horizon = (crate::DEFAULT_POSE_TTL + crate::DEFAULT_TICK_INTERVAL).as_secs_f64();
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
            if map.body_envelope().is_some() {
                let body_at=|support:[f64;3],yaw:f64| {
                    let q=map.body_orientation_at(support,yaw).ok()?;
                    let p=map.body_position_from_support(support,q,self.config.body_to_support_z_m).ok()?;
                    Some((p,q))
                };
                let (Some(a),Some(b),Some(c),Some(d))=(body_at(support,yaw+angle*(sample-1) as f64/sample as f64),body_at(next.support_xyz,yaw+angle),body_at(delayed_support,yaw),body_at(next_delayed.support_xyz,yaw))else{return false;};
                if map.validate_body_motion(a,b,&[]).is_err() || map.validate_body_motion(c,d,&[]).is_err() || map.validate_body_motion(b,d,&[]).is_err(){return false;}
            }
            support = next.support_xyz;
            delayed_support = next_delayed.support_xyz;
        }
        true
    }
}

impl NavPlanner for SurfaceFollower {
    fn configure_goal(&mut self, _goal:&PoseStamped, options:nuwax_nav_types_messages::GoalOptions)->Result<(),RuntimeError> {
        options.validate().map_err(|reason|RuntimeError::InvalidInput {field:"goal_options",reason})?;
        if options.allow_reverse && self.config.motion_limits.as_ref().is_none_or(|m|m.max_reverse_mps<=0.0) {return Err(RuntimeError::InvalidInput {field:"allow_reverse",reason:"三维执行未提供倒车标定".into()});}
        self.options=options;
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
        match path {
            ExecutionPath::Surface(path) if yaws.is_none() => self.admit_path(path, goal),
            _ => Err(rejected(
                "三维后端必须接收完整三维路径，不能使用二维朝向来源",
            )),
        }
    }

    fn update_odometry(&mut self, odometry: PoseStamped) {
        self.observe_body_velocity(&odometry);
        self.odometry = Some(odometry);
    }

    fn update_costmap(&mut self, _costmap: CostMap2D) {
        self.map = None;
        self.invalid_input = Some("三维后端拒绝二维占据图".into());
    }

    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        match map {
            ExecutionMap::Surface(map) => {
                if map.config().world_frame != "map" {
                    return Err(rejected("三维表面快照必须使用当前机体路径的map坐标系"));
                }
                self.map = Some(map);
                Ok(())
            }
            ExecutionMap::Planar(_) => Err(rejected("三维后端不能用二维占据图恢复就绪")),
        }
    }

    fn invalidate_execution_map(&mut self) {
        self.stop();
        self.map = None;
    }

    fn step(&mut self, robot_width_m: f64) -> PlannerStep {
        self.follow_step(robot_width_m)
    }

    fn stop(&mut self) {
        self.last_command=Twist2::ZERO;
        if self.observed_velocity.linear.x.hypot(self.observed_velocity.linear.y)>0.02 || self.observed_velocity.angular_z.abs()>0.04 {self.braking=true;self.settled_samples=0;}
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
    use nuwax_nav_types_messages::{NavFrameId, Pose3, Timestamp};

    #[test]
    fn near_field_evidence_covers_the_emergency_braking_tail() {
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
        for fresh in [true, false] {
            let mut blocks: BTreeMap<_, BTreeMap<_, _>> = BTreeMap::new();
            for (x,y,z) in base.cell_keys() {
                let key = [x,y,z];
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                    SurfaceObservation { observed_ns: if fresh || x < 12 {now} else {1}, hits: 2 });
            }
            let mut observations = SurfaceObservations::new(base.source_voxel_size_m(), 32).unwrap();
            for (key, block) in blocks { observations.replace_block(key,block).unwrap(); }
            let map = Arc::new(base.with_observations(Some(Arc::new(observations))));
            let mut config = SurfaceFollowerConfig::new(0.3);
            config.max_observation_age_sec = Some(1.0);
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
            follower.observed_velocity = speed;
            follower.control_target = Some(nuwax_nav_control::PlanarPose { x: 2.0, y: 0.55, yaw: 0.0 });
            let result = follower.bounded_command(speed);
            if fresh { assert!(matches!(result, PlannerStep::Command(v) if v.linear.x > 0.0), "{result:?}"); }
            else {
                // 初始机体及第一个50毫秒段仍在新鲜区域，停车尾段会越过1.2米。
                assert!(matches!(result, PlannerStep::ReplanRequired(_)), "{result:?}");
                assert_eq!(follower.last_command, Twist2::ZERO);
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
