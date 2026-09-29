//! 3DMR式持续路线执行：全局参考留存，Scan只更新前方轨迹，原控制器继续唯一生成速度。
use crate::{
    ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceFollower,
    SurfaceFollowerConfig, SurfaceSnapshot,
};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_pct::PctPlanner;
use nuwax_nav_scan::{BodyPose, ScanConfig};
use nuwax_nav_types_messages::{
    GoalOptions, Path2D, Path3D, Point3, PoseStamped, Quaternion, Twist2,
};
use std::sync::Arc;

/// 只保存最近的失败查询；完全相同的输入不重复执行昂贵的姿态搜索。
struct FailedLocalPlan {
    map: Arc<SurfaceSnapshot>,
    start: BodyPose,
    reference: Vec<[f64; 3]>,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    failure: nuwax_nav_scan::LocalPlanError,
    follow_reference: bool,
    prefer_straight: bool,
}

pub struct RouteFollower {
    controller: SurfaceFollower,
    config: SurfaceFollowerConfig,
    scan: ScanConfig,
    boundary: Vec<[f64; 2]>,
    planner: Arc<PctPlanner>,
    map: Option<Arc<SurfaceSnapshot>>,
    pose: Option<PoseStamped>,
    route: Option<Path3D>,
    goal: Option<PoseStamped>,
    options: GoalOptions,
    cumulative: Vec<f64>,
    progress: f64,
    local_end: f64,
    local_progress_at_plan: f64,
    local_path: Option<Path3D>,
    local_map: Option<(Arc<SurfaceSnapshot>, Point3, Arc<nuwax_nav_pct::Tomogram>)>,
    local_start: Option<Point3>,
    dirty: bool,
    local_plans: u64,
    failed_local: Vec<FailedLocalPlan>,
}
fn error(message: impl Into<String>) -> RuntimeError {
    RuntimeError::InvalidInput {
        field: "route",
        reason: message.into(),
    }
}
fn body(p: &PoseStamped) -> BodyPose {
    let q = p.pose.orientation;
    (
        [p.pose.position.x, p.pose.position.y, p.pose.position.z],
        [q.x, q.y, q.z, q.w],
    )
}
impl RouteFollower {
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
        let scan = config.scan.clone();
        scan.validate().map_err(error)?;
        Ok(Self {
            controller: SurfaceFollower::new(config.clone())?,
            planner: Arc::clone(&config.pct),
            config,
            scan,
            boundary: Vec::new(),
            map: None,
            pose: None,
            route: None,
            goal: None,
            options: GoalOptions::default(),
            cumulative: Vec::new(),
            progress: 0.0,
            local_end: 0.0,
            local_progress_at_plan: 0.0,
            local_path: None,
            local_map: None,
            local_start: None,
            dirty: true,
            local_plans: 0,
            failed_local: Vec::new(),
        })
    }
    /// 已执行的全局路线弧长，不受近场轨迹替换重置。
    pub fn progress_m(&self) -> f64 {
        self.progress
    }
    pub fn local_plans(&self) -> u64 {
        self.local_plans
    }
    pub fn local_path(&self) -> Option<&Path3D> {
        self.local_path.as_ref()
    }
    fn advance_progress(&mut self) {
        let (Some(route), Some(pose)) = (&self.route, &self.pose) else {
            return;
        };
        let mut best = f64::INFINITY;
        let mut progress = self.progress;
        let window =
            (self.progress + self.scan.horizon_m).min(*self.cumulative.last().unwrap_or(&0.0));
        for (i, pair) in route.poses.windows(2).enumerate() {
            if self.cumulative[i + 1] + 1e-6 < self.progress {
                continue;
            }
            if self.cumulative[i] > window {
                break;
            }
            let a = pair[0].pose.position;
            let b = pair[1].pose.position;
            let v = [b.x - a.x, b.y - a.y, b.z - a.z];
            let l2 = v.iter().map(|x| x * x).sum::<f64>();
            if l2 < 1e-12 {
                continue;
            }
            let d = pose.pose.position;
            let t = (((d.x - a.x) * v[0] + (d.y - a.y) * v[1] + (d.z - a.z) * v[2]) / l2)
                .clamp(0.0, 1.0);
            let projected = Point3::new(a.x + v[0] * t, a.y + v[1] * t, a.z + v[2] * t);
            let distance = projected.distance(d);
            if distance < best {
                best = distance;
                progress = self.cumulative[i] + t * l2.sqrt();
            }
        }
        if best < self.config.max_path_deviation_m {
            self.progress = self.progress.max(progress);
        }
    }
    fn reference(&self, horizon: f64) -> Result<(Vec<[f64; 3]>, f64, bool, bool), String> {
        let route = self.route.as_ref().ok_or("缺少全局路线")?;
        let current = self.pose.as_ref().ok_or("缺少实际位姿")?;
        let offset = self.config.body_to_support_z_m;
        let mut result = vec![[
            current.pose.position.x,
            current.pose.position.y,
            current.pose.position.z - offset,
        ]];
        let end = (self.progress + horizon).min(*self.cumulative.last().ok_or("全局路线为空")?);
        let mut direction:Option<[f64;2]>=None;
        let mut global_straight=true;
        for (i, pair) in route.poses.windows(2).enumerate() {
            if self.cumulative[i + 1] <= self.progress + 0.02 {
                continue;
            }
            let p = pair[1].pose.position;
            // 路段形状由原全局路线决定，实际机体的横向偏差不能把直线变成弯道。
            let delta=[p.x-pair[0].pose.position.x,p.y-pair[0].pose.position.y];
            let length=delta[0].hypot(delta[1]);
            if length>1e-9 {
                let next=delta.map(|v|v/length);
                if let Some(first)=direction {
                    global_straight &= first[0]*next[0]+first[1]*next[1]>0.0
                        && (first[0]*next[1]-first[1]*next[0]).abs()<1e-9;
                }else{direction=Some(next);}
            }
            if self.cumulative[i + 1] <= end {
                result.push([p.x, p.y, p.z - offset]);
            } else {
                let a = pair[0].pose.position;
                let length = self.cumulative[i + 1] - self.cumulative[i];
                let t = ((end - self.cumulative[i]) / length.max(1e-9)).clamp(0.0, 1.0);
                result.push([
                    a.x + (p.x - a.x) * t,
                    a.y + (p.y - a.y) * t,
                    a.z + (p.z - a.z) * t - offset,
                ]);
                break;
            }
        }
        let at_end = end >= *self.cumulative.last().unwrap() - 1e-6;
        if at_end {
            if let Some(goal) = &self.goal {
                if current.pose.position.distance(goal.pose.position)
                    >= self.options.position_tolerance_m
                {
                    // 层析末点是名义地面参考，实际站姿仍须进入用户三维到达球。
                    // 保留指向原目标的末段，允许投影在真正入球时结束。
                    let target = [
                        goal.pose.position.x,
                        goal.pose.position.y,
                        goal.pose.position.z - offset,
                    ];
                    if result.last() != Some(&target) {
                        result.push(target);
                    }
                }
            }
        }
        Ok((result, end, at_end, global_straight))
    }
    fn regenerate(&mut self, detour: bool) -> Result<(), String> {
        let map = self.map.as_ref().ok_or("缺少当前地形")?;
        let pose = self.pose.as_ref().ok_or("缺少实际位姿")?;
        let mut scan = self.scan.clone();
        scan.reference_corridor_m = self.config.max_path_deviation_m;
        let footprint = map.body_envelope().map_or(0.0, |body| body.radius() * 2.0);
        let turning = self
            .config
            .kinematics()
            .map_or(0.0, |motion| motion.minimum_turning_radius_m * 2.0);
        scan.horizon_m = scan
            .horizon_m
            .max(footprint)
            .max(turning)
            .max(self.config.lookahead_m * 2.0);
        let tomogram = if let Some((_, _, local)) =
            self.local_map.as_ref().filter(|(previous, center, _)| {
                nuwax_nav_pct::same_source_geometry(previous, map)
                    && center.distance(pose.pose.position) < 0.25
            }) {
            Arc::clone(local)
        } else {
            let local = Arc::new(nuwax_nav_pct::Tomogram::local_from_surface(
                map,
                [pose.pose.position.x, pose.pose.position.y],
                scan.horizon_m + 1.5,
            )?);
            self.local_map = Some((Arc::clone(map), pose.pose.position, Arc::clone(&local)));
            local
        };
        let kinematics = self.config.kinematics().ok_or("近场执行缺少运动能力")?;
        let mut selected = None;
        let mut failures = Vec::new();
        // 局部轨迹须连到前方路线；不能以不断截短的可行前缀把机体送入无后续空间的角落。
        for horizon in [scan.horizon_m] {
            let (mut reference, end, at_end, global_straight) = self.reference(horizon)?;
            let prefer_straight=global_straight && !detour;
            let final_yaw = if at_end && self.options.requires_orientation() {
                self.goal.as_ref().map(|g| g.pose.orientation.yaw())
            } else {
                None
            };
            let arrival = if at_end {
                self.goal
                    .as_ref()
                    .map(|g| (body(g).0, self.options.position_tolerance_m))
            } else {
                None
            };
            if detour && reference.len() > 1 {
                let target = *reference.last().unwrap();
                if let Ok(route) = tomogram.plan_bounded(
                    reference[0],
                    target,
                    tomogram.resolution,
                    &self.boundary,
                    None,
                    Some(scan.horizon_m * 1.5),
                    &self.planner.local_costs(),
                ) {
                    reference = route.points;
                }
            }
            let start = body(pose);
            let cached = self.failed_local.iter().find(|cached| {
                cached.map.same_body_query_view(map)
                    && cached.start == start
                    && cached.reference == reference
                    && cached.final_yaw == final_yaw
                    && cached.arrival == arrival
                    && cached.follow_reference == detour
                    && cached.prefer_straight == prefer_straight
            });
            let result = if let Some(cached) = cached {
                Err(cached.failure.clone())
            } else {
                let result = nuwax_nav_scan::plan(
                    map, &tomogram, start, &reference, &kinematics, &scan,
                    final_yaw, arrival, &self.boundary, detour, prefer_straight,
                );
                if let Err(failure) = &result {
                    if self.failed_local.len() == 2 {
                        self.failed_local.remove(0);
                    }
                    self.failed_local.push(FailedLocalPlan {
                        map: Arc::clone(map), start, reference, final_yaw, arrival,
                        failure: failure.clone(),
                        follow_reference: detour,
                        prefer_straight,
                    });
                }
                result
            };
            match result {
                Ok(trajectory) => {
                    selected = Some((trajectory, end));
                    break;
                }
                Err(reason) => failures.push(reason),
            }
        }
        let (trajectory, end) = selected.ok_or_else(|| {
            if let Some(point) = failures.iter().find_map(|failure| failure.blocked) {
                let mut support = point;
                support[2] -= self.config.body_to_support_z_m;
                self.planner.report_local_blockage(
                    support,
                    map.body_envelope().map_or(0.4, |body| body.radius() * 1.5),
                );
            }
            format!(
                "近场轨迹暂不可接续：{}",
                failures
                    .last()
                    .map_or("没有参考段", |failure| failure.reason.as_str())
            )
        })?;
        let path = Path3D {
            poses: trajectory
                .poses
                .iter()
                .map(|&(p, q)| {
                    let mut value = pose.clone();
                    value.pose.position = Point3::new(p[0], p[1], p[2]);
                    value.pose.orientation = Quaternion {
                        x: q[0],
                        y: q[1],
                        z: q[2],
                        w: q[3],
                    };
                    value
                })
                .collect(),
        };
        self.local_path = Some(path.clone());
        self.controller
            .replace_execution_path(
                ExecutionPath::Surface(path),
                None,
                self.goal.clone().ok_or("缺少用户目标")?,
            )
            .map_err(|e| e.to_string())?;
        self.local_progress_at_plan = self.progress;
        self.local_end = end;
        self.local_start = Some(pose.pose.position);
        self.dirty = false;
        self.local_plans += 1;
        tracing::info!(target:"navigation.scan",event="navigation.scan.trajectory",progress_m=self.progress,
            local_end_m=end,elapsed_ms=trajectory.elapsed_ms,points=trajectory.poses.len(),smoothed=trajectory.smoothed,
            local_plans=self.local_plans,"近场轨迹已接续，继续使用全局路线");
        Ok(())
    }
}
impl NavPlanner for RouteFollower {
    fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
        self.failed_local.clear();
        self.boundary = boundary.to_vec();
        self.controller.configure_boundary(boundary);
    }
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: GoalOptions,
    ) -> Result<(), RuntimeError> {
        self.failed_local.clear();
        self.planner.clear_local_costs();
        self.controller.configure_goal(goal, options)?;
        self.options = options;
        self.goal = Some(goal.clone());
        Ok(())
    }
    fn replace_path(&mut self, path: Path2D) {
        self.stop();
        self.controller.replace_path(path);
    }
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        let ExecutionPath::Surface(route) = path else {
            return Err(error("三维路线不能使用二维路径"));
        };
        if yaws.is_some() || route.poses.is_empty() {
            return Err(error("三维路线为空或带重复朝向源"));
        }
        if self
            .map
            .as_ref()
            .is_some_and(|map| map.body_envelope().is_none())
        {
            return self.controller.replace_execution_path(
                ExecutionPath::Surface(route),
                None,
                goal,
            );
        }
        for pose in &route.poses {
            crate::validate_map_pose(pose, "route.payload")?;
        }
        crate::validate_map_pose(&goal, "goal_pose")?;
        self.cumulative = vec![0.0];
        for pair in route.poses.windows(2) {
            self.cumulative.push(
                self.cumulative.last().unwrap()
                    + pair[0].pose.position.distance(pair[1].pose.position),
            );
        }
        self.route = Some(route);
        self.goal = Some(goal);
        self.progress = 0.0;
        self.local_end = 0.0;
        self.dirty = true;
        self.advance_progress();
        // 近场计算在原受管控制工作线程执行，路径受理不等待整条轨迹精化。
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        self.controller.update_odometry(pose.clone());
        self.pose = Some(pose);
    }
    fn update_costmap(&mut self, map: CostMap2D) {
        self.stop();
        self.map = None;
        self.controller.update_costmap(map);
    }
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        let ExecutionMap::Surface(surface) = map else {
            return Err(error("三维路线执行缺少三维地图"));
        };
        // 新图始终进入实际控制检查；远离路线的变化不强制丢弃仍可执行的近场轨迹。
        if self.local_path.is_none() {
            self.dirty = true;
        }
        self.controller
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&surface)))?;
        self.map = Some(surface);
        Ok(())
    }
    fn invalidate_execution_map(&mut self) {
        self.stop();
        self.map = None;
        self.controller.invalidate_execution_map();
    }
    fn observe_applied_command(&mut self, command: Twist2) {
        self.controller.observe_applied_command(command);
    }
    fn step(&mut self, width: f64) -> PlannerStep {
        if self
            .map
            .as_ref()
            .is_some_and(|map| map.body_envelope().is_none())
        {
            return self.controller.step(width);
        }
        self.advance_progress();
        let window = (self.local_end - self.local_progress_at_plan).max(0.1);
        let moved = self
            .pose
            .as_ref()
            .zip(self.local_start)
            .is_some_and(|(p, start)| {
                p.pose.position.distance(start) >= self.scan.refresh_distance_m.min(window * 0.6)
            });
        let at_global_end = self
            .cumulative
            .last()
            .is_some_and(|&end| self.local_end >= end - 1e-6);
        let near_local_end = !at_global_end
            && self.local_end - self.progress < self.scan.refresh_distance_m.min(window * 0.35);
        if self.dirty || moved && !at_global_end || near_local_end {
            if let Err(reason) = self.regenerate(false) {
                if let Err(local) = self.regenerate(true) {
                    // 新前方窗口无法接续时应从当前仍有回旋空间的位置重算全局。
                    // 继续消耗旧窗口可能把机体送进已知没有后续连接的角落。
                    return PlannerStep::ReplanRequired(format!("{reason}；近场绕行：{local}"));
                }
            }
        }
        match self.controller.step(width) {
            PlannerStep::ReplanRequired(reason) => match self.regenerate(true) {
                Ok(()) => self.controller.step(width),
                Err(local) => {
                    PlannerStep::ReplanRequired(format!("{reason}；局部重接续失败：{local}"))
                }
            },
            step => step,
        }
    }
    fn stop(&mut self) {
        self.controller.stop();
        self.route = None;
        self.goal = None;
        self.cumulative.clear();
        self.progress = 0.0;
        self.local_end = 0.0;
        self.local_progress_at_plan = 0.0;
        self.local_path = None;
        self.failed_local.clear();
        self.local_map = None;
        self.local_start = None;
        self.dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
    use nuwax_nav_types_messages::{MotionKinematics, MotionModel, NavFrameId, Pose3, Timestamp};
    #[test]
    fn straight_global_route_stays_straight_after_lateral_tracking_error() {
        let map=Arc::new(crate::surface_follower::test_snapshot(0.0,false)
            .with_body_envelope(SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
                max_cross_slope_deg:18.0,support_uncertainty_m:0.0}).unwrap());
        let make=|x,y|PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,0),
            pose:Pose3{position:Point3::new(x,y,0.5),orientation:Quaternion::IDENTITY}};
        let mut config=SurfaceFollowerConfig::new(0.4);config.controller.simulation=true;
        config.basic_kinematics=Some(MotionKinematics{model:MotionModel::Differential,max_forward_mps:1.0,
            max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:0.8,minimum_turning_radius_m:0.0,can_rotate_in_place:true});
        let mut follower=RouteFollower::new(config).unwrap();
        let actual=make(0.5,1.2);let goal=make(5.0,1.0);
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(actual.clone());follower.configure_goal(&goal,GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![make(0.5,1.0),make(2.0,1.0),goal.clone()]}),None,goal).unwrap();
        follower.regenerate(false).unwrap();
        let path=follower.local_path().unwrap();let end=path.poses.last().unwrap().pose.position;
        let start=actual.pose.position;let dx=end.x-start.x;let dy=end.y-start.y;
        assert!(path.poses.iter().all(|p|((p.pose.position.x-start.x)*dy-(p.pose.position.y-start.y)*dx).abs()<1e-8),
            "全局仍是直线时，小横差不能重新生成S形局部轨迹");
        let goal=make(2.0,2.0);
        follower.configure_goal(&goal,GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![make(0.5,1.0),make(2.0,1.0),goal.clone()]}),None,goal).unwrap();
        follower.regenerate(false).unwrap();
        let path=follower.local_path().unwrap();let end=path.poses.last().unwrap().pose.position;
        let dx=end.x-start.x;let dy=end.y-start.y;
        assert!(path.poses.iter().any(|p|((p.pose.position.x-start.x)*dy-(p.pose.position.y-start.y)*dx).abs()>0.05),
            "真实全局转角仍应保留转弯，不能一律切成直线");
    }

    #[test]
    fn nominal_end_does_not_replace_the_actual_three_dimensional_goal_ball() {
        let map = Arc::new(
            crate::surface_follower::test_snapshot(0.0, false)
                .with_body_envelope(SurfaceBodyEnvelope {
                    footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                    z_bounds_m: [-0.05, 0.2],
                    body_to_support_z_m: 0.4,
                    max_step_up_m: 0.2,
                    max_step_down_m: 0.2,
                    max_cross_slope_deg: 18.0,
                    support_uncertainty_m: 0.0,
                })
                .unwrap(),
        );
        let make = |x, z| PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(x, 1.0, z),
                orientation: Quaternion::from_yaw(0.0),
            },
        };
        let goal = make(3.0, 0.5);
        let mut actual = make(1.0, 0.55);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.simulation = true;
        config.basic_kinematics = Some(MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        });
        let mut follower = RouteFollower::new(config).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map))
            .unwrap();
        follower.update_odometry(actual.clone());
        follower
            .configure_goal(
                &goal,
                GoalOptions {
                    position_tolerance_m: 0.35,
                    ..Default::default()
                },
            )
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![actual.clone(), make(2.652, 0.5)],
                }),
                None,
                goal.clone(),
            )
            .unwrap();
        let mut arrived = false;
        for i in 0..300 {
            match follower.step(0.3) {
                PlannerStep::Command(command) => {
                    assert_eq!(command.linear.y, 0.0);
                    assert!(command.linear.x >= 0.0);
                    let yaw = actual.pose.orientation.yaw();
                    actual.pose.position.x += command.linear.x * yaw.cos() * 0.05;
                    actual.pose.position.y += command.linear.x * yaw.sin() * 0.05;
                    actual.pose.orientation = Quaternion::from_yaw(yaw + command.angular_z * 0.05);
                    actual.stamp = Timestamp::from_unix(2 + i / 20, (i % 20) as u32 * 50_000_000);
                    follower.observe_applied_command(command);
                    follower.update_odometry(actual.clone());
                }
                PlannerStep::Succeeded => {
                    arrived = true;
                    break;
                }
                other => panic!("名义末点不能使有效目标反复重规划：{other:?}"),
            }
        }
        assert!(arrived);
        assert!(actual.pose.position.distance(goal.pose.position) < 0.35);
    }
    #[test]
    fn disconnected_forward_window_replans_before_consuming_old_trajectory() {
        let envelope = SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
            z_bounds_m: [-0.05, 0.2],
            body_to_support_z_m: 0.4,
            max_step_up_m: 0.2,
            max_step_down_m: 0.2,
            max_cross_slope_deg: 18.0,
            support_uncertainty_m: 0.0,
        };
        let make = |x| PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(x, 1.0, 0.5),
                orientation: Quaternion::from_yaw(0.0),
            },
        };
        let start = make(0.5);
        let goal = make(5.0);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.simulation = true;
        config.scan.refresh_distance_m = 0.15;
        config.basic_kinematics = Some(MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        });
        let mut follower = RouteFollower::new(config).unwrap();
        let clear = Arc::new(
            crate::surface_follower::test_snapshot(0.0, false)
                .with_body_envelope(envelope.clone())
                .unwrap(),
        );
        follower
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&clear)))
            .unwrap();
        follower.update_odometry(start.clone());
        follower
            .configure_goal(&goal, GoalOptions::default())
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![start, goal.clone()],
                }),
                None,
                goal,
            )
            .unwrap();
        assert!(matches!(follower.step(0.2), PlannerStep::Command(_)));
        let previous_local = follower.local_plans();
        let missing = Arc::new(
            crate::surface_follower::test_snapshot(0.0, true)
                .with_body_envelope(envelope)
                .unwrap(),
        );
        follower
            .update_execution_map(ExecutionMap::Surface(missing))
            .unwrap();
        let mut moved = make(0.7);
        moved.stamp = Timestamp::from_unix(2, 0);
        follower.update_odometry(moved);
        assert!(
            matches!(follower.step(0.2), PlannerStep::ReplanRequired(_)),
            "前方窗口无连接时应及时改变全局路线，不能继续进入已知无后续空间的旧路径"
        );
        assert_eq!(follower.local_plans(), previous_local);
        assert!(
            follower.local_path().is_some(),
            "重规划请求不清除可供显示的最后局部轨迹"
        );
        assert!(!follower.failed_local.is_empty());
        let cached = follower.failed_local.len();
        assert!(follower.regenerate(false).is_err());
        assert_eq!(follower.failed_local.len(), cached);
        follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();
        follower.regenerate(false).expect("障碍移除后同一位姿必须能重新接续");
        assert_eq!(follower.local_plans(), previous_local + 1);
        follower.stop();
        assert!(follower.failed_local.is_empty());
    }
}
