//! 3DMR式持续路线执行：全局参考留存，Scan只更新前方轨迹，原控制器继续唯一生成速度。
use crate::{
    ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceFollower,
    SurfaceFollowerConfig, SurfaceSnapshot,
};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_pct::PctPlanner;
use nuwax_nav_scan::{BodyPose, LocalPlanError, ScanConfig};
use nuwax_nav_types_messages::{
    GoalOptions, Path2D, Path3D, Point3, PoseStamped, Quaternion, Twist2,
};
use std::sync::Arc;

/// 只保存最近的失败查询；完全相同的输入不重复执行昂贵的姿态搜索。
struct FailedLocalPlan {
    map: Arc<SurfaceSnapshot>,
    tomogram: Arc<nuwax_nav_pct::Tomogram>,
    boundary: Vec<[f64; 2]>,
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
    local_planning_calls: u64,
    failed_local: Vec<FailedLocalPlan>,
    wait_reason: Option<String>,
    route_generation: u64,
    admitted_generation: Option<u64>,
    local_failures: Vec<LocalPlanError>,
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
fn exact_points<const N: usize>(a: &[[f64; N]], b: &[[f64; N]]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a,b)| a.map(f64::to_bits) == b.map(f64::to_bits))
}
fn pose_bits(pose: BodyPose) -> ([u64; 3], [u64; 4]) {
    (pose.0.map(f64::to_bits), pose.1.map(f64::to_bits))
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
            local_planning_calls: 0,
            failed_local: Vec::new(),
            wait_reason: None,
            route_generation: 0,
            admitted_generation: None,
            local_failures: Vec::new(),
        })
    }
    /// 已执行的全局路线弧长，不受近场轨迹替换重置。
    pub fn progress_m(&self) -> f64 {
        self.progress
    }
    pub fn local_plans(&self) -> u64 {
        self.local_plans
    }
    /// Number of entries into local planning, including failed attempts.
    pub fn local_planning_calls(&self) -> u64 { self.local_planning_calls }
    pub fn local_path(&self) -> Option<&Path3D> {
        self.local_path.as_ref()
    }
    /// Typed diagnostics from the latest unsuccessful local planning attempts.
    pub fn local_failures(&self) -> &[LocalPlanError] { &self.local_failures }
    fn revoke_local_admission(&mut self) {
        self.route_generation = self.route_generation.wrapping_add(1);
        self.admitted_generation = None;
        self.local_path = None;
        self.local_start = None;
        self.failed_local.clear();
        self.local_failures.clear();
        self.dirty = true;
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
    fn regenerate(&mut self, detour: bool) -> Result<(), LocalPlanError> {
        if !self.config.local_planning_enabled {
            return Err("local planning is disabled".into());
        }
        self.local_planning_calls += 1;
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
                Arc::ptr_eq(&cached.tomogram, &tomogram)
                    && exact_points(&cached.boundary, &self.boundary)
                    && cached.map.same_body_query_view(map)
                    && pose_bits(cached.start) == pose_bits(start)
                    && exact_points(&cached.reference, &reference)
                    && cached.final_yaw.map(f64::to_bits) == final_yaw.map(f64::to_bits)
                    && cached.arrival.map(|(p,t)| (p.map(f64::to_bits),t.to_bits()))
                        == arrival.map(|(p,t)| (p.map(f64::to_bits),t.to_bits()))
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
                        map: Arc::clone(map), tomogram: Arc::clone(&tomogram),
                        boundary: self.boundary.clone(), start, reference, final_yaw, arrival,
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
            failures.pop().unwrap_or_else(|| "没有参考段".into())
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
        self.controller
            .replace_execution_path(
                ExecutionPath::Surface(path.clone()),
                None,
                self.goal.clone().ok_or("缺少用户目标")?,
            )
            .map_err(|e| e.to_string())?;
        self.local_path = Some(path);
        self.admitted_generation = Some(self.route_generation);
        self.local_failures.clear();
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
        if self.config.local_planning_enabled { self.planner.clear_local_costs(); }
        self.controller.configure_goal(goal, options)?;
        self.revoke_local_admission();
        self.route = None;
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
        for pose in &route.poses {
            crate::validate_map_pose(pose, "route.payload")?;
        }
        crate::validate_map_pose(&goal, "goal_pose")?;
        let direct = !self.config.local_planning_enabled
            || self.map.as_ref().is_some_and(|map| map.body_envelope().is_none());
        if direct {
            self.controller.replace_execution_path(
                ExecutionPath::Surface(route.clone()), None, goal.clone(),
            )?;
        }

        self.revoke_local_admission();
        if direct { self.admitted_generation = Some(self.route_generation); }
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
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        if self.wait_reason.is_some() { return Vec::new(); }
        self.local_path.as_ref().map(|path| path.poses.iter().map(|p| {
            let p = p.pose.position; [p.x, p.y, p.z]
        }).collect()).unwrap_or_default()
    }
    fn step(&mut self, width: f64) -> PlannerStep {
        self.wait_reason = None;
        if self.route.is_none() { return PlannerStep::Pending(Vec::new()); }
        if self
            .map
            .as_ref()
            .is_some_and(|map| map.body_envelope().is_none())
        {
            return if self.admitted_generation == Some(self.route_generation) {
                self.controller.step(width)
            } else { PlannerStep::Pending(Vec::new()) };
        }
        if let Some(reason) = self.controller.current_body_error() {
            // 起点缺少合法支撑或净空时，重发同一全局路线只会造成规划忙循环。
            // 原实际速度回执、输入期限和取消仍由执行调度器处理。
            self.wait_reason = Some(format!(
                "当前机体支撑或净空尚不可用，保持零速等待地图或位姿恢复：{reason}"
            ));
            return PlannerStep::Pending(Vec::new());
        }
        self.advance_progress();
        if !self.config.local_planning_enabled {
            return self.controller.step(width);
        }
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
                    self.local_failures = vec![reason, local];
                    if self.local_failures.iter().any(|f| f.kind == nuwax_nav_scan::LocalPlanFailureKind::Unclassified) {
                        return PlannerStep::ReplanRequired(format!("{}；近场绕行：{}",
                            self.local_failures[0], self.local_failures[1]));
                    }
                    if self.admitted_generation == Some(self.route_generation) {
                        // Only a fresh controller evaluation can authorize continued motion.
                        return self.controller.step(width);
                    }
                    self.wait_reason = Some("近场搜索尚未取得可执行轨迹，保持零速等待".into());
                    return PlannerStep::Pending(self.local_failures.clone());
                }
            }
        }
        match self.controller.step(width) {
            PlannerStep::ReplanRequired(reason) => match self.regenerate(true) {
                Ok(()) => self.controller.step(width),
                Err(local) => {
                    self.local_failures = vec![local];
                    // Preserve the already fresh controller rejection unchanged.
                    PlannerStep::ReplanRequired(reason)
                }
            },
            step => step,
        }
    }
    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }
    fn stop(&mut self) {
        self.wait_reason = None;
        self.revoke_local_admission();
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
    fn comparison_config(enabled: Option<bool>) -> SurfaceFollowerConfig {
        let mut config = SurfaceFollowerConfig::new(0.4);
        if let Some(enabled) = enabled { config.local_planning_enabled = enabled; }
        config.controller.simulation = true;
        config.controller.speed = 0.5;
        config.motion_limits = Some(nuwax_nav_types_messages::MotionLimits {
            model: MotionModel::Differential, max_forward_mps: 0.5, max_reverse_mps: 0.0,
            max_lateral_mps: 0.0, max_angular_rps: 0.8, linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.05,
            minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
        });
        config
    }
    fn comparison_map(blocked: bool, observed_ns: Option<u64>) -> Arc<SurfaceSnapshot> {
        let map = crate::surface_follower::test_snapshot(0.0, blocked)
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
                max_step_up_m: 0.2, max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
            }).unwrap();
        if let Some(observed_ns) = observed_ns {
            use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
            let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
            for (x,y,z) in map.cell_keys() {
                let key = [x,y,z];
                blocks.entry(key.map(|v| v.div_euclid(32))).or_default().insert(key,
                    SurfaceObservation { observed_ns, hits: 2 });
            }
            let mut observations = SurfaceObservations::new(map.source_voxel_size_m(),32).unwrap();
            for (key,block) in blocks { observations.replace_block(key,block).unwrap(); }
            Arc::new(map.with_observations(Some(Arc::new(observations))))
        } else { Arc::new(map) }
    }
    fn comparison_pose(x: f64, tick: u32) -> PoseStamped {
        PoseStamped { frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1 + i64::from(tick / 20), tick % 20 * 50_000_000),
            pose: Pose3 { position: Point3::new(x,1.0,0.5), orientation: Quaternion::IDENTITY } }
    }
    fn comparison_route(follower: &mut RouteFollower, start: PoseStamped, goal: PoseStamped) {
        follower.update_odometry(start.clone());
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses: vec![start,goal.clone()] }), None,goal).unwrap();
    }
    // Seed exact valid requests with deterministic failures to exercise runtime provenance
    // independently of search timing; scan tests exercise the actual interrupted search.
    fn cache_budget_failures(follower: &mut RouteFollower) {
        follower.advance_progress();
        let map = follower.map.as_ref().unwrap().clone();
        let pose = follower.pose.as_ref().unwrap().clone();
        if follower.local_map.as_ref().is_none_or(|(previous,center,_)|
            !nuwax_nav_pct::same_source_geometry(previous,&map) || center.distance(pose.pose.position) >= 0.25) {
            let local = Arc::new(nuwax_nav_pct::Tomogram::local_from_surface(&map,
                [pose.pose.position.x,pose.pose.position.y], follower.scan.horizon_m + 1.5).unwrap());
            follower.local_map = Some((map.clone(), pose.pose.position, local));
        }
        let tomogram = follower.local_map.as_ref().unwrap().2.clone();
        follower.failed_local.clear();
        for detour in [false,true] {
            let (mut reference, _, at_end, straight) = follower.reference(follower.scan.horizon_m).unwrap();
            let arrival = at_end.then(|| (body(follower.goal.as_ref().unwrap()).0, follower.options.position_tolerance_m));
            let final_yaw = (at_end && follower.options.requires_orientation())
                .then(|| follower.goal.as_ref().unwrap().pose.orientation.yaw());
            if detour && reference.len() > 1 {
                if let Ok(route) = tomogram.plan_bounded(reference[0], *reference.last().unwrap(),
                    tomogram.resolution, &follower.boundary, None, Some(follower.scan.horizon_m*1.5),
                    &follower.planner.local_costs()) { reference = route.points; }
            }
            follower.failed_local.push(FailedLocalPlan {
                map: map.clone(), tomogram: tomogram.clone(), boundary: follower.boundary.clone(),
                start: body(&pose), reference, final_yaw, arrival,
                failure: LocalPlanError { kind: nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted,
                    closest_pose: Some(body(&pose)), blocked: None, reason: "test budget exhausted".into() },
                follow_reference: detour, prefer_straight: straight && !detour,
            });
        }
    }
    fn open_follower() -> RouteFollower {
        let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
        comparison_route(&mut follower, comparison_pose(0.5,0), comparison_pose(3.0,0));
        follower
    }
    #[test]
    fn incomplete_attempts_keep_goal_route_and_typed_evidence_without_penalty() {
        let mut follower = open_follower();
        cache_budget_failures(&mut follower);
        follower.failed_local[1].failure.kind = nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted;
        let goal = follower.goal.clone();
        let route = follower.route.clone();
        for _ in 0..3 {
            let PlannerStep::Pending(failures) = follower.step(0.3) else { panic!("no admitted trajectory must wait"); };
            assert_eq!(failures.len(), 2);
            assert_eq!(failures[0].kind, nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted);
            assert_eq!(failures[1].kind, nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted);
            assert!(failures.iter().all(|f| f.blocked.is_none()));
            assert_eq!(follower.failed_local.len(), 2);
            assert_eq!(follower.goal, goal);
            assert_eq!(follower.route, route);
            assert!(follower.local_path_for_display().is_empty());
            assert!(follower.planner.local_costs().is_empty());
        }
    }
    #[test]
    fn unclassified_faults_do_not_become_incomplete_search_waits() {
        let mut follower = open_follower();
        cache_budget_failures(&mut follower);
        follower.failed_local[0].failure = "invalid local input".into();
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(reason)
            if reason.contains("invalid local input")));
        assert_eq!(follower.local_failures()[0].kind, nuwax_nav_scan::LocalPlanFailureKind::Unclassified);
        assert_eq!(follower.local_failures()[1].kind, nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted);
    }
    #[test]
    fn unavailable_kinematics_is_a_real_preparation_error_not_pending() {
        let mut follower = open_follower();
        follower.config.motion_limits = None;
        follower.config.basic_kinematics = None;
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(reason)
            if reason.contains("缺少运动能力")));
        assert_eq!(follower.local_failures().len(),2);
        assert!(follower.local_failures().iter().all(|f|
            f.kind == nuwax_nav_scan::LocalPlanFailureKind::Unclassified));
    }
    #[test]
    fn cache_key_preserves_signed_zero_and_quaternion_antipodes() {
        let pose = ([0.0,1.0,2.0], [0.0,0.0,0.0,1.0]);
        let mut changed = pose;
        changed.0[0] = -0.0;
        assert_ne!(pose_bits(pose), pose_bits(changed));
        changed = pose;
        changed.1 = changed.1.map(|x| -x);
        assert_ne!(pose_bits(pose), pose_bits(changed));
        assert!(!exact_points(&[[0.0,1.0]], &[[-0.0,1.0]]));
    }
    #[test]
    fn replacement_cannot_reuse_old_motion_arrival_or_display_on_budget_failure() {
        for at_old_goal in [false,true] {
            let mut follower = open_follower();
            assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
            assert!(!follower.local_path_for_display().is_empty());
            if at_old_goal { follower.update_odometry(comparison_pose(3.0,1)); }
            let new_goal = comparison_pose(4.0,1);
            // Replacing a route directly is also a new admission generation.
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses: vec![follower.pose.clone().unwrap(),new_goal.clone()]
            }), None, new_goal.clone()).unwrap();
            cache_budget_failures(&mut follower);
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            assert_eq!(follower.goal.as_ref(), Some(&new_goal));
            assert!(follower.local_path_for_display().is_empty());
            assert!(follower.local_path().is_none());
            follower.configure_goal(&comparison_pose(5.0,2), GoalOptions::default()).unwrap();
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            assert!(follower.local_path_for_display().is_empty());
        }
    }
    #[test]
    fn current_admission_keeps_fresh_motion_arrival_and_boundary_rejection() {
        for outcome in 0..3 {
            let mut follower = open_follower();
            assert!(matches!(follower.step(0.3), PlannerStep::Command(_)));
            if outcome == 1 { follower.update_odometry(comparison_pose(3.0,1)); }
            if outcome == 2 {
                follower.configure_boundary(&[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
            }
            cache_budget_failures(&mut follower);
            follower.dirty = true;
            let step = follower.step(0.3);
            match outcome {
                0 => assert!(matches!(step,PlannerStep::Command(c) if c.linear.x > 0.0)),
                1 => assert_eq!(step,PlannerStep::Succeeded),
                _ => assert!(matches!(step,PlannerStep::ReplanRequired(_)), "fresh boundary rejection lost: {step:?}"),
            }
            assert_eq!(follower.local_failures().len(),2);
            assert!(follower.planner.local_costs().is_empty());
        }
    }
    #[test]
    fn failed_reconnection_preserves_the_existing_fresh_controller_rejection_verbatim() {
        let prepare = || {
            let mut follower = open_follower();
            follower.step(0.3);
            follower.configure_boundary(&[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
            cache_budget_failures(&mut follower);
            follower
        };
        // Use an independent controller because rejection itself starts braking state.
        let mut oracle = prepare();
        let expected = oracle.controller.step(0.3);
        assert!(matches!(expected,PlannerStep::ReplanRequired(_)));
        let mut follower = prepare();
        assert!(!follower.dirty);
        assert_eq!(follower.step(0.3), expected);
        assert_eq!(follower.local_failures().len(),1);
    }
    #[test]
    fn failed_attempt_cache_requires_exact_owner_pose_boundary_source_and_semantics() {
        for change in 0..7 {
            let mut follower = open_follower();
            cache_budget_failures(&mut follower);
            assert_eq!(follower.regenerate(false).unwrap_err().reason, "test budget exhausted");
            match change {
                0 => {
                    let (source,center,_) = follower.local_map.take().unwrap();
                    let owner = Arc::new(nuwax_nav_pct::Tomogram::local_from_surface(&source,
                        [center.x,center.y], follower.scan.horizon_m+1.5).unwrap());
                    follower.local_map = Some((source,center,owner));
                }
                1 => follower.update_odometry(comparison_pose(0.51,1)),
                2 => follower.boundary = vec![[0.0,0.0],[6.0,0.0],[6.0,3.0],[0.0,3.0]],
                3 => follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap(),
                4 => follower.options.position_tolerance_m *= 0.9,
                5 => follower.goal.as_mut().unwrap().pose.position.x += 0.1,
                _ => { let mut pose = follower.pose.clone().unwrap();
                    pose.pose.orientation.x = -0.0; follower.update_odometry(pose); }
            }
            follower.regenerate(false).expect("changed dependencies cannot reuse a cached failure");
            assert!(follower.failed_local.len() <= 2);
        }
    }

    #[test]
    fn local_planning_default_and_explicit_on_match_and_off_progresses_without_planning() {
        let mut traces = Vec::new();
        for enabled in [None,Some(true),Some(false)] {
            let mut follower = RouteFollower::new(comparison_config(enabled)).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
            let mut actual = comparison_pose(0.5,0);
            comparison_route(&mut follower, actual.clone(),comparison_pose(3.0,0));
            let mut commands = Vec::new();
            let mut arrived = false;
            for tick in 1..=140 {
                let step = follower.step(0.3);
                if step == PlannerStep::Succeeded { arrived = true; break; }
                let PlannerStep::Command(command) = step else { panic!("clear route must progress: mode={enabled:?} tick={tick} pose={actual:?} step={step:?}"); };
                assert!(command.linear.x >= 0.0);
                commands.push(command);
                actual.pose.position.x += command.linear.x * 0.05;
                actual.stamp = comparison_pose(0.0,tick).stamp;
                follower.observe_applied_command(command);
                follower.update_odometry(actual.clone());
            }
            assert!(arrived, "mode {enabled:?} did not reach its goal");
            assert!(actual.pose.position.x > 0.8, "mode {enabled:?} did not move");
            assert!(follower.progress_m() > 0.2);
            if enabled == Some(false) {
                assert_eq!(follower.local_planning_calls(),0);
                assert_eq!(follower.local_plans(),0);
                assert!(follower.local_map.is_none() && follower.local_path.is_none());
                assert!(follower.regenerate(false).is_err());
                assert_eq!(follower.local_planning_calls(),0);
            } else { assert!(follower.local_planning_calls() > 0); }
            traces.push(commands);
            follower.stop();
            assert!(!matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0));
        }
        assert_eq!(traces[0],traces[1],"default must preserve explicit ON behavior");
    }
    #[test]
    fn both_modes_keep_obstacle_braking_and_sparse_route_deviation_checks() {
        for enabled in [true,false] {
            let clear = comparison_map(false,None);
            let mut points = Vec::new();
            for ix in 0..65 { for iy in 0..35 {
                points.push(((ix as f32+0.5)*0.1,(iy as f32+0.5)*0.1,0.05));
            } }
            for iy in 0..35 { for iz in 1..8 { points.push((0.75,(iy as f32+0.5)*0.1,(iz as f32+0.5)*0.1)); } }
            let blocked = Arc::new(SurfaceSnapshot::from_points(&points,clear.config().clone(),0.1).unwrap()
                .with_body_envelope(clear.body_envelope().unwrap().clone()).unwrap());
            let mut follower = RouteFollower::new(comparison_config(Some(enabled))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();
            comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
            assert!(matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x>0.0));
            // The physical receipt remains authoritative even when planning is off.
            follower.observe_applied_command(Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0});
            follower.update_odometry(comparison_pose(0.5,1));
            follower.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
            let step = follower.step(0.3);
            assert!(!matches!(step,PlannerStep::Command(c) if c.linear.x>0.0),"new wall must stop the braking envelope: mode={enabled} {step:?}");
            if !enabled { assert_eq!(follower.local_planning_calls(),0); }
        }
        let mut follower = RouteFollower::new(comparison_config(Some(false))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
        comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
        let mut off_route = comparison_pose(1.5,1);
        off_route.pose.position.y = 2.5;
        follower.update_odometry(off_route);
        assert!(matches!(follower.step(0.3),PlannerStep::ReplanRequired(_)));
        assert_eq!(follower.local_planning_calls(),0);
    }

    #[test]
    fn off_sparse_route_rejects_a_valid_body_on_a_different_floor() {
        let clear = comparison_map(false,None);
        let mut points = Vec::new();
        for ix in 0..65 { for iy in 0..35 { for z in [0.05,1.55] {
            points.push(((ix as f32+0.5)*0.1,(iy as f32+0.5)*0.1,z));
        } } }
        let floors = Arc::new(SurfaceSnapshot::from_points(&points,clear.config().clone(),0.1).unwrap()
            .with_body_envelope(clear.body_envelope().unwrap().clone()).unwrap());
        let mut follower = RouteFollower::new(comparison_config(Some(false))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(floors)).unwrap();
        comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
        let mut upper_floor = comparison_pose(1.5,1);
        upper_floor.pose.position.z = 2.0;
        follower.update_odometry(upper_floor);
        assert!(follower.controller.current_body_error().is_none(),"upper-floor pose must have valid support and clearance");
        assert!(matches!(follower.step(0.3),PlannerStep::ReplanRequired(reason) if reason.contains("偏离连续三维路线")),
            "a valid body on another floor is not on the lower route's 3D segment");
        assert_eq!(follower.local_planning_calls(),0);
    }

    #[test]
    fn both_modes_reject_changed_support_and_expired_observations() {
        for enabled in [true,false] {
            let mut follower = RouteFollower::new(comparison_config(Some(enabled))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
            comparison_route(&mut follower,comparison_pose(0.95,0),comparison_pose(3.0,0));
            assert!(matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0));
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(true,None))).unwrap();
            follower.update_odometry(comparison_pose(0.95,1));
            assert!(!matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"changed support allowed motion, mode={enabled}");
            let mut config = comparison_config(Some(enabled));
            config.max_observation_age_sec = Some(1.0);
            let mut follower = RouteFollower::new(config).unwrap();
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,Some(now)))).unwrap();
            comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
            assert!(matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"fresh evidence should move, mode={enabled}");
            // A replacement source carries honestly older observations; no future timestamps or relaxed TTL.
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,Some(now.saturating_sub(2_000_000_000))))).unwrap();
            follower.update_odometry(comparison_pose(0.5,1));
            assert!(!matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"expired evidence allowed motion, mode={enabled}");
            if !enabled { assert_eq!(follower.local_planning_calls(),0); }
        }
    }

    #[test]
    fn invalid_current_support_waits_without_replanning_and_recovers_on_pose_update() {
        let map = Arc::new(crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
                max_step_up_m: 0.2, max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
            }).unwrap());
        let make = |x,z| PoseStamped { frame_id: NavFrameId::Map, stamp: Timestamp::from_unix(1,0),
            pose: Pose3 { position: Point3::new(x,1.0,z), orientation: Quaternion::IDENTITY } };
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.simulation = true;
        config.basic_kinematics = Some(MotionKinematics { model: MotionModel::Differential,
            max_forward_mps: 1.0, max_reverse_mps: 0.0, max_lateral_mps: 0.0,
            max_angular_rps: 0.8, minimum_turning_radius_m: 0.0, can_rotate_in_place: true });
        let mut follower = RouteFollower::new(config).unwrap();
        let start = make(0.5,0.5);
        let goal = make(3.0,0.5);
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(make(0.5,2.0));
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses: vec![start.clone(),goal.clone()] }), None, goal).unwrap();
        for _ in 0..3 {
            assert_eq!(follower.step(0.3), PlannerStep::Pending(Vec::new()));
            assert!(follower.execution_wait_reason().unwrap().contains("支撑或净空"));
            assert_eq!(follower.local_plans(),0);
            assert!(follower.failed_local.is_empty());
        }
        follower.update_odometry(start.clone());
        assert!(matches!(follower.step(0.3), PlannerStep::Command(command) if command != Twist2::ZERO));
        assert!(follower.execution_wait_reason().is_none());
        assert_eq!(follower.local_plans(),1);
        // 中途输入再次失效时也保留同一参考，恢复后不重复生成局部路线。
        follower.update_odometry(make(0.5,2.0));
        assert_eq!(follower.step(0.3), PlannerStep::Pending(Vec::new()));
        follower.update_odometry(start);
        assert!(matches!(follower.step(0.3), PlannerStep::Command(command) if command != Twist2::ZERO));
        assert_eq!(follower.local_plans(),1);
        follower.stop();
        assert!(follower.execution_wait_reason().is_none());
        assert!(follower.route.is_none());
    }

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
    fn disconnected_forward_window_defers_to_fresh_controller_without_inventing_obstruction() {
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
        // 目标须在默认五米窗口之外，本例才会覆盖滚动更新而非末段跟随。
        let goal = make(6.0);
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
        assert!(follower.local_end < *follower.cumulative.last().unwrap());
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
        let outcome = follower.step(0.2);
        assert!(matches!(outcome, PlannerStep::Command(_) | PlannerStep::ReplanRequired(_)),
            "a current admitted route must use a fresh controller outcome: {outcome:?}");
        assert!(!follower.local_failures().is_empty());
        assert!(follower.local_failures().iter().all(|failure| failure.blocked.is_none()));
        assert!(follower.planner.local_costs().is_empty(), "search failure must not penalize PCT");
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
