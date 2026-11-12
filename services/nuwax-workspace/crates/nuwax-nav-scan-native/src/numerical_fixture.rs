// 数值回归夹具：显式驱动三个新接口，不进入旧单会话或许可路径。
use super::*;
#[derive(Clone, Copy)]
pub struct PlanRequest<'a> {
    /// 实测机身障碍体素中心，正常地面和可走台阶应归入 ground。
    pub obstacles: &'a [[f64; 3]],
    /// 地面实测坐标；缺测处沿用全局机身参考高度。
    pub ground: &'a [[f64; 3]],
    /// PCT（点云地形规划器）输出的世界坐标机体中心参考路径。
    pub reference: &'a [[f64; 3]],
    pub start_position: [f64; 3],
    pub start_velocity: Option<[f64; 3]>,
    pub start_acceleration: Option<[f64; 3]>,
    pub start_yaw: f64,
    pub start_yaw_rate: Option<f64>,
    /// 里程计采样时间；重复同一采样不得推进原控制器时钟。
    pub odometry_timestamp_ns: i64,
    pub config: Config,
    pub boundary: &'a [[f64; 2]],
    pub forbidden_columns: &'a [[i32; 2]],
    pub goal_yaw: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct PlanResult {
    pub samples: Vec<TrajectorySample>,
    pub duration: f64,
    /// 当前局部窗口目标在原折线参考轨迹中的时间位置。
    pub reference_progress: f64,
    pub reference_duration: f64,
    pub local_target: [f64; 3],
    pub command_forward: f64,
    pub command_lateral: f64,
    pub command_yaw_rate: f64,
    pub command_duration: f64,
    pub replanned: bool,
    /// Original normal trajectory cycle ended; outer goal may still require a new path.
    pub execution_cycle_complete: bool,
    pub execution_failed: bool,
    /// Velocity, acceleration, yaw-rate provenance; never equates missing measurement with measured zero.
    pub motion_boundary_sources: [MotionBoundarySource; 3],
}

pub struct Planner {
    map: Option<MapOwner>,
    planner: ScanPlanner,
    tracker: ScanTracker,
    curve: Option<Trajectory>,
    snapshot: Option<CollisionSnapshot>,
    config: Option<Config>,
    static_map: bool,
    registered: bool,
    previous: [f64; 3],
    sample_previous: [f64; 3],
    stamp: Option<i64>,
    pub phase: f64,
    travel: [f64; 3],
    travel_stamp: Option<i64>,
    plans: u64,
    early: bool,
    changed: bool,
    reference: Vec<[f64; 3]>,
    progress: f64,
}
impl Planner {
    pub fn new() -> Self {
        Self {
            map: None,
            planner: ScanPlanner::default(),
            tracker: ScanTracker::default(),
            curve: None,
            snapshot: None,
            config: None,
            static_map: false,
            registered: false,
            previous: [0.; 3],
            sample_previous: [0.; 3],
            stamp: None,
            phase: 0.,
            travel: [0.; 3],
            travel_stamp: None,
            plans: 0,
            early: false,
            changed: false,
            reference: Vec::new(),
            progress: 0.,
        }
    }
    pub fn set_static_map_fixture(&mut self, value: bool) {
        self.static_map = value;
    }
    pub fn notify_source_changed(&mut self) {
        self.changed = true;
    }
    pub fn progress_m(&self) -> f64 {
        self.progress
    }
    pub fn observe_applied(&mut self, command: [f64; 3]) {
        self.previous = command;
    }
    pub fn pose_clearance(&self, position: [f64; 3], yaw: f64) -> PoseClearance {
        self.map.as_ref().unwrap().snapshot().pose_clearance(position, yaw)
    }
    pub fn plan_count(&self) -> u64 {
        self.plans
    }
    pub fn original_samples(&self, dt: f64) -> Vec<TrajectorySample> {
        self.curve.as_ref().unwrap().samples(0., dt)
    }
    pub fn reset_execution(&mut self) {
        self.curve = None;
        self.tracker.clear();
        self.stamp = None;
        self.reference.clear();
        self.progress = 0.;
        self.travel_stamp = None;
        self.planner = ScanPlanner::default();
    }
    pub fn reset(&mut self) {
        self.reset_execution();
        self.map = None;
        self.registered = false;
        self.config = None;
        self.planner = ScanPlanner::default();
        self.tracker = ScanTracker::default();
    }
    pub fn configure(&mut self, c: &Config) -> Result<bool, String> {
        let rebuild = self.config.is_some_and(|old| {
            old.resolution != c.resolution
                || old.body_width != c.body_width
                || old.body_to_ground != c.body_to_ground
        });
        if self.map.is_none() || rebuild {
            self.map = Some(MapOwner::new(c)?);
            self.registered = false;
        }
        self.config = Some(*c);
        Ok(rebuild)
    }
    pub fn ingest_registered_frame(
        &mut self,
        c: &Config,
        points: &[[f64; 3]],
        origin: [f64; 3],
        body: [f64; 3],
        stamp: i64,
    ) -> Result<(), String> {
        self.ingest_registered_frame_with_ground(c, points, &[], origin, body, stamp)
    }
    pub fn ingest_registered_frame_with_ground(
        &mut self,
        c: &Config,
        points: &[[f64; 3]],
        members: &[u32],
        origin: [f64; 3],
        body: [f64; 3],
        _: i64,
    ) -> Result<(), String> {
        self.configure(c)?;
        self.map
            .as_mut()
            .unwrap()
            .ingest(points, members, origin, body)?;
        self.registered = true;
        Ok(())
    }
    pub fn step(&mut self, r: &PlanRequest<'_>) -> Result<PlanResult, String> {
        self.configure(&r.config)?;
        let map = self.map.as_mut().unwrap();
        if self.static_map {
            map.seed_occupied(
                &r.obstacles
                    .iter()
                    .chain(r.ground)
                    .copied()
                    .collect::<Vec<_>>(),
                r.start_position,
            );
            map.update_surface(&[], &[], r.boundary, r.forbidden_columns)?;
        } else {
            if !self.registered {
                return Err("registered source frame required".into());
            }
            map.update_surface(r.obstacles, r.ground, r.boundary, r.forbidden_columns)?;
        }
        let snapshot = map.snapshot();
        if self.stamp != Some(r.odometry_timestamp_ns) {
            self.sample_previous = if self.stamp.is_none() {
                let v = r.start_velocity.unwrap_or([0.; 3]);
                let (cs, sn) = (r.start_yaw.cos(), r.start_yaw.sin());
                [
                    cs * v[0] + sn * v[1],
                    -sn * v[0] + cs * v[1],
                    r.start_yaw_rate.unwrap_or(0.),
                ]
            } else {
                self.previous
            };
            self.stamp = Some(r.odometry_timestamp_ns);
        }
        if self.reference != r.reference {
            self.curve = None;
            self.tracker.clear_trajectory();
            self.reference = r.reference.to_vec();
            self.progress = 0.;
        }
        let mut along = 0.;
        let mut best = (f64::INFINITY, self.progress);
        for pair in r.reference.windows(2) {
            let v = std::array::from_fn::<_, 3, _>(|i| pair[1][i] - pair[0][i]);
            let len = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            if len > 0. {
                let t = ((0..3)
                    .map(|i| (r.start_position[i] - pair[0][i]) * v[i])
                    .sum::<f64>()
                    / (len * len))
                    .clamp(0., 1.);
                let arc = along + t * len;
                if arc + 1e-8 >= self.progress {
                    let distance = (0..3)
                        .map(|i| (r.start_position[i] - pair[0][i] - t * v[i]).powi(2))
                        .sum::<f64>();
                    if distance < best.0 {
                        best = (distance, arc);
                    }
                }
            }
            along += len;
        }
        self.progress = best.1;
        let measured = r.start_velocity.unwrap_or([0.; 3]);
        if let Some(previous) = self.travel_stamp {
            if r.odometry_timestamp_ns > previous {
                let weight = -(-((r.odometry_timestamp_ns - previous) as f64) * 1e-9).exp_m1();
                for i in 0..2 {
                    self.travel[i] += weight * (measured[i] - self.travel[i]);
                }
                self.travel[2] = measured[2];
            }
        } else {
            self.travel = measured;
        }
        self.travel_stamp = Some(r.odometry_timestamp_ns);
        let state = MotionState {
            position: r.start_position,
            velocity: r.start_velocity.unwrap_or([0.; 3]),
            planning_velocity: self.travel,
            yaw: r.start_yaw,
            yaw_rate: r.start_yaw_rate.unwrap_or(0.),
            route_progress: self.progress,
            timestamp_ns: r.odometry_timestamp_ns,
            measured_velocity: u32::from(r.start_velocity.is_some()),
            measured_yaw_rate: u32::from(r.start_yaw_rate.is_some()),
        };
        // 无参考的数值算例显式调用原地制动控制，生产无轨迹入口直接零速。
        let terminal_yaw = r
            .goal_yaw
            .or_else(|| r.reference.is_empty().then_some(r.start_yaw));
        let previous_plans = self.plans;
        let mut tracked = self.tracker.track_and_check(
            &r.config,
            &snapshot,
            &state,
            self.sample_previous,
            terminal_yaw,
        )?;
        if tracked.recovery_samples.is_empty()
            && r.reference.len() >= 2
            && r.goal_yaw.is_none()
            && (self.curve.is_none()
                || tracked.invalid()
                || (!self.early && tracked.replan_early())
                || self.changed
                    && self.snapshot.as_ref().is_some_and(|old| {
                        self.curve
                            .as_ref()
                            .is_some_and(|c| snapshot.affects(old, c, self.phase))
                    }))
        {
            if tracked.replan_early() {
                self.early = true;
            }
            let result = self.planner.plan(
                &r.config,
                &snapshot,
                r.reference,
                &state,
                self.curve.as_ref(),
                tracked.phase,
                None,
            );
            self.changed = false;
            match result {
                Ok(curve) => {
                    self.tracker.install(&curve);
                    self.curve = Some(curve);
                    self.plans += 1;
                    tracked = self.tracker.track_and_check(
                        &r.config,
                        &snapshot,
                        &state,
                        self.sample_previous,
                        terminal_yaw,
                    )?;
                }
                Err(reason) => {
                    if reason.contains("reference")
                        || reason.contains("参考")
                        || reason.contains("waypoint")
                        || reason.contains("two distinct consecutive points")
                    {
                        return Err(reason);
                    }
                }
            }
        }
        if tracked.no_candidate() {
            return Err("Current motion and braking sweep intersect the body collision map".into());
        }
        self.phase = tracked.phase;
        self.previous = tracked.command;
        self.snapshot = Some(snapshot);
        let view = self
            .curve
            .as_ref()
            .map(|c| c.inspect(tracked.phase, r.config.sample_dt));
        let samples = if !tracked.recovery_samples.is_empty() {
            tracked.recovery_samples.clone()
        } else {
            view.as_ref().map_or_else(Vec::new, |v| v.samples.clone())
        };
        let out = PlanResult {
            samples,
            duration: view.as_ref().map_or(0., |v| v.duration),
            reference_progress: view.as_ref().map_or(0., |v| v.reference_progress),
            reference_duration: view.as_ref().map_or(0., |v| v.reference_duration),
            local_target: view.as_ref().map_or([0.; 3], |v| v.local_target),
            command_forward: tracked.command[0],
            command_lateral: tracked.command[1],
            command_yaw_rate: tracked.command[2],
            command_duration: r.config.sample_dt,
            replanned: self.plans > previous_plans,
            execution_cycle_complete: false,
            execution_failed: false,
            motion_boundary_sources: [
                view.as_ref()
                    .map_or(MotionBoundarySource::InitialZeroAssumption, |v| {
                        v.motion_boundary_sources[0]
                    }),
                view.as_ref()
                    .map_or(MotionBoundarySource::InitialZeroAssumption, |v| {
                        v.motion_boundary_sources[1]
                    }),
                if r.start_yaw_rate.is_some() {
                    MotionBoundarySource::Measured
                } else {
                    MotionBoundarySource::PreviousControlEstimate
                },
            ],
        };
        Ok(out)
    }
}
