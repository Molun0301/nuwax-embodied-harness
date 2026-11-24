//! 采样执行器消费与样条执行器相同的地图工作线程和快照。
use crate::local_workers::{MapInitializer, MapInput, MapWorker, PreparedMap};
use crate::sampling_controller::{PoseGoal, Route, Sampler};
use crate::surface_follower::{
    orientation_within_tolerance, position_within_tolerance, SurfaceFollower, SurfaceFollowerConfig,
};
use crate::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceSnapshot};
use nuwax_core::logging::{CollisionLogPose, DiagnosticLimiter};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_types_messages::{
    clock, GoalOptions, Path2D, Path3D, Point3, PoseStamped, Quaternion, Twist2,
};
use std::{sync::Arc, time::Instant};

const REPLAN_AFTER_NS: i64 = 10_000_000_000;
const FAIL_AFTER_NS: i64 = 30_000_000_000;
const PROGRESS_M: f64 = 0.1;
const BLOCKED: &str = "前方受阻，局部无可行动作";
const NO_PROGRESS: &str = "路线剩余长度未减少，等待全局重规划";

#[derive(Default)]
struct Progress {
    since: Option<i64>,
    last_progress: Option<clock::Instant>,
    remaining: Option<f64>,
    replanned: bool,
}

impl Progress {
    fn observe(&mut self, now: i64, remaining: f64) {
        let best = self.remaining.get_or_insert(remaining);
        self.since.get_or_insert(now);
        self.last_progress.get_or_insert_with(clock::Instant::now);
        if *best - remaining >= PROGRESS_M {
            *best = remaining;
            self.since = Some(now);
            self.last_progress = Some(clock::Instant::now());
            self.replanned = false;
        }
    }

    fn elapsed(&self, now: i64) -> i64 {
        self.since.map_or(0, |since| now - since)
    }
}

/// 生产地图线程独占射线概率图，控制器只消费只读快照。
pub(crate) struct SamplingFollower {
    config: SurfaceFollowerConfig,
    map: Option<Arc<SurfaceSnapshot>>,
    odometry: Option<PoseStamped>,
    goal: Option<PoseStamped>,
    options: GoalOptions,
    boundary: Arc<Vec<[f64; 2]>>,
    reference: Vec<[f64; 3]>,
    route: Route,
    stop: Option<([f64; 3], nuwax_nav_scan_native::CollisionSnapshot)>,
    applied: Twist2,
    registered_frame: Option<Arc<crate::RegisteredFrame>>,
    map_worker: MapWorker,
    prepared: Option<PreparedMap>,
    epoch: u64,
    correction_replan_pending: bool,
    wait_reason: Option<String>,
    progress: Progress,
    sampler: Sampler,
    prediction: Option<Path3D>,
    collision_log: DiagnosticLimiter,
}

impl SamplingFollower {
    /// 静态地图初始化仅由独立数值夹具提供。
    pub fn with_map_initializer(
        config: SurfaceFollowerConfig,
        initializer: Option<MapInitializer>,
    ) -> Result<Self, RuntimeError> {
        SurfaceFollower::validate_config(&config)?;
        Ok(Self {
            config,
            map: None,
            odometry: None,
            goal: None,
            options: GoalOptions::default(),
            boundary: Arc::new(Vec::new()),
            reference: Vec::new(),
            route: Route::default(),
            stop: None,
            applied: Twist2::ZERO,
            registered_frame: None,
            map_worker: MapWorker::new(initializer).map_err(RuntimeError::Planner)?,
            prepared: None,
            epoch: 0,
            correction_replan_pending: false,
            wait_reason: None,
            progress: Progress::default(),
            sampler: Sampler::default(),
            prediction: None,
            collision_log: DiagnosticLimiter::default(),
        })
    }

    fn position(pose: &PoseStamped) -> [f64; 3] {
        let p = pose.pose.position;
        [p.x, p.y, p.z]
    }

    fn position_reached(&self) -> bool {
        let (Some(pose), Some(goal), Some(map)) = (&self.odometry, &self.goal, &self.map) else {
            return false;
        };
        position_within_tolerance(
            Self::position(pose),
            Self::position(goal),
            self.options.position_tolerance_m,
            map,
        )
    }
    fn orientation_reached(&self) -> bool {
        self.odometry
            .as_ref()
            .zip(self.goal.as_ref())
            .is_some_and(|(pose, goal)| orientation_within_tolerance(pose, goal, self.options))
    }
    pub(crate) fn reached(&self) -> bool {
        self.position_reached() && self.orientation_reached()
    }
    fn native_config(&self, map: &SurfaceSnapshot) -> nuwax_nav_scan_native::Config {
        let mut config = self.config.native_config(map, self.options, false);
        config.planning_horizon = config
            .planning_horizon
            .max(config.max_velocity * crate::sampling_controller::PREDICTION_SECONDS);
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
            active: None,
            active_snapshot: None,
            phase: 0.,
        });
    }

    fn retire_route(&mut self) {
        self.epoch += 1;
        self.map_worker.retire(self.epoch, false);
        self.reference.clear();
        self.route = Route::default();
        self.stop = None;
        self.prediction = None;
        self.sampler.yaw_error = None;
        self.wait_reason = None;
    }

    fn update_stop(&mut self, map: &PreparedMap, config: &nuwax_nav_scan_native::Config) {
        if self.reference.is_empty()
            || !self.options.requires_orientation()
            || self
                .stop
                .as_ref()
                .is_some_and(|(_, snapshot)| snapshot.same(&map.snapshot))
        {
            return;
        }
        let goal = self.goal.as_ref().unwrap();
        let stop = map
            .snapshot
            .terminal_points(
                config,
                Self::position(goal),
                goal.pose.orientation.yaw(),
                self.options.position_tolerance_m,
            )
            .stop;
        if self.stop.as_ref().is_none_or(|(old, _)| *old != stop) {
            self.route = Route::new(SurfaceFollower::reference_to_stop(
                &self.reference,
                stop,
                config.body_width / 2.,
            ));
            self.progress.remaining = None;
        }
        self.stop = Some((stop, map.snapshot.clone()));
    }

    fn blocked(&mut self, map: Option<&PreparedMap>, reason: &str, terminal: bool) -> String {
        let pose = self.odometry.as_ref().unwrap();
        let position = Self::position(pose);
        let yaw = pose.pose.orientation.yaw();
        let now = Instant::now();
        let capture =
            terminal || nuwax_core::logging::CollisionEvidence::capture(&self.collision_log, now);
        let evidence = map.filter(|_| capture).map_or_else(
            || {
                nuwax_core::logging::CollisionEvidence::new(
                    position,
                    self.config.body_to_support_z_m,
                )
            },
            |map| {
                self.sampler
                    .blocking_cells(&map.snapshot, &map.config, position, yaw)
            },
        );
        evidence.log_failure(
            &mut self.collision_log,
            now,
            "局部采样控制",
            reason,
            CollisionLogPose::with_yaw(position, Some(yaw)),
            self.goal.as_ref().map(|goal| {
                CollisionLogPose::with_yaw(
                    Self::position(goal),
                    self.options
                        .requires_orientation()
                        .then(|| goal.pose.orientation.yaw()),
                )
            }),
        );
        if terminal {
            format!("{reason}，挡路格 {evidence}")
        } else {
            reason.into()
        }
    }

    /// 末态轨迹展示使用本拍选中命令的预测位姿。
    pub fn local_path(&self) -> Option<&Path3D> {
        self.prediction.as_ref()
    }

    /// 最近一次采样使用的前视角误差。
    pub fn lookahead_yaw_error(&self) -> Option<f64> {
        self.sampler.yaw_error
    }
}

impl NavPlanner for SamplingFollower {
    fn set_control_period(&mut self, period: std::time::Duration) {
        self.config.controller.control_frequency = 1. / period.as_secs_f64();
    }
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: GoalOptions,
    ) -> Result<(), RuntimeError> {
        self.retire_route();
        self.goal = Some(goal.clone());
        self.progress = Progress::default();
        self.sampler = Sampler::default();
        self.correction_replan_pending = false;
        self.options = options;
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
        self.reference = path.poses.iter().map(Self::position).collect();
        self.route = Route::new(self.reference.clone());
        self.progress.remaining = None;
        self.progress
            .last_progress
            .get_or_insert_with(clock::Instant::now);
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
        if frame.is_none() && !self.reference.is_empty() {
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
            self.map_worker.retire(self.epoch, true);
            self.prepared = None;
        }
        self.registered_frame = frame;
        self.submit_map();
    }

    fn refresh_execution_context(
        &mut self,
        applied: Twist2,
    ) -> Option<Arc<crate::replan_feedback::ActualExecutionContext>> {
        Some(Arc::new(crate::replan_feedback::ActualExecutionContext {
            source: self.map.clone()?,
            odometry: self.odometry.clone()?,
            applied,
            measured_motion: None,
        }))
    }

    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }

    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        self.prediction.as_ref().map_or_else(Vec::new, |path| {
            path.poses.iter().map(Self::position).collect()
        })
    }

    fn uses_route_progress(&self) -> bool {
        true
    }

    fn suspended_route_failure(&mut self) -> Option<String> {
        if self.progress.last_progress?.elapsed().as_nanos() < FAIL_AFTER_NS as u128 {
            return None;
        }
        self.odometry.as_ref()?;
        Some(self.blocked(self.prepared.clone().as_ref(), BLOCKED, true))
    }

    fn has_pending_replan(&self) -> bool {
        self.correction_replan_pending
    }

    fn step_at_goal(&mut self) -> Option<PlannerStep> {
        if self.position_reached() {
            return Some(self.step());
        }
        let now = self.odometry.as_ref()?.stamp.nanos;
        if self.progress.elapsed(now) >= FAIL_AFTER_NS {
            let map = self.prepared.clone();
            return Some(PlannerStep::Failed(self.blocked(
                map.as_ref(),
                BLOCKED,
                true,
            )));
        }
        None
    }

    fn step(&mut self) -> PlannerStep {
        self.prediction = None;
        self.sampler.yaw_error = None;
        let position_reached = self.position_reached();
        if position_reached && self.orientation_reached() {
            return PlannerStep::Succeeded;
        }
        if std::mem::take(&mut self.correction_replan_pending) {
            return PlannerStep::ReplanRequired("地图已校正，重新规划路线".into());
        }
        let Some(pose) = self.odometry.clone() else {
            return PlannerStep::Pending(Vec::new());
        };
        let Some(goal) = self.goal.as_ref() else {
            return PlannerStep::Pending(Vec::new());
        };
        let (position, yaw, now) = (
            Self::position(&pose),
            pose.pose.orientation.yaw(),
            pose.stamp.nanos,
        );
        let goal_position = Self::position(goal);
        let pose_goal = self.options.requires_orientation().then(|| PoseGoal {
            position: goal_position,
            yaw: goal.pose.orientation.yaw(),
            position_tolerance_m: self.options.position_tolerance_m,
            position_reached,
        });
        if self.route.is_empty() && !position_reached {
            return PlannerStep::Pending(Vec::new());
        }
        if let Some(result) = self.map_worker.take() {
            match result {
                Ok(map) if map.epoch == self.epoch => self.prepared = Some(map),
                Ok(_) => {}
                Err(reason) => {
                    self.prepared = None;
                    self.wait_reason = Some(reason);
                }
            }
        }
        let map = self.prepared.clone().filter(|map| map.epoch == self.epoch);
        if let Some(map) = &map {
            let config = self.native_config(&map.source);
            self.update_stop(map, &config);
        }
        let remaining = if self.route.is_empty() {
            0.
        } else {
            self.route.length() - self.route.nearest(position).0
        };
        self.progress.observe(now, remaining);
        let elapsed = self.progress.elapsed(now);
        if elapsed >= FAIL_AFTER_NS {
            return PlannerStep::Failed(self.blocked(
                self.prepared.clone().as_ref(),
                BLOCKED,
                true,
            ));
        }
        if elapsed >= REPLAN_AFTER_NS && !self.progress.replanned {
            self.progress.replanned = true;
            return PlannerStep::ReplanRequired(NO_PROGRESS.into());
        }
        let Some(map) = map else {
            self.submit_map();
            return PlannerStep::Command(Twist2::ZERO);
        };
        let near_goal;
        let reference = if self.route.is_empty() {
            near_goal = Route::new(vec![goal_position]);
            &near_goal
        } else {
            &self.route
        };
        let config = self.native_config(&map.source);
        if let Some(selection) = self.sampler.select(
            &map.snapshot,
            &config,
            reference,
            position,
            yaw,
            self.applied,
            pose_goal,
        ) {
            self.wait_reason = None;
            self.prediction = Some(Path3D {
                poses: selection
                    .poses
                    .iter()
                    .enumerate()
                    .map(|(i, p)| {
                        let mut predicted = pose.clone();
                        predicted.pose.position = Point3::new(p[0], p[1], p[2]);
                        predicted.pose.orientation = Quaternion::from_yaw(
                            yaw + (i + 1) as f64 * 0.1 * selection.command.angular_z,
                        );
                        predicted
                    })
                    .collect(),
            });
            return PlannerStep::Command(selection.command);
        }
        PlannerStep::ReplanRequired(self.blocked(Some(&map), BLOCKED, false))
    }

    fn stop(&mut self) {
        self.retire_route();
        self.submit_map();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{NavFrameId, Path3D, Point3, Pose3, Timestamp};

    fn pose(x: f64, time: i64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(time, 0),
            pose: Pose3 {
                position: Point3::new(x, 1., 0.5),
                orientation: nuwax_nav_types_messages::Quaternion::IDENTITY,
            },
        }
    }

    fn test_follower() -> SamplingFollower {
        let (config, source) = crate::native_test_helpers::fixture();
        let mut follower = SamplingFollower::with_map_initializer(
            config,
            Some(|map, obstacles, ground, boundary, forbidden, body| {
                map.seed_occupied(
                    &obstacles.iter().chain(ground).copied().collect::<Vec<_>>(),
                    body,
                );
                map.update_surface(&[], &[], boundary, forbidden)
            }),
        )
        .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(source))
            .unwrap();
        let goal = pose(4., 1);
        follower
            .configure_goal(&goal, GoalOptions::default())
            .unwrap();
        follower.update_odometry(pose(0.5, 1));
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![pose(0.5, 1), goal.clone()],
                }),
                None,
                goal,
            )
            .unwrap();
        follower
    }

    #[test]
    fn business_wall_leaves_no_candidate_and_requests_zero_then_replan() {
        let mut follower = test_follower();
        follower.configure_boundary(&[[1., 0.], [4., 0.], [4., 2.], [1., 2.]]);
        let until = Instant::now() + std::time::Duration::from_secs(2);
        loop {
            match follower.step() {
                PlannerStep::ReplanRequired(reason) => {
                    assert_eq!(reason, BLOCKED);
                    break;
                }
                PlannerStep::Command(Twist2::ZERO) => {}
                other => panic!("禁行区内得到了动作 {other:?}"),
            }
            assert!(Instant::now() < until, "地图线程未交付");
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }

    #[test]
    fn route_replacement_does_not_restart_progress_deadline() {
        let mut follower = test_follower();
        follower.step();
        follower.update_odometry(pose(0.51, 11));
        assert_eq!(
            follower.step(),
            PlannerStep::ReplanRequired(NO_PROGRESS.into())
        );
        follower.stop();
        let end = pose(4., 11);
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![pose(0.51, 11), end.clone()],
                }),
                None,
                end,
            )
            .unwrap();
        follower.update_odometry(pose(0.51, 21));
        assert!(
            !matches!(follower.step(), PlannerStep::ReplanRequired(ref reason) if reason == NO_PROGRESS)
        );
        follower.stop();
        follower.update_odometry(pose(0.51, 31));
        assert!(
            matches!(follower.step_at_goal(), Some(PlannerStep::Failed(reason)) if reason.starts_with(BLOCKED))
        );
    }
}
