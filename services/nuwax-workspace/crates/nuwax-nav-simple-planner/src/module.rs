//! Synchronous module boundary for [`crate::SimplePlanner`].
//!
//! A runtime supplies typed inputs and calls [`Module::tick`] at a fixed rate.
//! The adapter keeps transport concerns out of the planner while preserving the
//! original module's costmap, waypoint, path, and hold semantics.

use crate::{
    NavError, Point3, PointCloud, PointStamped, SimplePlanner, SimplePlannerConfig, Timestamp,
};
use nuwax_nav_types_messages::{NavFrameId, OccupancyGrid2D, Path2D, PoseStamped};

/// Recommended lifecycle period (10 Hz).
pub const TICK_PERIOD_MS: u64 = 100;

/// Why the module emitted a stationary safety command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldReason {
    /// `stop_movement=true` cancelled the active goal.
    StopRequested,
    /// A* could not find a path through the current costmap.
    PlanningFailed,
    /// The robot entered the configured goal-reached radius.
    GoalReached,
    /// The lifecycle was stopped while a robot pose was available.
    LifecycleStopped,
}

/// A single lifecycle output.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleOutput {
    /// A fresh A* path and its leading lookahead waypoint.
    Plan {
        /// World-frame A* path.
        path: Path2D,
        /// Leading waypoint for the local planner.
        waypoint: PointStamped,
    },
    /// A refreshed waypoint from the cached path during the replan cooldown.
    Waypoint(PointStamped),
    /// A stationary waypoint plus an explicit diagnostic path.
    SafetyHold {
        /// Trigger for the hold.
        reason: HoldReason,
        /// Robot-position waypoint that commands no further motion.
        waypoint: PointStamped,
        /// One-point path for cancel/reached, or robot-to-goal for plan failure.
        path: Path2D,
    },
    /// No goal is active and no lifecycle output is pending.
    Idle,
}

/// Typed module input.
#[derive(Debug, Clone, PartialEq)]
pub enum ModuleInput {
    /// Robot pose normally supplied by the runtime's TF/odometry adapter.
    Odometry(PoseStamped),
    /// Navigation goal.
    Goal(PointStamped),
    /// Movement-cancellation signal.
    StopMovement(bool),
    /// Fast local terrain layered on top of the persistent costmap.
    TerrainMap(PointCloud),
    /// Persistent terrain snapshot that rebuilds the costmap.
    TerrainMapExt(PointCloud),
}

/// Lifecycle and typed input contract.
pub trait Module: Send + Sync + 'static {
    /// Start processing. Repeated calls while running are idempotent.
    fn start(&mut self) -> Result<(), NavError>;
    /// Stop processing and queue a stationary lifecycle hold.
    fn stop(&mut self) -> Result<(), NavError>;
    /// Update the robot pose.
    fn on_odometry(&mut self, pose: PoseStamped) -> Result<(), NavError>;
    /// Set or replace the active goal.
    fn on_goal(&mut self, goal: PointStamped) -> Result<(), NavError>;
    /// Cancel navigation when `stop` is true.
    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError>;
    /// Layer a fast local terrain snapshot into the current costmap.
    fn on_terrain_map(&mut self, cloud: PointCloud) -> Result<(), NavError>;
    /// Rebuild the costmap from a persistent terrain snapshot.
    fn on_terrain_map_ext(&mut self, cloud: PointCloud) -> Result<(), NavError>;
    /// Advance planning by one lifecycle step.
    fn tick(&mut self) -> Result<ModuleOutput, NavError>;
}

type PlanningResult = (
    SimplePlanner,
    (Option<Option<Vec<crate::Point2>>>, bool, bool),
);
struct PlanningJob {
    handle: std::thread::JoinHandle<PlanningResult>,
    cancelled: std::sync::Arc<std::sync::atomic::AtomicBool>,
    map_generation: u64,
}

/// Lifecycle wrapper around [`SimplePlanner`].
pub struct SimplePlannerModule {
    background_planning: bool,
    planning_job: Option<PlanningJob>,
    map_generation: u64,
    config: SimplePlannerConfig,
    planner: SimplePlanner,
    running: bool,
    clock_seconds: f64,
    last_pose: Option<PoseStamped>,
    active_goal: Option<PointStamped>,
    pending_hold: Option<HoldReason>,
}

impl SimplePlannerModule {
    /// 生产拥有方启用单个后台规划工作，输入及停止仍在原调度线程处理。
    pub fn enable_background_planning(&mut self) {
        self.background_planning = true;
    }

    fn cancel_planning(&mut self) {
        if let Some(job) = &self.planning_job {
            job.cancelled
                .store(true, std::sync::atomic::Ordering::Relaxed);
        }
    }
    fn finish_planning(&mut self) -> Result<(), NavError> {
        self.cancel_planning();
        if let Some(job) = self.planning_job.take() {
            job.handle
                .join()
                .map_err(|_| NavError::Internal("二维规划工作线程异常退出".into()))?;
        }
        Ok(())
    }
    fn planning_step(
        &mut self,
    ) -> Result<(Option<Option<Vec<crate::Point2>>>, bool, bool), NavError> {
        if !self.background_planning {
            return Ok(self.planner.replan_once(self.clock_seconds));
        }
        if self
            .planning_job
            .as_ref()
            .is_some_and(|job| job.handle.is_finished())
        {
            let job = self.planning_job.take().expect("已完成工作存在");
            let (computed, result) = job
                .handle
                .join()
                .map_err(|_| NavError::Internal("二维规划工作线程异常退出".into()))?;
            if !job.cancelled.load(std::sync::atomic::Ordering::Relaxed)
                && (result.0.as_ref().is_some_and(Option::is_some)
                    || job.map_generation == self.map_generation)
                && self.planner.accept_plan(&computed)
            {
                return Ok(result);
            }
        }
        if self.planning_job.is_none()
            && self.active_goal.is_some()
            && self.planner.needs_replan(self.clock_seconds)
        {
            let cancelled = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
            let mut computed = self.planner.clone();
            computed.set_cancellation(cancelled.clone());
            let now = self.clock_seconds;
            let handle = std::thread::Builder::new()
                .name("二维路径规划".into())
                .spawn(move || {
                    let output = computed.replan_once(now);
                    (computed, output)
                })
                .map_err(|error| NavError::Internal(format!("二维规划线程启动失败：{error}")))?;
            self.planning_job = Some(PlanningJob {
                handle,
                cancelled,
                map_generation: self.map_generation,
            });
        }
        Ok((None, false, false))
    }

    /// 当前目标的执行选项在收到位置后显式设置。
    pub fn set_goal_options(
        &mut self,
        options: nuwax_nav_types_messages::GoalOptions,
    ) -> Result<(), NavError> {
        self.cancel_planning();
        self.planner.set_goal_options(options)
    }
    /// 与最近位置路径同序的机体朝向。
    pub fn planned_yaws(&self) -> Option<&[f64]> {
        self.planner.cached_yaws()
    }

    /// 普通目标消息之外保留同一请求的最终朝向。
    pub fn set_goal_yaw(&mut self, yaw: f64) {
        self.cancel_planning();
        self.planner.set_goal_yaw(yaw);
    }

    /// Construct with the default planner configuration.
    #[must_use]
    pub fn new() -> Self {
        Self::with_config(SimplePlannerConfig::default())
            .expect("default SimplePlannerConfig must be valid")
    }

    /// Construct with an explicit planner configuration.
    ///
    /// # Errors
    /// Returns the same validation error as [`SimplePlanner::new`].
    pub fn with_config(config: SimplePlannerConfig) -> Result<Self, NavError> {
        Ok(Self {
            background_planning: false,
            planning_job: None,
            map_generation: 0,
            planner: SimplePlanner::new(config.clone())?,
            config,
            running: false,
            clock_seconds: 0.0,
            last_pose: None,
            active_goal: None,
            pending_hold: None,
        })
    }

    /// Whether the module is accepting input and normal ticks.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// 用已经分类的有界占据图更新规划输入，保留未知区与地图几何。
    pub fn on_occupancy_grid(&mut self, grid: OccupancyGrid2D) -> Result<(), NavError> {
        self.ensure_running()?;
        let update = self.planner.update_occupancy(grid)?;
        tracing::trace!(changed_cells = update.changed_cells, changed_blocks = update.changed_blocks,
            obstacle_changes = update.obstacle_changes, full_rebuild = update.full_rebuild, "二维占据代价更新");
        self.map_generation = self.map_generation.wrapping_add(1);
        Ok(())
    }

    /// Dispatch a typed input through the validated runtime handlers.
    pub fn handle_input(&mut self, input: ModuleInput) -> Result<(), NavError> {
        match input {
            ModuleInput::Odometry(pose) => self.on_odometry(pose),
            ModuleInput::Goal(goal) => self.on_goal(goal),
            ModuleInput::StopMovement(stop) => self.on_stop_movement(stop),
            ModuleInput::TerrainMap(cloud) => self.on_terrain_map(cloud),
            ModuleInput::TerrainMapExt(cloud) => self.on_terrain_map_ext(cloud),
        }
    }

    fn ensure_running(&self) -> Result<(), NavError> {
        if self.running {
            Ok(())
        } else {
            Err(NavError::InvalidInput {
                field: "module",
                reason: "simple-planner module is not running".into(),
            })
        }
    }

    fn safety_hold(&self, reason: HoldReason) -> Result<ModuleOutput, NavError> {
        let pose = self.last_pose.as_ref().ok_or(NavError::InvalidInput {
            field: "odometry",
            reason: "cannot emit a safety hold without a robot pose".into(),
        })?;
        let robot = pose.pose.position.to_2d();
        let frame_id = NavFrameId::from_static(&self.config.world_frame);
        let stamp = pose.stamp;
        let waypoint = PointStamped::new(frame_id.clone(), stamp, pose.pose.position);
        let mut path = vec![robot];
        if reason == HoldReason::PlanningFailed {
            if let Some(goal) = self.active_goal.as_ref() {
                path.push(goal.point.to_2d());
            }
        }
        Ok(ModuleOutput::SafetyHold {
            reason,
            waypoint,
            path: Path2D::stamped(frame_id, stamp, path),
        })
    }
}

impl Drop for SimplePlannerModule {
    fn drop(&mut self) {
        // 错误返回和正常停止均回收实际线程，不能遗留孤立计算。
        if let Err(error) = self.finish_planning() {
            tracing::error!(%error, "二维规划关闭失败");
        }
    }
}

impl Default for SimplePlannerModule {
    fn default() -> Self {
        Self::new()
    }
}

impl Module for SimplePlannerModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(());
        }
        self.planner = SimplePlanner::new(self.config.clone())?;
        self.clock_seconds = 0.0;
        self.last_pose = None;
        self.active_goal = None;
        self.pending_hold = None;
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        if !self.running {
            return Ok(());
        }
        self.finish_planning()?;
        self.planner.cancel_navigation();
        self.active_goal = None;
        if self.last_pose.is_some() {
            self.pending_hold = Some(HoldReason::LifecycleStopped);
        }
        self.running = false;
        Ok(())
    }

    fn on_odometry(&mut self, pose: PoseStamped) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_frame(
            &pose.frame_id,
            &self.config.world_frame,
            "odometry.frame_id",
        )?;
        validate_point(pose.pose.position, "odometry")?;
        if !pose.pose.orientation.w.is_finite()
            || !pose.pose.orientation.x.is_finite()
            || !pose.pose.orientation.y.is_finite()
            || !pose.pose.orientation.z.is_finite()
        {
            return Err(NavError::InvalidInput {
                field: "odometry",
                reason: "orientation contains a non-finite value".into(),
            });
        }
        self.planner.update_odometry(pose.pose);
        self.last_pose = Some(pose);
        Ok(())
    }

    fn on_goal(&mut self, goal: PointStamped) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_frame(&goal.frame_id, &self.config.world_frame, "goal.frame_id")?;
        validate_point(goal.point, "goal")?;
        self.cancel_planning();
        self.planner.set_goal(goal.point, self.clock_seconds);
        self.active_goal = Some(goal);
        self.pending_hold = None;
        Ok(())
    }

    fn on_stop_movement(&mut self, stop: bool) -> Result<(), NavError> {
        self.ensure_running()?;
        if stop {
            self.cancel_planning();
            self.planner.on_stop_movement(true);
            self.active_goal = None;
            self.pending_hold = Some(HoldReason::StopRequested);
        }
        Ok(())
    }

    fn on_terrain_map(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_cloud(&cloud, "terrain_map")?;
        if !cloud.is_empty() {
            self.planner.ingest_local_terrain(&cloud.points);
        }
        Ok(())
    }

    fn on_terrain_map_ext(&mut self, cloud: PointCloud) -> Result<(), NavError> {
        self.ensure_running()?;
        validate_cloud(&cloud, "terrain_map_ext")?;
        self.planner.ingest_world_terrain(&cloud.points);
        Ok(())
    }

    fn tick(&mut self) -> Result<ModuleOutput, NavError> {
        if let Some(reason) = self.pending_hold.take() {
            return self.safety_hold(reason);
        }
        self.ensure_running()?;
        self.clock_seconds += TICK_PERIOD_MS as f64 / 1_000.0;

        if let (Some(pose), Some(goal)) = (self.last_pose.as_ref(), self.active_goal.as_ref()) {
            if self.planner.goal_pose_reached() {
                let map = self.planner.costmap();
                if !map.permits_endpoints(
                    map.world_to_cell(pose.pose.position.x, pose.pose.position.y),
                    map.world_to_cell(goal.point.x, goal.point.y),
                    map.inflation_radius(),
                ) {
                    return self.safety_hold(HoldReason::PlanningFailed);
                }
                self.cancel_planning();
                self.planner.cancel_navigation();
                self.active_goal = None;
                return self.safety_hold(HoldReason::GoalReached);
            }
        }

        let (plan, did_replan, _) = self.planning_step()?;
        match plan {
            Some(Some(path)) => {
                let mut waypoint = self.planner.update_waypoint().ok_or_else(|| {
                    NavError::Internal("planner produced a path without a waypoint".into())
                })?;
                waypoint.stamp = self
                    .last_pose
                    .as_ref()
                    .map_or_else(Timestamp::default, |pose| pose.stamp);
                Ok(ModuleOutput::Plan {
                    path: Path2D::stamped(waypoint.frame_id.clone(), waypoint.stamp, path),
                    waypoint,
                })
            }
            Some(None) => self.safety_hold(HoldReason::PlanningFailed),
            None if did_replan => Err(NavError::Internal(
                "planner failed before producing a path result".into(),
            )),
            None if self.active_goal.is_some() && self.planner.cached_path().is_some() => self
                .planner
                .update_waypoint()
                .map_or(Ok(ModuleOutput::Idle), |mut waypoint| {
                    waypoint.stamp = self
                        .last_pose
                        .as_ref()
                        .map_or_else(Timestamp::default, |pose| pose.stamp);
                    Ok(ModuleOutput::Waypoint(waypoint))
                }),
            None => Ok(ModuleOutput::Idle),
        }
    }
}

fn validate_frame(
    actual: &NavFrameId,
    expected: &str,
    field: &'static str,
) -> Result<(), NavError> {
    if actual.as_str() == expected {
        Ok(())
    } else {
        Err(NavError::InvalidInput {
            field,
            reason: format!(
                "frame `{}` does not match configured world frame `{expected}`",
                actual.as_str()
            ),
        })
    }
}

fn validate_cloud(cloud: &PointCloud, field: &'static str) -> Result<(), NavError> {
    cloud.validate().map_err(|reason| NavError::InvalidInput {
        field,
        reason: reason.into(),
    })?;
    for point in &cloud.points {
        validate_point(*point, field)?;
    }
    Ok(())
}

fn validate_point(point: Point3, field: &'static str) -> Result<(), NavError> {
    if point.x.is_finite() && point.y.is_finite() && point.z.is_finite() {
        Ok(())
    } else {
        Err(NavError::InvalidInput {
            field,
            reason: "contains a non-finite coordinate".into(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{OccupancyCell, Point2, Pose3};

    fn open_room_module() -> (SimplePlannerModule, OccupancyGrid2D) {
        let geometry = nuwax_nav_types_messages::RobotGeometry {
            self_collision: None,
            footprint_xy_m: vec![[-0.2, -0.15], [0.2, -0.15], [0.2, 0.15], [-0.2, 0.15]],
            z_min_m: -0.2,
            z_max_m: 0.2,
            source: "单元仿真".into(),
        };
        let mut module = SimplePlannerModule::with_config(SimplePlannerConfig {
            robot_geometry: Some(geometry),
            ..SimplePlannerConfig::default()
        })
        .unwrap();
        module.start().unwrap();
        let grid = OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 80, 40, OccupancyCell::Free);
        module.on_occupancy_grid(grid.clone()).unwrap();
        module
            .on_odometry(PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(1.05, 1.05, 0.0),
                    orientation: crate::Quaternion::from_yaw(0.0),
                },
                NavFrameId::Map,
            ))
            .unwrap();
        (module, grid)
    }
    fn target(x: f64, y: f64) -> PointStamped {
        PointStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            point: Point3::new(x, y, 0.0),
        }
    }
    fn wait_for_job(module: &SimplePlannerModule) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
        while !module.planning_job.as_ref().unwrap().handle.is_finished() {
            assert!(
                std::time::Instant::now() < deadline,
                "后台规划未在测试预算内结束"
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    #[test]
    fn goal_completion_uses_request_position_and_final_yaw_tolerances() {
        let (mut module, _) = open_room_module();
        module.on_goal(target(1.35, 1.05)).unwrap();
        module
            .set_goal_options(nuwax_nav_types_messages::GoalOptions {
                position_tolerance_m: 0.1,
                yaw_tolerance_rad: 0.05,
                ..Default::default()
            })
            .unwrap();
        module.set_goal_yaw(1.0);
        assert!(matches!(module.tick().unwrap(), ModuleOutput::Plan { .. }));
        module
            .on_odometry(PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(1.35, 1.05, 0.0),
                    orientation: crate::Quaternion::from_yaw(0.0),
                },
                NavFrameId::Map,
            ))
            .unwrap();
        assert!(!matches!(
            module.tick().unwrap(),
            ModuleOutput::SafetyHold {
                reason: HoldReason::GoalReached,
                ..
            }
        ));
        module
            .on_odometry(PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(1.35, 1.05, 0.0),
                    orientation: crate::Quaternion::from_yaw(1.0),
                },
                NavFrameId::Map,
            ))
            .unwrap();
        assert!(matches!(
            module.tick().unwrap(),
            ModuleOutput::SafetyHold {
                reason: HoldReason::GoalReached,
                ..
            }
        ));
    }

    #[test]
    fn ordinary_route_survives_observation_but_forward_obstacle_revokes_it() {
        let (mut module, mut grid) = open_room_module();
        module.on_goal(target(4.05, 1.05)).unwrap();
        let output = module.tick().unwrap();
        assert!(matches!(output, ModuleOutput::Plan { .. }), "{output:?}");
        assert!(
            module.planned_yaws().is_none(),
            "覆盖没有显式朝向数组的普通路径"
        );
        let path = module.planner.cached_path().unwrap().to_vec();
        grid.set(65, 30, OccupancyCell::Occupied);
        module.on_occupancy_grid(grid.clone()).unwrap();
        assert_eq!(module.planner.cached_path().unwrap(), path);
        grid.set(30, 10, OccupancyCell::Occupied);
        module.on_occupancy_grid(grid).unwrap();
        assert!(module.planner.cached_path().is_none());
    }
    #[test]
    fn background_replacement_and_cancel_reject_old_results_and_join_owner() {
        let (mut module, _) = open_room_module();
        module.enable_background_planning();
        module.on_goal(target(4.05, 1.05)).unwrap();
        assert_eq!(module.tick().unwrap(), ModuleOutput::Idle);
        wait_for_job(&module);
        module.on_goal(target(2.05, 1.05)).unwrap();
        assert_eq!(module.tick().unwrap(), ModuleOutput::Idle);
        wait_for_job(&module);
        let output = module.tick().unwrap();
        let ModuleOutput::Plan { path, .. } = output else {
            panic!("新目标未产生路径：{output:?}")
        };
        assert_eq!(path.points.last(), Some(&Point2::new(2.05, 1.05)));
        module.on_goal(target(4.05, 1.05)).unwrap();
        module.tick().unwrap();
        module.on_stop_movement(true).unwrap();
        assert!(matches!(
            module.tick().unwrap(),
            ModuleOutput::SafetyHold {
                reason: HoldReason::StopRequested,
                ..
            }
        ));
        module.stop().unwrap();
        assert!(module.planning_job.is_none());
        assert!(module.planner.cached_path().is_none());
    }
    #[test]
    fn background_result_is_checked_against_new_obstacles() {
        let (mut module, mut grid) = open_room_module();
        module.enable_background_planning();
        module.on_goal(target(4.05, 1.05)).unwrap();
        module.tick().unwrap();
        wait_for_job(&module);
        grid.set(30, 10, OccupancyCell::Occupied);
        module.on_occupancy_grid(grid).unwrap();
        assert_eq!(module.tick().unwrap(), ModuleOutput::Idle);
        assert!(module.planner.cached_path().is_none());
        module.stop().unwrap();
    }

    #[test]
    fn occupancy_input_restores_configured_footprint_after_terrain_shrink() {
        let mut module = SimplePlannerModule::new();
        module.start().unwrap();
        module
            .on_odometry(PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(0.55, 1.05, 0.0),
                    ..Pose3::default()
                },
                NavFrameId::Map,
            ))
            .unwrap();
        module
            .on_goal(PointStamped::from_xyz(NavFrameId::Map, 3.05, 1.05, 0.0))
            .unwrap();
        module.planner.replan_once(0.0);
        assert!(module.planner.replan_once(6.0).2);
        assert!(module.planner.effective_inflation() < 0.2);
        let grid = OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 40, 30, OccupancyCell::Free);
        for _ in 0..2 {
            module.on_occupancy_grid(grid.clone()).unwrap();
            assert_eq!(module.planner.effective_inflation(), 0.2);
            assert_eq!(module.planner.costmap().inflation_radius(), 0.2);
        }
    }

    #[test]
    fn occupancy_rejects_near_invalid_goals_and_preserves_exact_endpoints() {
        for cell in [OccupancyCell::Unknown, OccupancyCell::Occupied] {
            let mut module = SimplePlannerModule::with_config(SimplePlannerConfig {
                inflation_radius: 0.0,
                ..SimplePlannerConfig::default()
            })
            .unwrap();
            module.start().unwrap();
            let mut grid =
                OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 10, 10, OccupancyCell::Free);
            grid.set(3, 5, cell);
            module.on_occupancy_grid(grid).unwrap();
            module
                .on_odometry(PoseStamped::from_pose(
                    Pose3 {
                        position: Point3::new(0.25, 0.55, 0.0),
                        ..Pose3::default()
                    },
                    NavFrameId::Map,
                ))
                .unwrap();
            for goal in [(0.35, 0.55), (-0.05, 0.55)] {
                module
                    .on_goal(PointStamped::from_xyz(NavFrameId::Map, goal.0, goal.1, 0.0))
                    .unwrap();
                assert!(
                    matches!(
                        module.tick().unwrap(),
                        ModuleOutput::SafetyHold {
                            reason: HoldReason::PlanningFailed,
                            ..
                        }
                    ),
                    "近距离的未知、占据或图外目标不能被报告到达"
                );
            }
        }
        let grid = OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 1.0, 5, 4, OccupancyCell::Free);
        let map = crate::Costmap::from_occupancy(grid, 0.0).unwrap();
        let start = Point2::new(0.1, 1.1);
        let goal = Point2::new(3.99, 1.99);
        let path = crate::plan_on_costmap(&map, start, goal, None, None)
            .unwrap()
            .unwrap();
        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
    }

    #[test]
    fn occupancy_keeps_geometry_height_independent_obstacles_and_replans_updates() {
        let mut module = SimplePlannerModule::with_config(SimplePlannerConfig {
            inflation_radius: 0.0,
            ..SimplePlannerConfig::default()
        })
        .unwrap();
        module.start().unwrap();
        let origin = Point2::new(-1.13, 2.17);
        let mut grid = OccupancyGrid2D::filled(origin, 0.2, 20, 10, OccupancyCell::Free);
        let start = grid.cell_center(2, 5).unwrap();
        let goal = grid.cell_center(17, 5).unwrap();
        // 首帧没有位姿也能消费障碍分类，后续高度不会把占据格当作地面过滤。
        module.on_occupancy_grid(grid.clone()).unwrap();
        module
            .on_odometry(PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(start.x, start.y, 3.0),
                    ..Pose3::default()
                },
                NavFrameId::Map,
            ))
            .unwrap();
        module
            .on_goal(PointStamped::from_xyz(NavFrameId::Map, goal.x, goal.y, 3.0))
            .unwrap();
        let ModuleOutput::Plan { path, .. } = module.tick().unwrap() else {
            panic!("已知自由通道应当可规划");
        };
        assert_eq!(path.points.first(), Some(&start));
        assert_eq!(path.points.last(), Some(&goal));

        for y in 0..grid.height {
            grid.set(10, y, OccupancyCell::Occupied);
        }
        module.on_occupancy_grid(grid.clone()).unwrap();
        assert!(
            matches!(
                module.tick().unwrap(),
                ModuleOutput::SafetyHold {
                    reason: HoldReason::PlanningFailed,
                    ..
                }
            ),
            "地图更新须立即废弃缓存路线，不能绕到图外穿墙"
        );

        for y in 0..grid.height {
            grid.set(10, y, OccupancyCell::Unknown);
        }
        module.on_occupancy_grid(grid.clone()).unwrap();
        assert!(
            matches!(
                module.tick().unwrap(),
                ModuleOutput::SafetyHold {
                    reason: HoldReason::PlanningFailed,
                    ..
                }
            ),
            "未知带不等于自由通道"
        );
        for y in 0..grid.height {
            grid.set(10, y, OccupancyCell::Free);
        }
        module.on_occupancy_grid(grid).unwrap();
        assert!(matches!(module.tick().unwrap(), ModuleOutput::Plan { .. }));
    }

    fn cloud(points: Vec<Point3>) -> PointCloud {
        PointCloud {
            stamp: Timestamp::default(),
            frame_id: NavFrameId::Map,
            points,
            intensities: Vec::new(),
        }
    }

    #[test]
    fn empty_persistent_snapshot_clears_previous_obstacles() {
        let mut module = SimplePlannerModule::new();
        module.start().expect("start");
        module
            .on_terrain_map_ext(cloud(vec![Point3::new(1.0, 1.0, 1.0)]))
            .expect("occupied snapshot");
        assert!(!module.planner.costmap().heights().is_empty());

        module
            .on_terrain_map_ext(cloud(Vec::new()))
            .expect("all-free snapshot");
        assert!(module.planner.costmap().heights().is_empty());
    }
}
