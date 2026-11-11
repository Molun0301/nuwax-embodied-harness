//! Session 独有的探索对象，复用原前沿选择、失败反馈与返航路线算法。
use std::sync::Arc;
use std::time::{Duration, Instant};

use nuwax_exploration::mission::{MissionPose, MissionProgress, ReturnPolicy, SourceCheckpoint};
use nuwax_exploration::surface_adapter::{SurfaceGoalSourceAdapter, SurfaceInputReceiver};
use nuwax_exploration::wavefront_adapter::WavefrontGoalSourceAdapter;
use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationBoundary, ExplorationController,
    ExplorationGoalFeedback, ExplorationPoll, ExplorationStateSnapshot, ExplorationStopReason,
    ExplorationTerminalKind,
};
use nuwax_explore_wavefront::WavefrontConfig;
use nuwax_nav_mls_planner::{
    SurfaceMapBuilder, SurfaceObservations, SurfaceSnapshot,
};
use nuwax_nav_stack::surface_config::SurfaceNavigationConfig;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::{
    ClockKind, GoalOptions, NavigationCapabilities, OccupancyCell, OccupancyGrid2D, Point2, Point3,
    Pose3, PoseStamped, Quaternion, RobotGeometry, Timestamp,
};
use serde::{Deserialize, Serialize};
use tonic::Status;

use crate::explore_mission::{
    frontier_goal_options, home_goal_options, MissionCalibration, MissionRoutes,
};
use crate::proto;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub(crate) struct ExploreStopConditions {
    pub max_coverage_pct: Option<f64>,
    pub no_frontier_timeout_ms: Option<u64>,
    pub max_duration_ms: Option<u64>,
    pub max_goals: Option<u32>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExploreStartConfig {
    pub boundary: Option<ExplorationBoundary>,
    pub return_policy: ReturnPolicy,
    pub stop_conditions: ExploreStopConditions,
    pub max_navigation_retries: u32,
    pub blacklist_radius_m: f64,
}

impl Default for ExploreStartConfig {
    fn default() -> Self {
        Self {
            boundary: None,
            return_policy: ReturnPolicy::default(),
            stop_conditions: ExploreStopConditions::default(),
            max_navigation_retries: 1,
            blacklist_radius_m: WavefrontConfig::default().safe_distance_m,
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Progress {
    map_name: String,
    elapsed_ns: u64,
    reached_goals: u32,
    mission: Option<MissionProgress>,
    return_reason: Option<ExplorationStopReason>,
    sources: Vec<SourceCheckpoint>,
}

struct Child {
    id: String,
    source_id: String,
    returning: bool,
}

/// 调度结果交给 Session 更新唯一公开状态并派发或取消子导航。
#[derive(Clone)]
pub struct ExplorerTick {
    pub status: proto::ExplorationStatus,
    pub goal: Option<(String, proto::NavigationGoal)>,
    pub boundary: Vec<[f64; 2]>,
    pub cancel_goal: Option<String>,
}

pub struct Explorer {
    map_name: String,
    map_frame: String,
    surface_config: Option<SurfaceNavigationConfig>,
    robot_geometry:RobotGeometry,
    capabilities:Option<NavigationCapabilities>,
    pose_ttl_ns:u64,
    surface_input: SurfaceInputReceiver,
    controller: ExplorationController,
    config: Option<ExploreStartConfig>,
    id: String,
    motion_generation: u64,
    child: Option<Child>,
    reached_goals: u32,
    frontier_count: u32,
    elapsed_before: Duration,
    started: Option<Instant>,
    no_frontier_since: Option<Instant>,
    input_wait_since: Option<Instant>,
    mission: Option<MissionProgress>,
    calibration: Option<MissionCalibration>,
    return_reason: Option<ExplorationStopReason>,
    stop_reason: Option<ExplorationStopReason>,
    return_failures: u32,
}

impl Explorer {
    pub fn new(map_name: String, map_frame: String, surface_mode: bool) -> Result<Self, Status> {
        Self::from_config(
            map_name,
            map_frame,
            surface_mode,
            nuwax_config::navigation().map_err(Status::failed_precondition)?,
            nuwax_config::robot().map_err(Status::failed_precondition)?,
            nuwax_config::derived().map_err(Status::failed_precondition)?,
        )
    }
    pub fn from_config(
        map_name: String,
        map_frame: String,
        surface_mode: bool,
        n: &nuwax_config::sections::Navigation,
        r: &nuwax_config::sections::Robot,
        d: &nuwax_config::sections::Derived,
    ) -> Result<Self, Status> {
        let surface_config = surface_mode
            .then(|| SurfaceNavigationConfig::from_config(n, r, d))
            .transpose()
            .map_err(|e| Status::failed_precondition(e.to_string()))?;
        let robot_geometry = RobotGeometry::from_runtime_json(&d.robot_geometry_json)
            .map_err(Status::failed_precondition)?;
        let capabilities = NavigationCapabilities::from_config(&n.capabilities_json)
            .map_err(Status::failed_precondition)?;
        Ok(Self {
            map_name,
            map_frame,
            surface_config,
            robot_geometry,
            capabilities,
            pose_ttl_ns: (n.explore_pose_ttl_ms as u64).saturating_mul(1_000_000),
            surface_input: SurfaceInputReceiver::new(),
            controller: ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront),
            config: None,
            id: String::new(),
            motion_generation: 0,
            child: None,
            reached_goals: 0,
            frontier_count: 0,
            elapsed_before: Duration::ZERO,
            started: None,
            no_frontier_since: None,
            input_wait_since: None,
            mission: None,
            calibration: None,
            return_reason: None,
            stop_reason: None,
            return_failures: 0,
        })
    }

    pub fn set_map_name(&mut self, name: String) {
        self.map_name = name;
    }

    pub fn start(
        &mut self,
        id: String,
        request: proto::ExplorationRequest,
    ) -> Result<proto::ExplorationStatus, Status> {
        let boundary = if request.boundary.is_empty() {
            None
        } else {
            Some(
                ExplorationBoundary::from_segments(
                    request
                        .boundary
                        .iter()
                        .map(|segment| nuwax_exploration::BoundarySegment {
                            start: Point2::new(segment.start_x, segment.start_y),
                            end: Point2::new(segment.end_x, segment.end_y),
                        })
                        .collect(),
                )
                .map_err(|error| Status::invalid_argument(error.to_string()))?,
            )
        };
        let stop = request.stop_conditions.unwrap_or_default();
        let return_policy = request
            .return_policy
            .map(|p| ReturnPolicy {
                maximum_distance_m: p.maximum_distance_m,
                reserve_distance_m: p.reserve_distance_m,
                return_on_completion: p.return_on_completion,
            })
            .unwrap_or_default();
        return_policy.validate().map_err(Status::invalid_argument)?;
        if self.surface_config.is_some() && stop.max_coverage_pct.is_some() {
            return Err(Status::invalid_argument(
                "三维探索不能用未知总体积计算覆盖率",
            ));
        }
        if !request.blacklist_radius_m.is_finite()
            || request.blacklist_radius_m < 0.0
            || stop
                .max_coverage_pct
                .is_some_and(|v| !v.is_finite() || !(0.0..=100.0).contains(&v))
        {
            return Err(Status::invalid_argument("探索停止条件或黑名单半径无效"));
        }
        let config = ExploreStartConfig {
            boundary,
            return_policy,
            stop_conditions: ExploreStopConditions {
                max_coverage_pct: stop.max_coverage_pct,
                no_frontier_timeout_ms: stop.no_frontier_timeout_ms,
                max_duration_ms: stop.max_duration_ms,
                max_goals: stop.max_goals,
            },
            max_navigation_retries: request.max_navigation_retries,
            blacklist_radius_m: request.blacklist_radius_m,
        };
        let restored: Option<Progress> = if request.resume_progress_json.is_empty() {
            None
        } else {
            let value: Progress = serde_json::from_str(&request.resume_progress_json)
                .map_err(|error| Status::invalid_argument(format!("探索进度无效：{error}")))?;
            if value.map_name != self.map_name {
                return Err(Status::failed_precondition("探索进度地图名与当前地图不符"));
            }
            if let Some(mission) = &value.mission {
                mission.validate().map_err(Status::invalid_argument)?;
            }
            Some(value)
        };
        let observation = NavigationCapabilities::from_runtime()
            .map_err(Status::failed_precondition)?
            .and_then(|c| c.observation);
        let mut controller =
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        if let Some(surface) = &self.surface_config {
            let mut source = SurfaceGoalSourceAdapter::new(
                "surface",
                surface.follower.body_to_support_z_m,
                self.surface_input.clone(),
            )
            .map_err(|e| Status::failed_precondition(e.to_string()))?;
            if let Some(model) = observation {
                source
                    .set_observation_model(model)
                    .map_err(|e| Status::failed_precondition(e.to_string()))?;
            }
            source
                .configure_constraints(
                    config.boundary.clone(),
                    config.max_navigation_retries,
                    config.blacklist_radius_m,
                )
                .map_err(|e| Status::invalid_argument(e.to_string()))?;
            controller.add_source(source);
        } else {
            let geometry = self.robot_geometry.clone();
            let mut source = WavefrontGoalSourceAdapter::with_default_id(
                "wavefront",
                WavefrontConfig::default(),
                geometry,
            );
            if let Some(model) = observation {
                source
                    .set_observation_model(model)
                    .map_err(|e| Status::failed_precondition(e.to_string()))?;
            }
            source.configure_constraints(
                config.boundary.clone(),
                config.max_navigation_retries,
                config.blacklist_radius_m,
            );
            controller.add_source(source);
        }
        controller
            .begin_exploration()
            .map_err(|e| Status::internal(e.to_string()))?;
        if let Some(saved) = &restored {
            controller
                .restore_checkpoint(&saved.sources)
                .map_err(|e| Status::invalid_argument(e.to_string()))?;
        }
        let calibration = config
            .return_policy
            .enabled()
            .then(|| MissionCalibration::capture(self.capabilities.as_ref(),&self.robot_geometry))
            .transpose()
            .map_err(Status::failed_precondition)?;
        self.controller = controller;
        self.config = Some(config);
        self.id = id;
        self.motion_generation = request.motion_generation;
        self.child = None;
        self.reached_goals = restored.as_ref().map_or(0, |p| p.reached_goals);
        self.frontier_count = 0;
        self.elapsed_before = Duration::from_nanos(restored.as_ref().map_or(0, |p| p.elapsed_ns));
        self.mission = restored.as_ref().and_then(|p| p.mission.clone());
        self.return_reason = restored.and_then(|p| p.return_reason);
        self.calibration = calibration;
        self.started = Some(Instant::now());
        self.no_frontier_since = None;
        self.input_wait_since = None;
        self.stop_reason = None;
        self.return_failures = 0;
        Ok(self.status("探索已开始"))
    }

    pub fn stop(&mut self) -> Result<proto::ExplorationStatus, String> {
        if let Some(child) = self.child.take() {
            if !child.returning {
                self.controller
                    .report_goal_feedback(&child.source_id, ExplorationGoalFeedback::Cancelled)
                    .map_err(|e| e.to_string())?;
            }
        }
        self.finish(ExplorationStopReason::UserCancelled)?;
        Ok(self.status("探索已停止"))
    }

    pub fn progress(&self) -> Result<proto::ExplorationProgress, Status> {
        if self.id.is_empty() {
            return Err(Status::failed_precondition("当前会话尚未开始探索"));
        }
        let saved = Progress {
            map_name: self.map_name.clone(),
            elapsed_ns: self.elapsed().as_nanos() as u64,
            reached_goals: self.reached_goals,
            mission: self.mission.clone(),
            return_reason: self.return_reason,
            sources: self
                .controller
                .checkpoint()
                .map_err(|e| Status::internal(e.to_string()))?,
        };
        Ok(proto::ExplorationProgress {
            exploration_id: self.id.clone(),
            progress_json: serde_json::to_string(&saved)
                .map_err(|e| Status::internal(e.to_string()))?,
        })
    }

    pub fn navigation_feedback(&mut self, status: &proto::NavigationStatus) -> Result<(), String> {
        if !self
            .child
            .as_ref()
            .is_some_and(|child| child.id == status.goal_id)
        {
            return Ok(());
        }
        let phase =
            proto::GoalStatus::try_from(status.status).unwrap_or(proto::GoalStatus::Unspecified);
        let feedback = match phase {
            proto::GoalStatus::Succeeded => ExplorationGoalFeedback::Reached,
            proto::GoalStatus::Failed => ExplorationGoalFeedback::Failed(status.message.clone()),
            proto::GoalStatus::Cancelled
            | proto::GoalStatus::Replaced
            | proto::GoalStatus::Stopped => ExplorationGoalFeedback::Cancelled,
            _ => return Ok(()),
        };
        let child = self.child.take().expect("已检查当前子目标");
        if phase == proto::GoalStatus::Succeeded {
            self.reached_goals += 1;
            if child.returning {
                self.finish(
                    self.return_reason
                        .unwrap_or(ExplorationStopReason::NoInformationGain),
                )?;
                return Ok(());
            }
        }
        if matches!(
            phase,
            proto::GoalStatus::Cancelled | proto::GoalStatus::Replaced | proto::GoalStatus::Stopped
        ) {
            if !child.returning {
                self.controller
                    .report_goal_feedback(&child.source_id, feedback)
                    .map_err(|e| e.to_string())?;
            }
            self.finish(ExplorationStopReason::UserCancelled)?;
        } else if child.returning {
            self.return_failures += 1;
            if self.return_failures
                > self
                    .config
                    .as_ref()
                    .expect("探索配置")
                    .max_navigation_retries
            {
                self.finish(ExplorationStopReason::ConsecutiveFailuresExceeded)?;
            }
        } else {
            self.controller
                .report_goal_feedback(&child.source_id, feedback)
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    pub fn tick(
        &mut self,
        pose: Option<proto::Pose>,
        grid: Option<proto::OccupancyGrid>,
        surface: Result<Arc<SurfaceSnapshot>, String>,
        forbidden: Result<(), String>,
    ) -> Result<ExplorerTick, String> {
        match self.tick_step(pose, grid, surface, forbidden) {
            Ok(update) => Ok(update),
            Err(message) => {
                self.fail_exploration(ExplorationStopReason::ConsecutiveFailuresExceeded, &message)
            }
        }
    }

    fn tick_step(
        &mut self,
        pose: Option<proto::Pose>,
        grid: Option<proto::OccupancyGrid>,
        surface: Result<Arc<SurfaceSnapshot>, String>,
        forbidden: Result<(), String>,
    ) -> Result<ExplorerTick, String> {
        if !self.controller.is_active() {
            return Ok(self.output("探索已结束", None, None));
        }
        let config = self.config.as_ref().expect("活动探索配置").clone();
        let stop = if config
            .stop_conditions
            .max_duration_ms
            .is_some_and(|v| self.elapsed() >= Duration::from_millis(v))
        {
            Some(ExplorationStopReason::DurationExceeded)
        } else if config
            .stop_conditions
            .max_goals
            .is_some_and(|v| self.reached_goals >= v)
        {
            Some(ExplorationStopReason::MaxGoalsReached)
        } else {
            None
        };
        if let Some(reason) = stop.filter(|_| !config.return_policy.enabled()) {
            let cancel = self.child.take().map(|child| {
                if !child.returning {
                    let _ = self
                        .controller
                        .report_goal_feedback(&child.source_id, ExplorationGoalFeedback::Cancelled);
                }
                child.id
            });
            self.finish(reason)?;
            return Ok(self.output("达到探索停止条件", None, cancel));
        }
        if let Err(reason) = forbidden {
            self.no_frontier_since = None;
            self.input_wait_since = None;
            return Ok(self.output(&format!("等待禁行区输入：{reason}"), None, None));
        }
        let Some(pose) = pose else {
            return self.wait_input("等待地图位姿：尚未收到，来源年龄未知");
        };
        if pose.frame_id != self.map_frame {
            return self.wait_input(&format!("等待地图位姿：坐标系{}不匹配{}", pose.frame_id, self.map_frame));
        }
        if let Err(reason) = fresh(pose.timestamp_ns, self.pose_ttl_ns) {
            return self.wait_input(&format!("等待地图位姿：{reason}"));
        }
        let Some(meta) = grid.as_ref().and_then(|grid| grid.meta.as_ref()) else {
            return self.wait_input("等待占据栅格：尚未收到元数据，来源年龄未知");
        };
        if meta.frame_id != self.map_frame {
            return self.wait_input(&format!("等待占据栅格：坐标系{}不匹配{}", meta.frame_id, self.map_frame));
        }
        if let Err(reason) = fresh(meta.timestamp_ns, 2_000_000_000) {
            return self.wait_input(&format!("等待占据栅格：{reason}"));
        }
        if self.surface_config.is_some() {
            if let Err(reason) = &surface {
                return self.wait_input(reason);
            }
        }
        self.input_wait_since = None;
        let robot = to_pose(&pose)?;
        let location = mission_pose(&pose)?;
        if self.mission.is_none() {
            self.mission = Some(MissionProgress::new(location.clone())?);
        }
        self.mission.as_mut().expect("探索行程").observe(&location, pose.timestamp_ns);
        let grid = grid.as_ref().map(to_grid).transpose()?;
        let route = if config.return_policy.enabled() {
            Some(match (&surface, &self.surface_config) {
                (Ok(map), Some(surface_config)) => MissionRoutes::Surface {
                    map: map.clone(),
                    config: surface_config.clone(),
                    boundary: self.boundary(),
                },
                _ => MissionRoutes::planar(
                    grid.as_ref().ok_or("返航缺少地图")?.clone(),
                    self.calibration.as_ref().expect("返航标定"),
                    &config,
                )?,
            })
        } else {
            None
        };
        let mut cancel_goal = None;
        let mut stop = stop;
        if let (Some(mission), Some(route)) = (&self.mission, &route) {
            if !mission.returning {
                let home = match route.query(&location, &mission.home, home_goal_options(), None) {
                    Ok(home) => home,
                    Err(error) => {
                        return self.fail_exploration(
                            if error.is_inconclusive() {
                                ExplorationStopReason::PlanningResourceLimited
                            } else {
                                ExplorationStopReason::ConsecutiveFailuresExceeded
                            },
                            &error.to_string(),
                        )
                    }
                };
                if config
                    .return_policy
                    .should_return(mission.travelled_m, home.length)?
                {
                    stop = Some(ExplorationStopReason::DistanceBudgetReached);
                }
            }
        }
        if let Some(reason) = stop {
            cancel_goal = self.child.take().map(|child| {
                if !child.returning {
                    let _ = self
                        .controller
                        .report_goal_feedback(&child.source_id, ExplorationGoalFeedback::Cancelled);
                }
                child.id
            });
            if self.begin_return(reason) {
            } else {
                self.finish(reason)?;
                return Ok(self.output("达到探索停止条件", None, cancel_goal));
            }
        }
        if self.child.is_some() {
            return Ok(self.output("执行探索子目标", None, cancel_goal));
        }
        if self.mission.as_ref().is_some_and(|m| m.returning) {
            let home = self.mission.as_ref().expect("返航行程").home.clone();
            let id = uuid::Uuid::new_v4().to_string();
            self.child = Some(Child {
                id: id.clone(),
                source_id: String::new(),
                returning: true,
            });
            let goal = self.goal(home, home_goal_options());
            return Ok(self.output("正在返航", Some((id, goal)), cancel_goal));
        }
        if config
            .stop_conditions
            .max_coverage_pct
            .is_some_and(|limit| {
                grid.as_ref()
                    .is_some_and(|g| coverage(g, config.boundary.as_ref()) >= limit)
            })
        {
            if self.begin_return(ExplorationStopReason::CoverageReached) {
                return Ok(self.output("达到覆盖率，准备返航", None, None));
            }
            self.finish(ExplorationStopReason::CoverageReached)?;
            return Ok(self.output("达到覆盖率", None, None));
        }
        if let Ok(surface) = surface {
            self.surface_input
                .observe(surface)
                .map_err(|e| e.to_string())?;
        }
        let mut snapshot = ExplorationStateSnapshot::new(
            robot,
            Timestamp {
                nanos: unix_ns() as i64,
                clock: ClockKind::Unix,
            },
        );
        snapshot.global_costmap = grid.as_ref();
        let poll = self
            .controller
            .poll_next_goal_with_deferred_stop(&snapshot, config.return_policy.enabled())
            .map_err(|e| e.to_string())?;
        if matches!(poll, ExplorationPoll::Goal(_) | ExplorationPoll::NoFrontier) {
            self.frontier_count = self.controller.frontier_snapshot().len() as u32;
        }
        match poll {
            ExplorationPoll::Goal(frontier) => {
                self.no_frontier_since = None;
                let navigation =
                    nuwax_exploration::conversions::frontier_goal_to_navigation_goal(&frontier);
                let target = navigation.target.pose;
                let goal_pose = MissionPose {
                    position: [target.position.x, target.position.y, target.position.z],
                    orientation_xyzw: [
                        target.orientation.x,
                        target.orientation.y,
                        target.orientation.z,
                        target.orientation.w,
                    ],
                };
                let options = frontier_goal_options(
                    &frontier,
                    self.surface_config.is_some(),
                    self.surface_config.as_ref(),
                );
                if let (Some(mission), Some(route)) = (&self.mission, &route) {
                    let advice =
                        route
                            .query(&location, &goal_pose, options, None)
                            .and_then(|outward| {
                                route
                                    .query(&outward.end, &mission.home, home_goal_options(), None)
                                    .map(|home| (outward, home))
                            });
                    let (outward, home) = match advice {
                        Ok(routes) => routes,
                        Err(error) if error.is_inconclusive() => {
                            return self.fail_exploration(
                                ExplorationStopReason::PlanningResourceLimited,
                                &error.to_string(),
                            )
                        }
                        Err(error) => {
                            self.controller
                                .report_goal_feedback(
                                    &frontier.id,
                                    ExplorationGoalFeedback::Failed(error.to_string()),
                                )
                                .map_err(|e| e.to_string())?;
                            return Ok(self.output("候选前沿路线不可达", None, None));
                        }
                    };
                    if !config.return_policy.can_visit(
                        mission.travelled_m,
                        outward.length,
                        home.length,
                    ) {
                        self.controller
                            .report_goal_feedback(&frontier.id, ExplorationGoalFeedback::Cancelled)
                            .map_err(|e| e.to_string())?;
                        self.begin_return(ExplorationStopReason::DistanceBudgetReached);
                        return Ok(self.output("探索行程预算触发返航", None, None));
                    }
                }
                let id = uuid::Uuid::new_v4().to_string();
                self.child = Some(Child {
                    id: id.clone(),
                    source_id: frontier.id,
                    returning: false,
                });
                let goal = self.goal(goal_pose, options);
                Ok(self.output("前往探索前沿", Some((id, goal)), None))
            }
            ExplorationPoll::Stop(reason) => {
                if !self.begin_return(reason) {
                    self.finish(reason)?;
                }
                Ok(self.output("探索来源已结束", None, None))
            }
            ExplorationPoll::NoFrontier => {
                let since = *self.no_frontier_since.get_or_insert_with(Instant::now);
                let limit = Duration::from_millis(
                    config
                        .stop_conditions
                        .no_frontier_timeout_ms
                        .unwrap_or(5_000),
                );
                if since.elapsed() >= limit {
                    if !self.begin_return(ExplorationStopReason::NoFrontierTimeout) {
                        self.finish(ExplorationStopReason::NoFrontierTimeout)?;
                    }
                }
                Ok(self.output("等待新前沿", None, None))
            }
            ExplorationPoll::Pending => Ok(self.output("正在选择前沿", None, None)),
            ExplorationPoll::SourceUnavailable(reason) => self.wait_input(&reason),
        }
    }

    pub(crate) fn failed(&mut self, message: &str) -> Result<ExplorerTick, String> {
        self.fail_exploration(ExplorationStopReason::ConsecutiveFailuresExceeded, message)
    }

    fn fail_exploration(
        &mut self,
        reason: ExplorationStopReason,
        message: &str,
    ) -> Result<ExplorerTick, String> {
        let cancel = self.child.take().map(|child| {
            if !child.returning {
                let _ = self
                    .controller
                    .report_goal_feedback(&child.source_id, ExplorationGoalFeedback::Cancelled);
            }
            child.id
        });
        self.finish(reason)?;
        Ok(self.output(message, None, cancel))
    }

    fn begin_return(&mut self, reason: ExplorationStopReason) -> bool {
        if reason.terminal_kind() != ExplorationTerminalKind::Succeeded {
            return false;
        }
        let Some(config) = &self.config else {
            return false;
        };
        if !config.return_policy.return_on_completion
            && reason != ExplorationStopReason::DistanceBudgetReached
        {
            return false;
        }
        let Some(mission) = &mut self.mission else {
            return false;
        };
        mission.returning = true;
        self.return_reason = Some(reason);
        true
    }
    fn wait_input(&mut self, message: &str) -> Result<ExplorerTick, String> {
        self.no_frontier_since = None;
        if self
            .input_wait_since
            .get_or_insert_with(Instant::now)
            .elapsed()
            >= Duration::from_secs(30)
        {
            let cancel = self.child.take().map(|child| {
                if !child.returning {
                    let _ = self
                        .controller
                        .report_goal_feedback(&child.source_id, ExplorationGoalFeedback::Cancelled);
                }
                child.id
            });
            self.finish(ExplorationStopReason::PlanningResourceLimited)?;
            return Ok(self.output(message, None, cancel));
        }
        Ok(self.output(message, None, None))
    }
    fn finish(&mut self, reason: ExplorationStopReason) -> Result<(), String> {
        self.elapsed_before = self.elapsed();
        self.started = None;
        self.stop_reason = Some(reason);
        self.controller
            .stop_with_reason(reason)
            .map_err(|e| e.to_string())
    }
    fn elapsed(&self) -> Duration {
        self.elapsed_before + self.started.map_or(Duration::ZERO, |at| at.elapsed())
    }
    fn boundary(&self) -> Vec<[f64; 2]> {
        self.config
            .as_ref()
            .and_then(|c| c.boundary.as_ref())
            .map_or_else(Vec::new, |b| {
                b.vertices().iter().map(|p| [p.x, p.y]).collect()
            })
    }
    fn goal(&self, pose: MissionPose, options: GoalOptions) -> proto::NavigationGoal {
        proto::NavigationGoal {
            pose: Some(from_mission(&pose, &self.map_frame)),
            motion_generation: self.motion_generation,
            allow_reverse: Some(options.allow_reverse),
            position_tolerance_m: Some(options.position_tolerance_m),
            yaw_tolerance_rad: Some(options.yaw_tolerance_rad),
        }
    }
    fn status(&self, message: &str) -> proto::ExplorationStatus {
        let phase =
            self.stop_reason
                .map_or(proto::ExplorationPhase::Running, |reason| {
                    match reason.terminal_kind() {
                        ExplorationTerminalKind::Succeeded => proto::ExplorationPhase::Succeeded,
                        ExplorationTerminalKind::Cancelled => proto::ExplorationPhase::Stopped,
                        ExplorationTerminalKind::Failed => proto::ExplorationPhase::Failed,
                    }
                });
        proto::ExplorationStatus {
            exploration_id: self.id.clone(),
            phase: phase as i32,
            message: message.into(),
            frontier_count: self.frontier_count,
            goals_completed: self.reached_goals,
            returning: self.mission.as_ref().is_some_and(|m| m.returning),
            traveled_distance_m: self.mission.as_ref().map_or(0.0, |m| m.travelled_m),
        }
    }
    fn output(
        &self,
        message: &str,
        goal: Option<(String, proto::NavigationGoal)>,
        cancel_goal: Option<String>,
    ) -> ExplorerTick {
        ExplorerTick {
            status: self.status(message),
            goal,
            boundary: self.boundary(),
            cancel_goal,
        }
    }
}

fn unix_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

pub(crate) fn fresh(timestamp: u64, limit: u64) -> Result<(), String> {
    let age = nuwax_nav_types_messages::source_time::data_age_ns_at(timestamp, unix_ns())
        .map_err(|reason| format!("来源年龄不可用：{reason}；源时间={timestamp}"))?;
    if age > limit {
        return Err(format!("来源年龄={}毫秒，超过{}毫秒；源时间={timestamp}", age / 1_000_000, limit / 1_000_000));
    }
    Ok(())
}
fn mission_pose(pose: &proto::Pose) -> Result<MissionPose, String> {
    let p = pose.position.as_ref().ok_or("位姿缺少位置")?;
    let q = pose.orientation.as_ref().ok_or("位姿缺少朝向")?;
    Ok(MissionPose {
        position: [p.x, p.y, p.z],
        orientation_xyzw: [q.x, q.y, q.z, q.w],
    })
}
fn from_mission(pose: &MissionPose, frame: &str) -> proto::Pose {
    let p = pose.position;
    let q = pose.orientation_xyzw;
    proto::Pose {
        position: Some(proto::Vec3 {
            x: p[0],
            y: p[1],
            z: p[2],
        }),
        orientation: Some(proto::Quat {
            x: q[0],
            y: q[1],
            z: q[2],
            w: q[3],
        }),
        frame_id: frame.into(),
        timestamp_ns: unix_ns(),
    }
}
fn to_pose(pose: &proto::Pose) -> Result<PoseStamped, String> {
    let p = pose.position.as_ref().ok_or("位姿缺少位置")?;
    let q = pose.orientation.as_ref().ok_or("位姿缺少朝向")?;
    Ok(PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp {
            nanos: pose.timestamp_ns as i64,
            clock: ClockKind::Unix,
        },
        pose: Pose3 {
            position: Point3::new(p.x, p.y, p.z),
            orientation: Quaternion {
                x: q.x,
                y: q.y,
                z: q.z,
                w: q.w,
            },
        },
    })
}
fn to_grid(grid: &proto::OccupancyGrid) -> Result<OccupancyGrid2D, String> {
    let meta = grid.meta.as_ref().ok_or("栅格缺少元数据")?;
    let cells = grid
        .data
        .iter()
        .map(|v| match *v as i8 {
            -1 => OccupancyCell::Unknown,
            0 => OccupancyCell::Free,
            _ => OccupancyCell::Occupied,
        })
        .collect();
    Ok(OccupancyGrid2D::new(
        Point2::new(meta.origin_x, meta.origin_y),
        f64::from(meta.resolution),
        meta.width,
        meta.height,
        cells,
    ))
}
fn coverage(grid: &OccupancyGrid2D, boundary: Option<&ExplorationBoundary>) -> f64 {
    let mut total = 0_u64;
    let mut known = 0_u64;
    for y in 0..grid.height {
        for x in 0..grid.width {
            let point = Point2::new(
                grid.origin.x + (f64::from(x) + 0.5) * grid.resolution,
                grid.origin.y + (f64::from(y) + 0.5) * grid.resolution,
            );
            if boundary.is_some_and(|b| !b.contains(point)) {
                continue;
            }
            total += 1;
            if grid.cells[y as usize * grid.width as usize + x as usize] != OccupancyCell::Unknown {
                known += 1;
            }
        }
    }
    if total == 0 {
        0.0
    } else {
        100.0 * known as f64 / total as f64
    }
}

/// 探索线程持有的表面构建器，保存上一封图根和观测。
pub struct SurfaceInputBuilder {
    builder: SurfaceMapBuilder,
    planner_size: f32,
    previous: Option<(Arc<nuwax_map_chunks::Manifest>, Arc<SurfaceObservations>)>,
}

impl SurfaceInputBuilder {
    /// 捕获探索使用的几何配置和机体能力。
    pub fn new() -> Result<Self, String> {
        let config = SurfaceNavigationConfig::from_runtime().map_err(|e| e.to_string())?;
        Ok(Self {
            planner_size: config.geometry.voxel_size,
            builder: SurfaceMapBuilder::configured(config.geometry, Some(config.body_envelope)),
            previous: None,
        })
    }

    /// 与导航共用封图差分，只替换变化的观测块和规划列。
    pub fn build(
        &mut self,
        map: Arc<nuwax_map_chunks::SealedMap>,
    ) -> Result<Arc<SurfaceSnapshot>, String> {
        let root = map.manifest();
        let previous = self.previous.as_ref().filter(|(old, _)| {
            old.domain == root.domain && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits()
        });
        let mut reader = nuwax_map_chunks::Receiver::new(map.repository.clone());
        let (observations, _) = nuwax_nav_mls_planner::surface_chunks::observations_with_changes(
            &mut reader, root, previous.map(|(old, grid)| (old.as_ref(), grid)),
        )?;
        let (updates, read_points) = nuwax_nav_mls_planner::surface_chunks::column_updates(
            &mut reader, root, previous.map(|(old, _)| old.as_ref()),
            self.planner_size,
        )?;
        let snapshot = self.builder.build_column_updates(
            &updates, root.voxel_size_m, previous.is_none(), read_points,
        ).map_err(|e| e.to_string())?.snapshot;
        self.previous = Some((Arc::clone(root), Arc::clone(&observations)));
        Ok(Arc::new(snapshot.with_observations(Some(observations))))
    }
}

/// 在已构建的物理表面叠加当前地图禁区；文件变化只改变下一次规划输入。
pub fn apply_forbidden(
    surface: Arc<SurfaceSnapshot>,
    mask: Option<proto::OccupancyGrid>,
) -> Result<Arc<SurfaceSnapshot>, String> {
    let Some(mask) = mask else {
        return Ok(surface);
    };
    let meta = mask.meta.as_ref().ok_or("禁区遮罩缺少元数据")?;
    let blocked: Vec<_> = mask.data.iter().map(|value| *value == 100).collect();
    let output = surface
        .with_virtual_obstacles(
            [meta.origin_x, meta.origin_y],
            f64::from(meta.resolution),
            meta.width,
            meta.height,
            &blocked,
        )
        .map_err(|error| error.to_string())?;
    Ok(Arc::new(output))
}

#[cfg(test)]
#[path = "explorer_tests.rs"]
pub(crate) mod tests;
