//! `WavefrontGoalSourceAdapter`:包装 `nuwax-explore-wavefront::WavefrontGoalSelector`
//! 提供 runtime ingestion / id_source 注入,并把 selector 输出映射为
//! `ExplorationPoll`。
//!
//! 仅在 `runtime + wavefront` feature 启用时编译(方案 §8.5)。

use std::borrow::Cow;
use std::cell::RefCell;

use nuwax_explore_wavefront::selector::WavefrontGoalSelector;
use nuwax_explore_wavefront::session::ExplorationSession;
use nuwax_explore_wavefront::{
    FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection, StopReason, WavefrontConfig,
    WavefrontError,
};
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;

use crate::runtime::WavefrontInputReceiver;
use crate::{
    ExplorationBoundary, ExplorationGoalFeedback, ExplorationGoalSource, ExplorationPoll,
    ExplorationSourceDiagnostics, ExplorationStateSnapshot, ExploreError, ExploreResult,
};

/// 简单 deterministic id source(用于测试 / 生产 uuid 替代)
#[derive(Debug, Default)]
pub struct CountingIdSource {
    counter: u64,
}

impl FrontierGoalIdSource for CountingIdSource {
    fn next_frontier_goal_id(&mut self, seed: &FrontierGoalIdSeed) -> String {
        let n = self.counter;
        self.counter += 1;
        format!("wavefront-{:016x}-{}", seed.stamp.nanos, n)
    }
}

/// wavefront -> facade source adapter
///
/// 必须持 injectable `id_source`;`begin` / `end` 由 controller 触发。
enum PendingWavefrontPoll {
    Ready {
        outcome: ExploreResult<ExplorationPoll>,
        error: Option<Option<String>>,
    },
    Selection(nuwax_explore_wavefront::selector::PreparedWavefrontSelection),
}

/// Wavefront source with one explicitly resolved semantic poll transaction.
pub struct WavefrontGoalSourceAdapter {
    prepared: Option<PendingWavefrontPoll>,
    name: Cow<'static, str>,
    selector: WavefrontGoalSelector,
    session: ExplorationSession,
    id_source: Box<dyn FrontierGoalIdSource + Send>,
    cached_costmap: RefCell<Option<OccupancyGrid2D>>,
    cached_odom: RefCell<Option<nuwax_nav_types_messages::geometry::PoseStamped>>,
    is_active: bool,
    last_goal_id: Option<String>,
    last_goal: Option<nuwax_nav_types_messages::visual::FrontierGoal>,
    last_frontiers: Vec<nuwax_nav_types_messages::visual::FrontierGoal>,
    last_error: Option<String>,
    boundary: Option<ExplorationBoundary>,
    max_navigation_retries: u32,
    blacklist_radius_m: f64,
    max_consecutive_failures: u32,
    robot_geometry: Option<nuwax_nav_types_messages::RobotGeometry>,
}

impl WavefrontGoalSourceAdapter {
    /// 构造 adapter;name 由上层控制 multi-instance。
    #[must_use]
    pub fn new(
        name: impl Into<Cow<'static, str>>,
        config: WavefrontConfig,
        id_source: Box<dyn FrontierGoalIdSource + Send>,
    ) -> Self {
        let name = name.into();
        let blacklist_radius_m = config.safe_distance_m;
        let max_consecutive_failures = config.max_consecutive_failures;
        Self {
            name,
            prepared: None,
            selector: WavefrontGoalSelector::new(config),
            session: ExplorationSession::new(),
            id_source,
            cached_costmap: RefCell::new(None),
            cached_odom: RefCell::new(None),
            is_active: false,
            last_goal_id: None,
            last_goal: None,
            last_frontiers: Vec::new(),
            last_error: None,
            boundary: None,
            max_navigation_retries: 1,
            blacklist_radius_m,
            max_consecutive_failures,
            robot_geometry: None,
        }
    }

    /// 构造默认 adapter(使用 `CountingIdSource`,生产可替换为 uuid v7 / 确定性 hash)
    #[must_use]
    pub fn with_default_id(name: impl Into<Cow<'static, str>>, config: WavefrontConfig) -> Self {
        Self::new(name, config, Box::new(CountingIdSource::default()))
    }

    /// 接入同一导航标定中的传感器观察模型。
    pub fn set_observation_model(
        &mut self,
        model: nuwax_nav_types_messages::ObservationModel,
    ) -> ExploreResult<()> {
        if self.prepared.is_some() {
            return Err(ExploreError::PreparedPollBusy);
        }
        self.selector
            .set_observation_model(model)
            .map_err(|error| ExploreError::SourceError(error.to_string()))
    }
    /// 使用原拥有方捕获的机体外形。
    pub fn set_robot_geometry(&mut self, geometry: nuwax_nav_types_messages::RobotGeometry) {
        assert!(
            self.prepared.is_none(),
            "resolve prepared poll before configuration"
        );
        self.robot_geometry = Some(geometry);
    }

    /// 替换 id source(便于上层注入 deterministic generator)
    pub fn set_id_source(&mut self, id_source: Box<dyn FrontierGoalIdSource + Send>) {
        assert!(
            self.prepared.is_none(),
            "resolve prepared poll before configuration"
        );
        self.id_source = id_source;
    }

    /// 取出缓存的 costmap(若有)
    #[must_use]
    pub fn cached_costmap(&self) -> Option<OccupancyGrid2D> {
        self.cached_costmap.borrow().clone()
    }

    /// 设置会话 boundary 和 child navigation retry/blacklist 策略。
    pub fn configure_constraints(
        &mut self,
        boundary: Option<ExplorationBoundary>,
        max_navigation_retries: u32,
        blacklist_radius_m: f64,
    ) {
        assert!(
            self.prepared.is_none(),
            "resolve prepared poll before configuration"
        );
        self.boundary = boundary;
        self.max_navigation_retries = max_navigation_retries;
        self.blacklist_radius_m = blacklist_radius_m.max(0.0);
    }
}

impl WavefrontInputReceiver for WavefrontGoalSourceAdapter {
    fn ingest_costmap(&mut self, costmap: OccupancyGrid2D) -> ExploreResult<()> {
        if costmap.width == 0 || costmap.height == 0 {
            return Err(ExploreError::MissingCostmap);
        }
        if costmap.cells.len() != (costmap.width as usize) * (costmap.height as usize) {
            return Err(ExploreError::SourceError(
                "costmap cells length does not match width * height".to_string(),
            ));
        }
        *self.cached_costmap.borrow_mut() = Some(costmap);
        Ok(())
    }

    fn ingest_odometry(
        &mut self,
        pose: nuwax_nav_types_messages::geometry::PoseStamped,
    ) -> ExploreResult<()> {
        *self.cached_odom.borrow_mut() = Some(pose);
        Ok(())
    }
}

impl ExplorationGoalSource for WavefrontGoalSourceAdapter {
    fn checkpoint(&self) -> ExploreResult<String> {
        self.session
            .validate_checkpoint()
            .map_err(ExploreError::SourceError)?;
        serde_json::to_string(&(1_u32, &self.session))
            .map_err(|e| ExploreError::SourceError(e.to_string()))
    }
    fn restore_checkpoint(&mut self, payload: &str) -> ExploreResult<()> {
        if self.prepared.is_some() {
            return Err(ExploreError::PreparedPollBusy);
        }
        if self.last_goal.is_some() || payload.len() > 1_048_576 {
            return Err(ExploreError::SourceError(
                "二维探索恢复时已有目标或载荷过大".into(),
            ));
        }
        let (version, session): (u32, ExplorationSession) =
            serde_json::from_str(payload).map_err(|e| ExploreError::SourceError(e.to_string()))?;
        if version != 1 {
            return Err(ExploreError::SourceError("二维探索进度版本不支持".into()));
        }
        session
            .validate_checkpoint()
            .map_err(ExploreError::SourceError)?;
        self.session = session;
        self.last_frontiers.clear();
        Ok(())
    }
    fn restore_checkpoint_for_replan(&mut self, payload: &str) -> ExploreResult<()> {
        self.restore_checkpoint(payload)?;
        self.session.prepare_for_replan();
        Ok(())
    }
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }

    fn begin(&mut self) -> ExploreResult<()> {
        if self.prepared.is_some() {
            return Err(ExploreError::PreparedPollBusy);
        }
        self.is_active = true;
        self.session = ExplorationSession::new();
        self.last_goal_id = None;
        self.last_goal = None;
        self.last_frontiers.clear();
        self.last_error = None;
        Ok(())
    }

    fn end(&mut self) -> ExploreResult<()> {
        if self.prepared.is_some() {
            return Err(ExploreError::PreparedPollBusy);
        }
        self.is_active = false;
        Ok(())
    }

    fn poll_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<ExplorationPoll> {
        self.prepare_poll(state)?;
        self.commit_prepared_poll()
    }

    fn supports_prepared_poll(&self) -> bool {
        true
    }

    fn prepare_poll(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<crate::PreparedExplorationPoll> {
        if self.prepared.is_some() {
            return Err(ExploreError::PreparedPollBusy);
        }
        enum PrepareWavefrontValue {
            Early(ExplorationPoll),
            Selection(nuwax_explore_wavefront::selector::PreparedWavefrontSelection),
        }
        let mut error_update = None;
        let value = (|| -> ExploreResult<PrepareWavefrontValue> {
            if !self.is_active {
                return Ok(PrepareWavefrontValue::Early(
                    ExplorationPoll::SourceUnavailable("wavefront source not active".to_string()),
                ));
            }
            if self.last_goal.is_some() {
                return Ok(PrepareWavefrontValue::Early(ExplorationPoll::Pending));
            }
            if self.session.consecutive_failures() >= self.max_consecutive_failures
                && self.max_consecutive_failures > 0
            {
                return Ok(PrepareWavefrontValue::Early(ExplorationPoll::Stop(
                    crate::ExplorationStopReason::ConsecutiveFailuresExceeded,
                )));
            }
            // 优先用 snapshot 中的 costmap,否则用缓存
            let cached_costmap = self.cached_costmap.borrow();
            let Some(observations) = state.global_costmap.or(cached_costmap.as_ref()) else {
                error_update = Some(Some("missing costmap".to_string()));
                return Ok(PrepareWavefrontValue::Early(
                    ExplorationPoll::SourceUnavailable("missing costmap".to_string()),
                ));
            };
            let mut costmap = observations.clone();

            let robot_pose = state.robot_position();
            if let Some(geometry) = self.robot_geometry.as_ref() {
                let footprint = geometry
                    .grid_footprint(
                        costmap.resolution,
                        state.robot_pose.pose.orientation.yaw(),
                        costmap.width,
                        costmap.height,
                    )
                    .map_err(ExploreError::SourceError)?;
                // 当前机体已经占据这一区域，范围约束在随后施加，不写回规范地图。
                footprint.clear_current_occupancy(&mut costmap, robot_pose);
                self.selector.set_grid_footprint(Some(footprint));
            } else {
                self.selector.set_grid_footprint(None);
            }
            if let Some(boundary) = &self.boundary {
                if !boundary.contains(Point2::new(robot_pose.x, robot_pose.y)) {
                    error_update = Some(Some("robot pose outside exploration boundary".to_owned()));
                    return Ok(PrepareWavefrontValue::Early(ExplorationPoll::Stop(
                        crate::ExplorationStopReason::NoReachableFreeSpace,
                    )));
                }
                // 在候选搜索前应用边界，目标及其机体轮廓与导航使用同一区域。
                // 只修改私有候选栅格，范围外未知不作为新的待探索前沿。
                for y in 0..costmap.height {
                    for x in 0..costmap.width {
                        if costmap
                            .cell_center(x, y)
                            .is_some_and(|point| !boundary.contains(point))
                        {
                            costmap.set(
                                x,
                                y,
                                nuwax_nav_types_messages::grid::OccupancyCell::Occupied,
                            );
                        }
                    }
                }
            }
            let now = state.now;

            Ok(PrepareWavefrontValue::Selection(
                self.selector.prepare_goal_with_observations(
                    Point2 {
                        x: robot_pose.x,
                        y: robot_pose.y,
                    },
                    &costmap,
                    Some(observations),
                    &self.session,
                    now,
                ),
            ))
        })();
        let (pending, outcome) = match value {
            Ok(PrepareWavefrontValue::Selection(selection)) => {
                let outcome = self.selection_preview(&selection);
                (PendingWavefrontPoll::Selection(selection), outcome)
            }
            Ok(PrepareWavefrontValue::Early(poll)) => {
                let outcome = Ok(poll);
                (
                    PendingWavefrontPoll::Ready {
                        outcome: outcome.clone(),
                        error: error_update,
                    },
                    outcome,
                )
            }
            Err(error) => {
                let outcome = Err(error);
                (
                    PendingWavefrontPoll::Ready {
                        outcome: outcome.clone(),
                        error: error_update,
                    },
                    outcome,
                )
            }
        };
        self.prepared = Some(pending);
        Ok(crate::PreparedExplorationPoll::new(outcome))
    }

    fn commit_prepared_poll(&mut self) -> ExploreResult<ExplorationPoll> {
        match self.prepared.take().ok_or(ExploreError::NoPreparedPoll)? {
            PendingWavefrontPoll::Ready { outcome, error } => {
                if let Some(error) = error {
                    self.last_error = error;
                }
                outcome
            }
            PendingWavefrontPoll::Selection(selection) => {
                let selection = selection.resolve(&mut self.session, &mut *self.id_source);
                self.apply_selection(selection)
            }
        }
    }

    fn discard_prepared_poll(&mut self) -> ExploreResult<()> {
        self.prepared.take().ok_or(ExploreError::NoPreparedPoll)?;
        Ok(())
    }

    fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> ExploreResult<()> {
        if self.prepared.is_some() {
            return Err(ExploreError::PreparedPollBusy);
        }
        let Some(goal) = self.last_goal.as_ref() else {
            return Ok(());
        };
        if goal.id != goal_id {
            return Ok(());
        }
        let point = goal.pose.pose.position.to_2d();
        match feedback {
            ExplorationGoalFeedback::InputUnavailable(reason) => {
                self.last_error = Some(format!("等待导航输入恢复：{reason}"));
            }
            ExplorationGoalFeedback::Reached => {
                self.session.record_goal_reached(point);
                self.last_error = None;
            }
            ExplorationGoalFeedback::Failed(reason) => {
                let blacklisted = self.session.record_goal_failed(
                    point,
                    self.max_navigation_retries,
                    self.blacklist_radius_m,
                );
                self.last_error = Some(if blacklisted {
                    format!("navigation failed and frontier blacklisted: {reason}")
                } else {
                    format!("navigation failed; retry remains available: {reason}")
                });
            }
            ExplorationGoalFeedback::Cancelled => {
                self.last_error = Some("navigation goal cancelled".to_string());
            }
        }
        self.last_goal = None;
        self.last_goal_id = None;
        self.last_frontiers.clear();
        Ok(())
    }

    fn frontier_snapshot(&self) -> Vec<nuwax_nav_types_messages::visual::FrontierGoal> {
        self.last_frontiers.clone()
    }

    fn diagnostics(&self) -> ExplorationSourceDiagnostics {
        ExplorationSourceDiagnostics {
            source_name: self.name.to_string(),
            is_active: self.is_active,
            last_goal_id: self.last_goal_id.clone(),
            last_error: self.last_error.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn legacy_replan_selects_again_under_current_constraints() {
        let mut previous =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        previous.begin().unwrap();
        previous
            .session
            .record_goal_reached(Point2::new(12.5, -3.25));
        previous
            .session
            .blacklist_goal(Point2::new(2.25, 2.25), 100.0);
        previous.session.record_failure();
        let payload = previous.checkpoint().unwrap();
        let mut restored =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        restored.configure_constraints(None, 8, 0.25);
        restored.begin().unwrap();
        restored.restore_checkpoint_for_replan(&payload).unwrap();
        assert_eq!(restored.session.blacklist_count(), 0);
        let grid = frontier_grid();
        let mut state = sample_state(Some(&grid), Timestamp::default());
        state.robot_pose.pose.position.x = 2.25;
        state.robot_pose.pose.position.y = 2.25;
        assert!(matches!(
            restored.poll_goal(&state).unwrap(),
            ExplorationPoll::Goal(_)
        ));
        assert_eq!(previous.checkpoint().unwrap(), payload);
    }

    use super::*;
    use nuwax_nav_types_messages::frames::NavFrameId;
    use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped, Quaternion, Timestamp};
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

    fn sample_state<'a>(
        costmap: Option<&'a OccupancyGrid2D>,
        now: Timestamp,
    ) -> ExplorationStateSnapshot<'a> {
        let pose = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: now,
            pose: Pose3 {
                position: Point3 {
                    x: 1.0,
                    y: 1.0,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        ExplorationStateSnapshot {
            robot_pose: pose,
            global_costmap: costmap,
            #[cfg(feature = "surface")]
            frozen_surface_input: None,
            active_goal: None,
            latest_frontier_goal: None,
            now,
        }
    }

    fn frontier_grid() -> OccupancyGrid2D {
        let mut grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 12,
            height: 12,
            cells: vec![OccupancyCell::Unknown; 144],
        };
        for y in 2..=7 {
            for x in 2..=7 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        grid
    }

    #[test]
    fn input_interruption_leaves_planar_frontier_eligible_after_repeated_stops() {
        let mut source =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        source.configure_constraints(None, 0, 0.75);
        source.begin().unwrap();
        let map = frontier_grid();
        let mut state = sample_state(Some(&map), Timestamp::default());
        state.robot_pose.pose.position.x = 2.25;
        state.robot_pose.pose.position.y = 2.25;
        for _ in 0..3 {
            let ExplorationPoll::Goal(goal) = source.poll_goal(&state).unwrap() else {
                panic!("输入中断不能耗尽前沿预算")
            };
            source
                .report_goal_feedback(
                    &goal.id,
                    ExplorationGoalFeedback::InputUnavailable("输入超时，已停车".into()),
                )
                .unwrap();
            assert!(!source
                .session
                .is_blacklisted(goal.pose.pose.position.to_2d()));
            assert_eq!(source.session.regional_completed_rounds(), 0);
        }
    }

    #[test]
    fn calibrated_planar_source_uses_visible_frontier_cells_and_observation_yaw() {
        let mut source =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        source
            .set_observation_model(nuwax_nav_types_messages::ObservationModel {
                kind: nuwax_nav_types_messages::ObservationKind::Planar,
                min_range_m: 0.05,
                max_range_m: 2.8,
                horizontal_fov_rad: std::f64::consts::FRAC_PI_2,
                vertical_min_rad: 0.0,
                vertical_max_rad: 0.0,
                sensor_translation_body_m: [0.0; 3],
                sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
                max_rays: 128,
            })
            .unwrap();
        source.begin().unwrap();
        let map = frontier_grid();
        let mut state = sample_state(Some(&map), Timestamp::default());
        state.robot_pose.pose.position.x = 2.25;
        state.robot_pose.pose.position.y = 2.25;
        let ExplorationPoll::Goal(goal) = source.poll_goal(&state).unwrap() else {
            panic!("可见前沿应形成目标")
        };
        assert_eq!(
            goal.metadata.get("gain_basis").map(String::as_str),
            Some("calibrated_visible_frontier_cells")
        );
        assert!(goal.information_gain.unwrap() > 0.0);
        assert!(goal.pose.pose.orientation.yaw().is_finite());
    }

    #[test]
    fn current_body_observation_does_not_block_frontier_and_does_not_change_source_map() {
        use nuwax_nav_types_messages::RobotGeometry;
        let geometry = RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]],"z_min_m":-0.3,"z_max_m":0.1}"#).unwrap();
        for self_return in [
            OccupancyCell::Unknown,
            OccupancyCell::Occupied,
            OccupancyCell::Cost(83),
        ] {
            let mut grid = frontier_grid();
            let position = Point2::new(2.25, 2.25);
            let footprint = geometry
                .grid_footprint(grid.resolution, 0.0, grid.width, grid.height)
                .unwrap();
            for &(dx, dy) in &footprint.offsets {
                grid.set((4 + dx) as u32, (4 + dy) as u32, self_return);
            }
            let original = grid.clone();
            let mut adapter = WavefrontGoalSourceAdapter::with_default_id(
                "wavefront",
                WavefrontConfig::default(),
            );
            adapter.set_robot_geometry(geometry.clone());
            adapter.begin().unwrap();
            let mut state = sample_state(Some(&grid), Timestamp::default());
            state.robot_pose.pose.position.x = position.x;
            state.robot_pose.pose.position.y = position.y;
            assert!(
                matches!(adapter.poll_goal(&state).unwrap(), ExplorationPoll::Goal(_)),
                "当前机体的缺测或回波不得困住起点"
            );
            assert_eq!(grid, original, "自体处理只能作用于规划视图");
        }
    }

    #[test]
    fn missing_costmap_returns_unavailable() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.begin().unwrap();
        let s = sample_state(None, Timestamp::default());
        let poll = ad.poll_goal(&s).unwrap();
        assert!(matches!(poll, ExplorationPoll::SourceUnavailable(_)));
    }

    #[test]
    fn initial_unknown_footprint_waits_then_selects_goal_after_real_observation() {
        let mut controller =
            crate::ExplorationController::new(crate::ExplorationArbitrationPolicy::FirstAvailable);
        controller.add_source_boxed(Box::new(WavefrontGoalSourceAdapter::with_default_id(
            "wavefront",
            WavefrontConfig::default(),
        )));
        controller.begin_exploration().unwrap();
        let mut incomplete = frontier_grid();
        incomplete.set(4, 4, OccupancyCell::Unknown);
        let mut state = sample_state(Some(&incomplete), Timestamp::default());
        state.robot_pose.pose.position.x = 2.25;
        state.robot_pose.pose.position.y = 2.25;
        assert!(matches!(controller.poll_next_goal(&state).unwrap(),
            ExplorationPoll::SourceUnavailable(reason) if reason.contains("not fully observed")));
        assert!(controller.is_active(), "缺测不能关闭探索控制器");
        assert!(controller.frontier_snapshot().is_empty());

        let complete = frontier_grid();
        state.global_costmap = Some(&complete);
        assert!(
            matches!(
                controller.poll_next_goal(&state).unwrap(),
                ExplorationPoll::Goal(_)
            ),
            "后续真实地图补齐后应直接产生前沿目标，不必重启探索"
        );
        controller
            .stop_with_reason(crate::ExplorationStopReason::UserCancelled)
            .unwrap();
        controller.begin_exploration().unwrap();
        state.global_costmap = Some(&incomplete);
        assert!(matches!(
            controller.poll_next_goal(&state).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
        let mut blocked = frontier_grid();
        blocked.set(4, 4, OccupancyCell::Cost(50));
        state.global_costmap = Some(&blocked);
        assert!(matches!(
            controller.poll_next_goal(&state).unwrap(),
            ExplorationPoll::Stop(crate::ExplorationStopReason::NoReachableFreeSpace)
        ));
        assert!(
            controller
                .diagnostics()
                .sources
                .iter()
                .all(|source| source.last_error.is_none()),
            "已知障碍终态不能继续携带上一轮缺测诊断"
        );
    }

    #[test]
    fn free_grid_reports_no_frontier() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.begin().unwrap();
        let grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 16],
        };
        let s = sample_state(Some(&grid), Timestamp::default());
        let poll = ad.poll_goal(&s).unwrap();
        assert!(matches!(poll, ExplorationPoll::NoFrontier));
    }

    #[test]
    fn ingested_free_costmap_reports_no_frontier() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.begin().unwrap();
        let grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 16],
        };
        ad.ingest_costmap(grid).unwrap();
        let s = sample_state(None, Timestamp::default());
        let poll = ad.poll_goal(&s).unwrap();
        assert!(matches!(poll, ExplorationPoll::NoFrontier));
    }

    #[test]
    fn ingest_costmap_mismatched_returns_error() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        let bad = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 4], // 长度不匹配
        };
        let err = ad.ingest_costmap(bad).unwrap_err();
        assert!(matches!(err, ExploreError::SourceError(_)));
    }

    #[test]
    fn facade_goal_uses_selector_id_source() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.begin().unwrap();
        // free grid → Pending;我们只验证 id source 已经被 selector 使用过一次,
        // 通过 diagnostics 看不到,但 selector 会持有 id_source 的 mutable borrow;
        // 此处只验证 begin / poll 不 panic。
        let grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 16],
        };
        let s = sample_state(Some(&grid), Timestamp::default());
        let _ = ad.poll_goal(&s).unwrap();
    }

    #[test]
    fn planar_regional_age_ignores_old_feedback_cancellation_and_session_restart() {
        let mut adapter =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        let grid = frontier_grid();
        let mut state = sample_state(Some(&grid), Timestamp::default());
        state.robot_pose.pose.position.x = 2.25;
        state.robot_pose.pose.position.y = 2.25;
        adapter.begin().unwrap();
        let ExplorationPoll::Goal(selected) = adapter.poll_goal(&state).unwrap() else {
            panic!("需要安全目标");
        };
        adapter
            .report_goal_feedback("old-goal", ExplorationGoalFeedback::Failed("late".into()))
            .unwrap();
        assert_eq!(adapter.session.regional_completed_rounds(), 0);
        adapter
            .report_goal_feedback(
                &selected.id,
                ExplorationGoalFeedback::Failed("blocked".into()),
            )
            .unwrap();
        adapter
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        assert_eq!(adapter.session.regional_completed_rounds(), 1);
        adapter.end().unwrap();
        adapter.begin().unwrap();
        assert_eq!(adapter.session.regional_completed_rounds(), 0);
        let ExplorationPoll::Goal(selected) = adapter.poll_goal(&state).unwrap() else {
            panic!("需要安全目标");
        };
        adapter
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert_eq!(adapter.session.regional_completed_rounds(), 0);
    }

    #[test]
    fn production_planar_source_reports_shared_regional_selection() {
        let mut source =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        source.begin().unwrap();
        let map = frontier_grid();
        let mut state = sample_state(Some(&map), Timestamp::default());
        state.robot_pose.pose.position.x = 2.25;
        state.robot_pose.pose.position.y = 2.25;
        let ExplorationPoll::Goal(goal) = source.poll_goal(&state).unwrap() else {
            panic!("安全前沿应可用");
        };
        assert_eq!(
            goal.metadata.get("selection_method").map(String::as_str),
            Some("regional_route")
        );
        assert!(goal
            .distance_cost
            .is_some_and(|cost| cost.is_finite() && cost > 0.0));
    }

    #[test]
    fn selected_goal_stays_in_flight_until_navigation_feedback() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.begin().unwrap();
        let grid = frontier_grid();
        let mut s = sample_state(Some(&grid), Timestamp::default());
        // 机器人必须在已知自由区域内，不能依赖跨未知区域寻找替代起点。
        s.robot_pose.pose.position.x = 2.25;
        s.robot_pose.pose.position.y = 2.25;

        let first = ad.poll_goal(&s).unwrap();
        let goal = match first {
            ExplorationPoll::Goal(goal) => goal,
            other => panic!("expected frontier goal, got {other:?}"),
        };
        assert_eq!(ad.frontier_snapshot().len(), 1);
        assert!(matches!(
            ad.poll_goal(&s).unwrap(),
            ExplorationPoll::Pending
        ));

        ad.report_goal_feedback(&goal.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        assert!(ad.frontier_snapshot().is_empty());
        assert!(ad.last_goal.is_none());
    }

    #[test]
    fn boundary_rejects_out_of_area_frontier() {
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.configure_constraints(
            Some(
                ExplorationBoundary::from_vertices(vec![
                    Point2 { x: 0.0, y: 0.0 },
                    Point2 { x: 0.9, y: 0.0 },
                    Point2 { x: 0.9, y: 0.9 },
                    Point2 { x: 0.0, y: 0.9 },
                ])
                .unwrap(),
            ),
            0,
            0.5,
        );
        ad.begin().unwrap();
        let grid = frontier_grid();
        let mut s = sample_state(Some(&grid), Timestamp::default());
        s.robot_pose.pose.position.x = 2.25;
        s.robot_pose.pose.position.y = 2.25;

        assert!(matches!(
            ad.poll_goal(&s).unwrap(),
            ExplorationPoll::Stop(crate::ExplorationStopReason::NoReachableFreeSpace)
        ));
        assert!(ad.frontier_snapshot().is_empty());
        assert!(ad
            .last_error
            .as_deref()
            .is_some_and(|error| error.contains("outside exploration boundary")));
    }

    #[test]
    fn known_bounded_region_does_not_create_frontiers_outside_boundary() {
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 40, 40, OccupancyCell::Unknown);
        for y in 10..30 {
            for x in 10..30 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        let mut ad =
            WavefrontGoalSourceAdapter::with_default_id("wavefront", WavefrontConfig::default());
        ad.configure_constraints(
            Some(
                ExplorationBoundary::from_vertices(vec![
                    Point2::new(1.0, 1.0),
                    Point2::new(3.0, 1.0),
                    Point2::new(3.0, 3.0),
                    Point2::new(1.0, 3.0),
                ])
                .unwrap(),
            ),
            1,
            0.75,
        );
        ad.begin().unwrap();
        let mut state = sample_state(Some(&grid), Timestamp::default());
        state.robot_pose.pose.position.x = 2.0;
        state.robot_pose.pose.position.y = 2.0;
        assert!(matches!(
            ad.poll_goal(&state).unwrap(),
            ExplorationPoll::NoFrontier
        ));
        assert!(ad.frontier_snapshot().is_empty());
        assert_eq!(
            grid.get(0, 0),
            Some(OccupancyCell::Unknown),
            "原始观测不得被边界掩码修改"
        );
    }
}

impl WavefrontGoalSourceAdapter {
    fn apply_selection(
        &mut self,
        selection: Result<GoalSelection, WavefrontError>,
    ) -> ExploreResult<ExplorationPoll> {
        match selection {
            Ok(GoalSelection::Goal(goal)) => {
                let goal_point = goal.pose.pose.position.to_2d();
                if self
                    .boundary
                    .as_ref()
                    .is_some_and(|boundary| !boundary.contains(goal_point))
                {
                    self.session
                        .blacklist_goal(goal_point, self.blacklist_radius_m);
                    self.last_frontiers.clear();
                    self.last_error = Some(format!(
                        "frontier {} rejected: outside exploration boundary",
                        goal.id
                    ));
                    return Ok(ExplorationPoll::Pending);
                }
                let id = goal.id.clone();
                self.session.record_goal_published();
                self.last_frontiers = vec![goal.clone()];
                self.last_goal = Some(goal.clone());
                let polled = ExplorationPoll::Goal(goal);
                self.last_goal_id = Some(id);
                self.last_error = None;
                Ok(polled)
            }
            Ok(GoalSelection::NoFrontier) => {
                self.last_frontiers.clear();
                self.last_error = None;
                Ok(ExplorationPoll::NoFrontier)
            }
            Ok(GoalSelection::Stop(stop)) => {
                let facade_stop = match stop {
                    StopReason::NoInformationGain => {
                        crate::ExplorationStopReason::NoInformationGain
                    }
                    StopReason::NoReachableFreeSpace => {
                        crate::ExplorationStopReason::NoReachableFreeSpace
                    }
                    StopReason::ConsecutiveFailuresExceeded => {
                        crate::ExplorationStopReason::ConsecutiveFailuresExceeded
                    }
                    // MEDIUM-2 修正:用户 stop_movement 走独立 reason
                    StopReason::UserCancelled => crate::ExplorationStopReason::UserCancelled,
                };
                self.last_error = None;
                self.last_goal = None;
                self.last_frontiers.clear();
                Ok(ExplorationPoll::Stop(facade_stop))
            }
            Err(e @ WavefrontError::RobotFootprintUnobserved(..)) => {
                // 初图缺测属于输入不可用，等待后续真实地图补齐；原父任务截止和取消仍有效。
                let msg = e.to_string();
                self.last_error = Some(msg.clone());
                self.last_frontiers.clear();
                Ok(ExplorationPoll::SourceUnavailable(msg))
            }
            Err(e) => {
                let msg = format!("{e}");
                self.last_error = Some(msg.clone());
                Err(ExploreError::SourceError(msg))
            }
        }
    }
    fn selection_preview(
        &self,
        prepared: &nuwax_explore_wavefront::selector::PreparedWavefrontSelection,
    ) -> ExploreResult<ExplorationPoll> {
        use nuwax_explore_wavefront::frontier_goal::PreparedGoalSelection;
        match &prepared.outcome {
            Ok(PreparedGoalSelection::Goal(draft)) => {
                let goal = draft.preview_goal();
                if self
                    .boundary
                    .as_ref()
                    .is_some_and(|boundary| !boundary.contains(goal.pose.pose.position.to_2d()))
                {
                    Ok(ExplorationPoll::Pending)
                } else {
                    Ok(ExplorationPoll::Goal(goal))
                }
            }
            Ok(PreparedGoalSelection::NoFrontier) => Ok(ExplorationPoll::NoFrontier),
            Ok(PreparedGoalSelection::Stop(stop)) => Ok(ExplorationPoll::Stop(match stop {
                StopReason::NoInformationGain => crate::ExplorationStopReason::NoInformationGain,
                StopReason::NoReachableFreeSpace => {
                    crate::ExplorationStopReason::NoReachableFreeSpace
                }
                StopReason::ConsecutiveFailuresExceeded => {
                    crate::ExplorationStopReason::ConsecutiveFailuresExceeded
                }
                StopReason::UserCancelled => crate::ExplorationStopReason::UserCancelled,
            })),
            Err(error @ WavefrontError::RobotFootprintUnobserved(..)) => {
                Ok(ExplorationPoll::SourceUnavailable(error.to_string()))
            }
            Err(error) => Err(ExploreError::SourceError(error.to_string())),
        }
    }
}
