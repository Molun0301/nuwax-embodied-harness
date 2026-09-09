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
    FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection, StopReason, WavefrontConfig, WavefrontError,
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
pub struct WavefrontGoalSourceAdapter {
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
        }
    }

    /// 构造默认 adapter(使用 `CountingIdSource`,生产可替换为 uuid v7 / 确定性 hash)
    #[must_use]
    pub fn with_default_id(name: impl Into<Cow<'static, str>>, config: WavefrontConfig) -> Self {
        Self::new(name, config, Box::new(CountingIdSource::default()))
    }

    /// 替换 id source(便于上层注入 deterministic generator)
    pub fn set_id_source(&mut self, id_source: Box<dyn FrontierGoalIdSource + Send>) {
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
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }

    fn begin(&mut self) -> ExploreResult<()> {
        self.is_active = true;
        self.session = ExplorationSession::new();
        self.last_goal_id = None;
        self.last_goal = None;
        self.last_frontiers.clear();
        self.last_error = None;
        Ok(())
    }

    fn end(&mut self) -> ExploreResult<()> {
        self.is_active = false;
        Ok(())
    }

    fn poll_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<ExplorationPoll> {
        if !self.is_active {
            return Ok(ExplorationPoll::SourceUnavailable(
                "wavefront source not active".to_string(),
            ));
        }
        if self.last_goal.is_some() {
            return Ok(ExplorationPoll::Pending);
        }
        if self.session.consecutive_failures() >= self.max_consecutive_failures
            && self.max_consecutive_failures > 0
        {
            return Ok(ExplorationPoll::Stop(
                crate::ExplorationStopReason::ConsecutiveFailuresExceeded,
            ));
        }
        // 优先用 snapshot 中的 costmap,否则用缓存
        let costmap_opt: Option<OccupancyGrid2D> = state
            .global_costmap
            .cloned()
            .or_else(|| self.cached_costmap.borrow().clone());
        let Some(mut costmap) = costmap_opt else {
            self.last_error = Some("missing costmap".to_string());
            return Ok(ExplorationPoll::SourceUnavailable(
                "missing costmap".to_string(),
            ));
        };

        let robot_pose = state.robot_position();
        if let Some(boundary) = &self.boundary {
            if !boundary.contains(Point2::new(robot_pose.x, robot_pose.y)) {
                self.last_error = Some("robot pose outside exploration boundary".to_owned());
                return Ok(ExplorationPoll::Stop(
                    crate::ExplorationStopReason::NoReachableFreeSpace,
                ));
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
        match self.selector.select_goal_at(
            Point2 {
                x: robot_pose.x,
                y: robot_pose.y,
            },
            &costmap,
            &mut self.session,
            now,
            &mut *self.id_source,
        ) {
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

    fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> ExploreResult<()> {
        let Some(goal) = self.last_goal.as_ref() else {
            return Ok(());
        };
        if goal.id != goal_id {
            return Ok(());
        }
        let point = goal.pose.pose.position.to_2d();
        match feedback {
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
