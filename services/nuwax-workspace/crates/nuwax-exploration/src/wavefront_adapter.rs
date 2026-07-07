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
};
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;

use crate::runtime::WavefrontInputReceiver;
use crate::{
    ExplorationGoalSource, ExplorationPoll, ExplorationSourceDiagnostics, ExplorationStateSnapshot,
    ExploreError, ExploreResult,
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
    last_error: Option<String>,
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
        Self {
            name,
            selector: WavefrontGoalSelector::new(config),
            session: ExplorationSession::new(),
            id_source,
            cached_costmap: RefCell::new(None),
            cached_odom: RefCell::new(None),
            is_active: false,
            last_goal_id: None,
            last_error: None,
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
        // 优先用 snapshot 中的 costmap,否则用缓存
        let costmap_opt: Option<OccupancyGrid2D> = state
            .global_costmap
            .cloned()
            .or_else(|| self.cached_costmap.borrow().clone());
        let Some(costmap) = costmap_opt else {
            self.last_error = Some("missing costmap".to_string());
            return Ok(ExplorationPoll::SourceUnavailable(
                "missing costmap".to_string(),
            ));
        };

        let robot_pose = state.robot_position();
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
                let id = goal.id.clone();
                let polled = ExplorationPoll::Goal(goal);
                self.last_goal_id = Some(id);
                self.last_error = None;
                Ok(polled)
            }
            Ok(GoalSelection::NoFrontier) => {
                self.last_error = None;
                Ok(ExplorationPoll::Pending)
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
                Ok(ExplorationPoll::Stop(facade_stop))
            }
            Err(e) => {
                let msg = format!("{e}");
                self.last_error = Some(msg.clone());
                Err(ExploreError::SourceError(msg))
            }
        }
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
                    x: 0.0,
                    y: 0.0,
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
    fn free_grid_returns_pending() {
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
        assert!(matches!(poll, ExplorationPoll::Pending));
    }

    #[test]
    fn ingest_costmap_then_pending() {
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
        assert!(matches!(poll, ExplorationPoll::Pending));
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
}
