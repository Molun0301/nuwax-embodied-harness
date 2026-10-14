//! `TareWaypointGoalSourceAdapter`:接收 TARE bridge waypoint,
//! 通过 `TareWaypointGoalMapping` 转 `FrontierGoal`,暴露给 controller。
//!
//! 仅在 `runtime + tare` feature 启用时编译(方案 §8.5)。
//!
//! 设计:
//! - 缓存最新一个 valid waypoint 转换出的 `FrontierGoal`
//! - `poll_goal` 把缓存 goal 取出并清空(取一次);
//!   controller 在 `FirstAvailable` 之下可以连续得到 goal,直到外部再 `ingest_waypoint`。
//! - `ingest_waypoint` 验证失败时返回 `ExploreError::Conversion(...)`,
//!   不污染缓存。

#[cfg(all(feature = "runtime", feature = "tare"))]
use std::borrow::Cow;
use std::cell::RefCell;

use nuwax_explore_tare_bridge::TareWaypoint;
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::runtime::{TareWaypointGoalMapping, TareWaypointReceiver};
use crate::{
    ExplorationGoalSource, ExplorationPoll, ExplorationSourceDiagnostics, ExplorationStateSnapshot,
    ExploreResult,
};

/// TARE waypoint -> facade source adapter
pub struct TareWaypointGoalSourceAdapter {
    name: Cow<'static, str>,
    mapping: TareWaypointGoalMapping,
    cached_goal: RefCell<Option<FrontierGoal>>,
    is_active: bool,
    last_goal_id: Option<String>,
    last_error: Option<String>,
}

impl TareWaypointGoalSourceAdapter {
    /// 构造 adapter
    #[must_use]
    pub fn new(name: impl Into<Cow<'static, str>>) -> Self {
        Self::with_mapping(name, TareWaypointGoalMapping::default())
    }

    /// 构造自定义 mapping
    #[must_use]
    pub fn with_mapping(
        name: impl Into<Cow<'static, str>>,
        mapping: TareWaypointGoalMapping,
    ) -> Self {
        Self {
            name: name.into(),
            mapping,
            cached_goal: RefCell::new(None),
            is_active: false,
            last_goal_id: None,
            last_error: None,
        }
    }

    /// 取出最近缓存的 goal
    #[must_use]
    pub fn peek_cached_goal(&self) -> Option<FrontierGoal> {
        self.cached_goal.borrow().clone()
    }
}

impl TareWaypointReceiver for TareWaypointGoalSourceAdapter {
    fn ingest_waypoint(&mut self, waypoint: TareWaypoint) -> ExploreResult<()> {
        match self.mapping.waypoint_to_frontier_goal(&waypoint) {
            Ok(goal) => {
                self.last_goal_id = Some(goal.id.clone());
                *self.cached_goal.borrow_mut() = Some(goal);
                Ok(())
            }
            Err(e) => {
                let msg = format!("{e}");
                self.last_error = Some(msg.clone());
                Err(e)
            }
        }
    }
}

impl ExplorationGoalSource for TareWaypointGoalSourceAdapter {
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }

    fn begin(&mut self) -> ExploreResult<()> {
        self.is_active = true;
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
        _state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<ExplorationPoll> {
        // 取出并清空缓存(take-once)
        let taken = self.cached_goal.borrow_mut().take();
        match taken {
            Some(goal) => Ok(ExplorationPoll::Goal(goal)),
            None => Ok(ExplorationPoll::Pending),
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
    use crate::ExploreError;
    use nuwax_nav_types_messages::frames::NavFrameId;
    use nuwax_nav_types_messages::geometry::Timestamp;

    fn sample_waypoint() -> TareWaypoint {
        TareWaypoint::new(
            NavFrameId::Map,
            Timestamp::from_unix(1_700_000_000, 0),
            1.0,
            2.0,
            0.0,
        )
    }

    fn state() -> ExplorationStateSnapshot<'static> {
        ExplorationStateSnapshot::from_xy((0.0, 0.0), Timestamp::default())
    }

    #[test]
    fn empty_cache_returns_pending() {
        let mut ad = TareWaypointGoalSourceAdapter::new("tare");
        ad.begin().unwrap();
        let poll = ad.poll_goal(&state()).unwrap();
        assert!(matches!(poll, ExplorationPoll::Pending));
    }

    #[test]
    fn ingest_then_poll_returns_goal() {
        let mut ad = TareWaypointGoalSourceAdapter::new("tare");
        ad.begin().unwrap();
        ad.ingest_waypoint(sample_waypoint()).unwrap();
        let poll = ad.poll_goal(&state()).unwrap();
        match poll {
            ExplorationPoll::Goal(g) => {
                assert!((g.pose.pose.position.x - 1.0).abs() < 1e-9);
            }
            _ => panic!("expected Goal"),
        }
        // 缓存已被 take 清空
        let poll2 = ad.poll_goal(&state()).unwrap();
        assert!(matches!(poll2, ExplorationPoll::Pending));
    }

    #[test]
    fn invalid_waypoint_returns_conversion_error() {
        let mut ad = TareWaypointGoalSourceAdapter::new("tare");
        ad.begin().unwrap();
        let mut wp = sample_waypoint();
        wp.x = f64::NAN;
        let err = ad.ingest_waypoint(wp).unwrap_err();
        assert!(matches!(err, ExploreError::Conversion(_)));
    }
}
