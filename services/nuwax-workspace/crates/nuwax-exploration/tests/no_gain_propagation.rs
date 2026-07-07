//! no-gain / stop reason 传播测试(方案 §8.2)
//!
//! 覆盖:
//! - 单 source 发出 `Stop(NoInformationGain)` 之后,controller 不 fallback
//! - `EndExploration` 后 stop reason = `UserCancelled`

use std::borrow::Cow;

use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationController, ExplorationGoalSource, ExplorationPoll,
    ExplorationSourceDiagnostics, ExplorationStateSnapshot, ExplorationStopReason, ExploreError,
};

#[derive(Debug)]
struct StopSource {
    name: Cow<'static, str>,
    poll_count: std::cell::Cell<u32>,
}

impl StopSource {
    fn new(name: &'static str) -> Self {
        Self {
            name: Cow::Borrowed(name),
            poll_count: std::cell::Cell::new(0),
        }
    }
}

impl ExplorationGoalSource for StopSource {
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }
    fn begin(&mut self) -> Result<(), ExploreError> {
        Ok(())
    }
    fn end(&mut self) -> Result<(), ExploreError> {
        Ok(())
    }
    fn poll_goal(
        &mut self,
        _state: &ExplorationStateSnapshot<'_>,
    ) -> Result<ExplorationPoll, ExploreError> {
        let n = self.poll_count.get() + 1;
        self.poll_count.set(n);
        Ok(ExplorationPoll::Stop(
            ExplorationStopReason::NoInformationGain,
        ))
    }
    fn diagnostics(&self) -> ExplorationSourceDiagnostics {
        ExplorationSourceDiagnostics {
            source_name: self.name.to_string(),
            is_active: true,
            last_goal_id: None,
            last_error: None,
        }
    }
}

#[derive(Debug)]
struct FallbackGoalSource {
    name: Cow<'static, str>,
}

impl ExplorationGoalSource for FallbackGoalSource {
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }
    fn begin(&mut self) -> Result<(), ExploreError> {
        Ok(())
    }
    fn end(&mut self) -> Result<(), ExploreError> {
        Ok(())
    }
    fn poll_goal(
        &mut self,
        _state: &ExplorationStateSnapshot<'_>,
    ) -> Result<ExplorationPoll, ExploreError> {
        // 在 fallback 链上提供 Goal,但 stop 必须截断
        use nuwax_nav_types_messages::frames::NavFrameId;
        use nuwax_nav_types_messages::geometry::{
            Point3, Pose3, PoseStamped, Quaternion, Timestamp,
        };
        use nuwax_nav_types_messages::visual::FrontierGoal;
        let pose = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            pose: Pose3 {
                position: Point3 {
                    x: 1.0,
                    y: 1.0,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        Ok(ExplorationPoll::Goal(FrontierGoal::new("fb", pose, 0.5)))
    }
    fn diagnostics(&self) -> ExplorationSourceDiagnostics {
        ExplorationSourceDiagnostics {
            source_name: "fb".to_string(),
            is_active: true,
            last_goal_id: None,
            last_error: None,
        }
    }
}

#[test]
fn stop_does_not_fall_back_to_other_source() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    let fb = FallbackGoalSource {
        name: Cow::Borrowed("fb"),
    };
    ctrl.add_source(StopSource::new("stopper"));
    ctrl.add_source(fb);
    ctrl.begin_exploration().unwrap();
    let poll = ctrl
        .poll_next_goal(&ExplorationStateSnapshot::from_xy(
            (0.0, 0.0),
            nuwax_nav_types_messages::geometry::Timestamp::default(),
        ))
        .unwrap();
    match poll {
        ExplorationPoll::Stop(r) => assert_eq!(r, ExplorationStopReason::NoInformationGain),
        other => panic!("expected Stop(NoInformationGain), got {other:?}"),
    }
    assert!(!ctrl.is_active());
}

#[test]
fn end_exploration_records_user_cancelled_stop_reason() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.add_source(FallbackGoalSource {
        name: Cow::Borrowed("fb"),
    });
    ctrl.begin_exploration().unwrap();
    ctrl.end_exploration().unwrap();
    assert_eq!(
        ctrl.diagnostics().last_stop_reason,
        Some(ExplorationStopReason::UserCancelled)
    );
}
