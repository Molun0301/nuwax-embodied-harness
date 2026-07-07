//! `begin_exploration` reverse-order rollback 测试(方案 §8.2)

use std::borrow::Cow;

use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationController, ExplorationGoalSource, ExplorationPoll,
    ExplorationSourceDiagnostics, ExplorationStateSnapshot, ExploreError,
};

#[derive(Debug)]
struct FailOnBeginSource {
    name: Cow<'static, str>,
    begin_should_fail: bool,
    end_called: std::cell::Cell<u32>,
}

impl FailOnBeginSource {
    fn new(name: &'static str, fail: bool) -> Self {
        Self {
            name: Cow::Borrowed(name),
            begin_should_fail: fail,
            end_called: std::cell::Cell::new(0),
        }
    }
}

impl ExplorationGoalSource for FailOnBeginSource {
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }
    fn begin(&mut self) -> Result<(), ExploreError> {
        if self.begin_should_fail {
            Err(ExploreError::SourceError("begin failed".into()))
        } else {
            Ok(())
        }
    }
    fn end(&mut self) -> Result<(), ExploreError> {
        self.end_called.set(self.end_called.get() + 1);
        Ok(())
    }
    fn poll_goal(
        &mut self,
        _state: &ExplorationStateSnapshot<'_>,
    ) -> Result<ExplorationPoll, ExploreError> {
        Ok(ExplorationPoll::Pending)
    }
    fn diagnostics(&self) -> ExplorationSourceDiagnostics {
        ExplorationSourceDiagnostics {
            source_name: self.name.to_string(),
            is_active: false,
            last_goal_id: None,
            last_error: None,
        }
    }
}

#[test]
fn begin_failure_rolls_back_active_state() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    let a = FailOnBeginSource::new("a", false);
    let b = FailOnBeginSource::new("b", true); // begin fails
    let c = FailOnBeginSource::new("c", false);
    ctrl.add_source(a);
    ctrl.add_source(b);
    ctrl.add_source(c);

    let err = ctrl.begin_exploration().unwrap_err();
    assert!(matches!(err, ExploreError::SourceError(_)));
    // controller 应该是 inactive
    assert!(!ctrl.is_active());
}

#[test]
fn end_when_inactive_is_noop_and_records_user_cancelled() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.end_exploration().unwrap();
    assert!(!ctrl.is_active());
    assert_eq!(
        ctrl.diagnostics().last_stop_reason,
        Some(nuwax_exploration::ExplorationStopReason::UserCancelled)
    );
}

#[test]
fn shutdown_clears_sources() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.add_source(FailOnBeginSource::new("a", false));
    ctrl.begin_exploration().unwrap();
    assert!(ctrl.is_active());
    ctrl.shutdown();
    assert!(!ctrl.is_active());
    assert_eq!(ctrl.source_count(), 0);
}
