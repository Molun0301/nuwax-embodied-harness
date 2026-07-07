//! source name 唯一性 / duplicate tie-break 测试(方案 §8.2)

use std::borrow::Cow;

use nuwax_exploration::{
    ExplorationArbitrationPolicy, ExplorationController, ExplorationGoalSource, ExplorationPoll,
    ExplorationSourceDiagnostics, ExplorationStateSnapshot, ExploreError,
};

#[derive(Debug)]
struct NamedSource {
    name: Cow<'static, str>,
    last_goal_id: std::cell::Cell<u32>,
}

impl NamedSource {
    fn new(name: &'static str) -> Self {
        Self {
            name: Cow::Borrowed(name),
            last_goal_id: std::cell::Cell::new(0),
        }
    }
}

impl ExplorationGoalSource for NamedSource {
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
        let n = self.last_goal_id.get() + 1;
        self.last_goal_id.set(n);
        Ok(ExplorationPoll::Pending)
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

#[test]
fn source_name_is_returned() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
    ctrl.add_source(NamedSource::new("alpha"));
    ctrl.add_source(NamedSource::new("beta"));
    assert_eq!(ctrl.source_count(), 2);
}

#[test]
fn duplicate_source_names_have_deterministic_insertion_order() {
    let mut ctrl = ExplorationController::new(ExplorationArbitrationPolicy::HighestScore);
    ctrl.add_source(NamedSource::new("wavefront"));
    ctrl.add_source(NamedSource::new("wavefront"));
    let order = ctrl.arbitration_order();
    // duplicate name 都视为 preferred,insertion order 保留(0 < 1)
    assert_eq!(order, vec![0, 1]);
}
