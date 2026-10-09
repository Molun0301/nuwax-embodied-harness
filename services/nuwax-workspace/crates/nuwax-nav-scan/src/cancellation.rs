use crate::{LocalPlanError, LocalPlanFailureKind};

pub(crate) fn checkpoint() -> Result<(), LocalPlanError> {
    #[cfg(test)]
    TEST_CANCEL.with(|state| {
        let mut state = state.borrow_mut();
        if let Some((remaining, epoch)) = state.as_mut() {
            if *remaining == 0 {
                epoch.fetch_add(1, std::sync::atomic::Ordering::Release);
                *state = None;
            } else {
                *remaining -= 1;
            }
        }
    });
    if crate::planning_cancelled() {
        Err(LocalPlanError { body_witness: None,
            kind: LocalPlanFailureKind::Cancelled,
            closest_pose: None,
            blocked: None,
            reason: "近场规划已取消".into(),
        })
    } else { Ok(()) }
}
#[cfg(test)]
thread_local! {
    static TEST_CANCEL: std::cell::RefCell<Option<(usize, std::sync::Arc<std::sync::atomic::AtomicU64>)>> = const { std::cell::RefCell::new(None) };
}
/// Cancel at an exact checkpoint, without timing races or a background thread.
#[cfg(test)]
pub(crate) fn after_checkpoints<T>(count: usize, operation: impl FnOnce() -> T) -> T {
    use std::sync::{Arc, atomic::AtomicU64};
    struct Reset(Option<(usize, Arc<AtomicU64>)>);
    impl Drop for Reset {
        fn drop(&mut self) { TEST_CANCEL.with(|state| *state.borrow_mut() = self.0.take()); }
    }
    let epoch = Arc::new(AtomicU64::new(0));
    let token = crate::PlanningCancellation::new(epoch.clone(), 0);
    let _reset = Reset(TEST_CANCEL.with(|state| state.replace(Some((count, epoch)))));
    crate::with_planning_cancellation(&token, operation)
}
