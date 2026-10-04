//! Cooperative planning cancellation. This never mutates a source map or its evidence.
use std::{cell::RefCell, sync::{Arc, atomic::{AtomicU64, Ordering}}};

/// A planning request remains current only while the owner's epoch equals this value.
/// Advancing the owner's epoch cancels all older tokens without resetting a shared flag.
#[derive(Clone, Debug)]
pub struct PlanningCancellation {
    epoch: Arc<AtomicU64>,
    expected_epoch: u64,
}
impl PlanningCancellation {
    pub fn new(epoch: Arc<AtomicU64>, expected_epoch: u64) -> Self {
        Self { epoch, expected_epoch }
    }
    pub fn is_cancelled(&self) -> bool {
        self.epoch.load(Ordering::Acquire) != self.expected_epoch
    }
}
thread_local! {
    static SCOPES: RefCell<Vec<PlanningCancellation>> = const { RefCell::new(Vec::new()) };
}

/// Run synchronous planning with a cancellation token on this thread. Nested scopes
/// preserve the outer token, and RAII restores the previous scope even during unwind.
/// The scope does not propagate to spawned threads: install it inside the worker.
pub fn with_planning_cancellation<T>(token: &PlanningCancellation, operation: impl FnOnce() -> T) -> T {
    struct Reset(usize);
    impl Drop for Reset {
        fn drop(&mut self) { SCOPES.with(|scopes| scopes.borrow_mut().truncate(self.0)); }
    }
    let previous = SCOPES.with(|scopes| {
        let mut scopes = scopes.borrow_mut();
        let previous = scopes.len();
        scopes.push(token.clone());
        previous
    });
    let _reset = Reset(previous);
    operation()
}

/// Whether any current planning scope has been superseded.
pub fn planning_cancelled() -> bool {
    SCOPES.with(|scopes| scopes.borrow().iter().any(PlanningCancellation::is_cancelled))
}
pub(crate) fn checkpoint() -> Result<(), String> {
    #[cfg(test)]
    TEST_CANCEL.with(|state| {
        let mut state = state.borrow_mut();
        if let Some((remaining, epoch)) = state.as_mut() {
            if *remaining == 0 {
                epoch.fetch_add(1, Ordering::Release);
                *state = None;
            } else { *remaining -= 1; }
        }
    });
    if planning_cancelled() { Err("层析规划已取消".into()) } else { Ok(()) }
}

#[cfg(test)]
thread_local! {
    static TEST_CANCEL: RefCell<Option<(usize, Arc<AtomicU64>)>> = const { RefCell::new(None) };
}
#[cfg(test)]
pub(crate) fn after_checkpoints<T>(count: usize, operation: impl FnOnce() -> T) -> T {
    struct Reset(Option<(usize, Arc<AtomicU64>)>);
    impl Drop for Reset {
        fn drop(&mut self) { TEST_CANCEL.with(|state| *state.borrow_mut() = self.0.take()); }
    }
    let epoch = Arc::new(AtomicU64::new(0));
    let token = PlanningCancellation::new(epoch.clone(), 0);
    let _reset = Reset(TEST_CANCEL.with(|state| state.replace(Some((count, epoch)))));
    with_planning_cancellation(&token, operation)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cancellation_scopes_restore_after_nested_calls_and_unwind() {
        let epoch = Arc::new(AtomicU64::new(3));
        let old = PlanningCancellation::new(epoch.clone(), 3);
        let current = PlanningCancellation::new(epoch.clone(), 4);
        assert!(!planning_cancelled());
        with_planning_cancellation(&old, || {
            assert!(!planning_cancelled());
            epoch.store(4, Ordering::Release);
            assert!(old.is_cancelled());
            with_planning_cancellation(&current, || assert!(planning_cancelled()));
            assert!(planning_cancelled());
        });
        assert!(!planning_cancelled());
        let result = std::panic::catch_unwind(|| with_planning_cancellation(&old, || {
            assert!(planning_cancelled());
            panic!("exercise RAII restoration");
        }));
        assert!(result.is_err());
        assert!(!planning_cancelled());
        with_planning_cancellation(&current, || assert!(!planning_cancelled()));
    }
}
