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
    let action = TEST_CANCEL.with(|state| {
        let mut state = state.borrow_mut();
        if let Some((remaining, _)) = state.as_mut() {
            if *remaining == 0 {
                return state.take().map(|(_, action)| action);
            } else { *remaining -= 1; }
        }
        None
    });
    #[cfg(test)]
    if let Some(action) = action { action(); }
    if planning_cancelled() { Err("层析规划已取消".into()) } else { Ok(()) }
}

#[cfg(test)]
thread_local! {
    static TEST_CANCEL: RefCell<Option<(usize, Box<dyn FnOnce()>)>> = const { RefCell::new(None) };
}
#[cfg(test)]
pub(crate) fn after_checkpoint_action<T>(count: usize, action: impl FnOnce() + 'static,
    operation: impl FnOnce() -> T) -> T {
    struct Reset(Option<(usize, Box<dyn FnOnce()>)>);
    impl Drop for Reset {
        fn drop(&mut self) { TEST_CANCEL.with(|state| *state.borrow_mut() = self.0.take()); }
    }
    let _reset = Reset(TEST_CANCEL.with(|state| state.replace(Some((count, Box::new(action))))));
    operation()
}
#[cfg(test)]
pub(crate) fn after_checkpoints<T>(count: usize, operation: impl FnOnce() -> T) -> T {
    let epoch = Arc::new(AtomicU64::new(0));
    let token = PlanningCancellation::new(epoch.clone(), 0);
    after_checkpoint_action(count, move || { epoch.fetch_add(1, Ordering::Release); }, ||
        with_planning_cancellation(&token, operation))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn checkpoint_actions_restore_after_nested_unwind_and_allow_reentry() {
        let calls = std::rc::Rc::new(RefCell::new(Vec::new()));
        let outer = calls.clone();
        after_checkpoint_action(1, move || outer.borrow_mut().push("outer"), || {
            checkpoint().unwrap();
            let inner = calls.clone();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(||
                after_checkpoint_action(0, move || {
                    checkpoint().unwrap();
                    inner.borrow_mut().push("inner");
                }, || { checkpoint().unwrap(); panic!("test countdown restoration"); })));
            assert!(result.is_err());
            checkpoint().unwrap();
        });
        assert_eq!(*calls.borrow(), ["inner", "outer"]);
        checkpoint().unwrap();
    }
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
