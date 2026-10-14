//! Cooperative planning cancellation. This never mutates a source map or its evidence.
use std::{cell::RefCell, sync::{Arc, atomic::{AtomicBool, AtomicU64, Ordering}}};

/// A request-owned cancellation source shared with synchronous planning checkpoints.
/// Epoch tokens retire on owner advancement; flag tokens use the existing job flag.
#[derive(Clone, Debug)]
pub struct PlanningCancellation {
    source: CancellationSource,
}
#[derive(Clone, Debug)]
enum CancellationSource {
    Epoch { epoch: Arc<AtomicU64>, expected_epoch: u64 },
    Flag(Arc<AtomicBool>),
}
impl PlanningCancellation {
    pub fn new(epoch: Arc<AtomicU64>, expected_epoch: u64) -> Self {
        Self { source: CancellationSource::Epoch { epoch, expected_epoch } }
    }
    /// Share the existing request-owned flag with construction checkpoints.
    pub fn from_flag(flag: Arc<AtomicBool>) -> Self {
        Self { source: CancellationSource::Flag(flag) }
    }
    pub fn is_cancelled(&self) -> bool {
        match &self.source {
            CancellationSource::Epoch { epoch, expected_epoch } => epoch.load(Ordering::Acquire) != *expected_epoch,
            CancellationSource::Flag(flag) => flag.load(Ordering::Acquire),
        }
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
    fn owned_flags_preserve_mixed_epoch_scopes_and_unwind() {
        let epoch = Arc::new(AtomicU64::new(7));
        let current = PlanningCancellation::new(epoch.clone(), 7);
        let flag = Arc::new(AtomicBool::new(false));
        let token = PlanningCancellation::from_flag(flag.clone());
        with_planning_cancellation(&current, || {
            assert!(!planning_cancelled());
            with_planning_cancellation(&token, || {
                assert!(!planning_cancelled());
                flag.store(true, Ordering::Release);
                assert!(planning_cancelled());
                with_planning_cancellation(&current, || assert!(planning_cancelled()));
            });
            assert!(!planning_cancelled(), "the retired flag must leave its outer epoch intact");
            let unwind = std::panic::catch_unwind(|| with_planning_cancellation(&token, || {
                assert!(planning_cancelled());
                panic!("exercise owned-flag restoration");
            }));
            assert!(unwind.is_err());
            assert!(!planning_cancelled());
            epoch.store(8, Ordering::Release);
            let fresh = PlanningCancellation::from_flag(Arc::new(AtomicBool::new(false)));
            with_planning_cancellation(&fresh, || assert!(planning_cancelled()));
        });
        assert!(!planning_cancelled());
        assert!(token.is_cancelled());
    }

    #[test]
    fn owned_flag_scope_is_thread_local_and_fresh_requests_are_independent() {
        let retired = PlanningCancellation::from_flag(Arc::new(AtomicBool::new(true)));
        let flag = Arc::new(AtomicBool::new(false));
        let current = PlanningCancellation::from_flag(flag.clone());
        let (ready, entered) = std::sync::mpsc::channel();
        let (release, resume) = std::sync::mpsc::channel();
        with_planning_cancellation(&retired, || {
            let worker = std::thread::spawn(move || {
                assert!(!planning_cancelled(), "parent TLS must not enter a spawned worker");
                with_planning_cancellation(&current, || {
                    assert!(!planning_cancelled(), "the new request has its own flag");
                    ready.send(()).unwrap();
                    resume.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
                    assert!(planning_cancelled(), "the worker must share the owner's actual flag");
                });
                assert!(!planning_cancelled());
            });
            entered.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
            assert!(planning_cancelled());
            flag.store(true, Ordering::Release);
            release.send(()).unwrap();
            worker.join().unwrap();
            assert!(planning_cancelled(), "worker restoration cannot change parent TLS");
        });
        let fresh = PlanningCancellation::from_flag(Arc::new(AtomicBool::new(false)));
        with_planning_cancellation(&fresh, || assert!(!planning_cancelled()));
        assert!(!planning_cancelled());
    }

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
