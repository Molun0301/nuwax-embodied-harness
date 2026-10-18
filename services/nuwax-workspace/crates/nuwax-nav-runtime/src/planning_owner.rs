//! One reusable CPU owner. All payload destruction and computation occurs outside
//! the exchange lock. Slots are bounded: active + latest pending + completion.
use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    Arc, Condvar, Mutex,
};
use std::thread::JoinHandle;

#[cfg(test)]
#[path = "planning_owner_pending_only_v2_test_hooks.rs"]
mod pending_only_v2_test_hooks;
#[cfg(test)]
#[path = "planning_owner_pending_only_v2_tests.rs"]
mod pending_only_v2_tests;

#[cfg(feature = "surface-recovery-qualification")]
#[path = "planning_owner_first_dequeue.rs"]
pub(crate) mod qualification_first_dequeue;

#[cfg(any(test, feature = "surface-recovery-qualification"))]
thread_local! { static PENDING_REPLACE_LOCK_ATTEMPTS_FOR_QUALIFICATION: std::cell::Cell<u64> = const { std::cell::Cell::new(0) }; }
#[cfg(any(test, feature = "surface-recovery-qualification"))]
pub(crate) fn pending_replace_lock_attempts_for_qualification() -> u64 {
    PENDING_REPLACE_LOCK_ATTEMPTS_FOR_QUALIFICATION.get()
}


#[cfg(test)]
thread_local! { static CURRENT_REQUEST_FOR_TEST: std::cell::Cell<Option<(u64,u64)>> = const { std::cell::Cell::new(None) }; }
#[cfg(test)]
pub(crate) fn current_request_for_test()->Option<(u64,u64)> { CURRENT_REQUEST_FOR_TEST.get() }

#[derive(Clone)]
pub(crate) struct Cancellation {
    pub epoch: Arc<AtomicU64>,
    pub expected: u64,
}
impl Cancellation {
    pub fn cancelled(&self) -> bool {
        self.epoch.load(Ordering::Acquire) != self.expected
    }
}
pub(crate) struct Completion<R> {
    pub id: u64,
    pub epoch: u64,
    pub value: R,
}
struct Request<T> {
    id: u64,
    epoch: u64,
    value: T,
}
struct Exchange<T, R> {
    pending: Option<Request<T>>,
    completion: Option<Completion<R>>,
    active: bool,
    closed: bool,
}
struct Shared<T, R> {
    exchange: Mutex<Exchange<T, R>>,
    wake: Condvar,
    epoch: Arc<AtomicU64>,
    fault: AtomicBool,
    alive: AtomicBool,
}
pub(crate) struct PlanningOwner<T: Send + 'static, R: Send + 'static> {
    shared: Arc<Shared<T, R>>,
    thread: Option<JoinHandle<()>>,
}
impl<T: Send + 'static, R: Send + 'static> PlanningOwner<T, R> {
    pub fn new(
        mut compute: impl FnMut(T, &Cancellation) -> R + Send + 'static,
    ) -> std::io::Result<Self> {
        let shared = Arc::new(Shared {
            exchange: Mutex::new(Exchange {
                pending: None,
                completion: None,
                active: false,
                closed: false,
            }),
            wake: Condvar::new(),
            epoch: Arc::new(AtomicU64::new(1)),
            fault: AtomicBool::new(false),
            alive: AtomicBool::new(true),
        });
        let worker = shared.clone();
        #[cfg(feature = "surface-recovery-qualification")]
        let first_dequeue_probe = qualification_first_dequeue::take_for_new_owner();
        let thread = std::thread::Builder::new()
            .name("nav-local-planning".into())
            .spawn(move || {
                #[cfg(feature = "surface-recovery-qualification")]
                let mut first_dequeue_recorded = false;
                // Catch the entire loop, including an unexpected implementation panic.
                let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| loop {
                    #[cfg(feature = "surface-recovery-qualification")]
                    let mut dequeue_before = None;
                    let request = {
                        let mut slot = worker.exchange.lock().unwrap();
                        while !slot.closed && (slot.pending.is_none() || slot.completion.is_some())
                        {
                            slot = worker.wake.wait(slot).unwrap();
                        }
                        if slot.closed {
                            break;
                        }
                        slot.active = true;
                        #[cfg(feature = "surface-recovery-qualification")]
                        if !first_dequeue_recorded && first_dequeue_probe.is_some() {
                            dequeue_before = Some(std::time::Instant::now());
                        }
                        slot.pending.take().unwrap()
                    };
                    #[cfg(feature = "surface-recovery-qualification")]
                    if let (Some(probe), Some(before)) = (&first_dequeue_probe, dequeue_before) {
                        let after = std::time::Instant::now();
                        probe.record(request.id, request.epoch, before, after);
                        first_dequeue_recorded = true;
                    }
                    let cancel = Cancellation {
                        epoch: worker.epoch.clone(),
                        expected: request.epoch,
                    };
                    #[cfg(test)] CURRENT_REQUEST_FOR_TEST.set(Some((request.id,request.epoch)));
                    let result = if cancel.cancelled() {
                        None
                    } else {
                        Some(compute(request.value, &cancel))
                    };
                    #[cfg(test)] CURRENT_REQUEST_FOR_TEST.set(None);
                    let mut result = result.map(|value| Completion {
                        id: request.id,
                        epoch: request.epoch,
                        value,
                    });
                    {
                        let mut slot = worker.exchange.lock().unwrap();
                        slot.active = false;
                        if !slot.closed && !cancel.cancelled() {
                            slot.completion = result.take();
                        }
                    }
                    drop(result);
                    worker.wake.notify_all();
                }));
                if outcome.is_err() {
                    worker.fault.store(true, Ordering::Release);
                    let retired = {
                        let mut slot = worker.exchange.lock().unwrap_or_else(|e| e.into_inner());
                        slot.active = false;
                        (slot.pending.take(), slot.completion.take())
                    };
                    drop(retired);
                }
                worker.alive.store(false, Ordering::Release);
                worker.wake.notify_all();
            })?;
        Ok(Self {
            shared,
            thread: Some(thread),
        })
    }
    #[cfg(test)]
    pub fn is_idle_for_test(&self) -> bool {
        let s = self.shared.exchange.lock().unwrap();
        !s.active && s.pending.is_none() && s.completion.is_none()
    }
    #[cfg(test)]
    pub fn completion_ready_for_test(&self) -> bool {
        self.shared.exchange.lock().unwrap().completion.is_some()
    }
    pub fn epoch(&self) -> u64 {
        self.shared.epoch.load(Ordering::Acquire)
    }
    pub fn faulted(&self) -> bool {
        self.shared.fault.load(Ordering::Acquire) || !self.shared.alive.load(Ordering::Acquire)
    }
    pub fn submit(&self, id: u64, value: T) -> bool {
        self.submit_with_retired(id,value).is_some()
    }
    /// Return the replaced pending ID so its source proof follows the same slot.
    /// The active/completed slot is never retired by a newer pending submission.
    pub fn submit_with_retired(&self, id:u64, value:T)->Option<Option<u64>> {
        if self.faulted() {
            return None;
        }
        let old;
        {
            let mut slot = self.shared.exchange.lock().unwrap_or_else(|e| {
                self.shared.fault.store(true, Ordering::Release);
                e.into_inner()
            });
            if slot.closed || self.shared.fault.load(Ordering::Acquire) {
                return None;
            }
            old = slot.pending.replace(Request {
                id,
                epoch: self.epoch(),
                value,
            });
        }
        let retired=old.as_ref().map(|request|request.id);
        drop(old);
        self.shared.wake.notify_one();
        Some(retired)
    }
    /// Refresh only the expected queued request; never create another slot.
    /// The factory is called under the existing lock only after an exact match.
    /// It must only copy the small request fields/Arcs, without owner reentry,
    /// geometry work or owned captures that need heavyweight destruction.
    pub(crate) fn replace_pending_with_retired(
        &self,
        expected_id: u64,
        expected_epoch: u64,
        id: u64,
        make_request: impl FnOnce() -> T,
    ) -> Option<u64> {
        if self.faulted() {
            return None;
        }
        #[cfg(any(test, feature = "surface-recovery-qualification"))]
        PENDING_REPLACE_LOCK_ATTEMPTS_FOR_QUALIFICATION.with(|count| count.set(count.get() + 1));
        let old;
        {
            let mut slot = self.shared.exchange.lock().unwrap_or_else(|e| {
                self.shared.fault.store(true, Ordering::Release);
                e.into_inner()
            });
            if slot.closed || self.shared.fault.load(Ordering::Acquire)
                || !slot.pending.as_ref().is_some_and(|pending|
                    pending.id == expected_id && pending.epoch == expected_epoch)
                || self.epoch() != expected_epoch
            {
                return None;
            }
            #[cfg(test)]
            pending_only_v2_test_hooks::run(
                Arc::as_ptr(&self.shared) as usize,
                pending_only_v2_test_hooks::Phase::ReplaceChecked,
            );
            // cancel bumps the epoch before locking this exchange. Retain the
            // checked epoch even if cancellation races with this small factory.
            old = slot.pending.replace(Request {
                id,
                epoch: expected_epoch,
                value: make_request(),
            });
        }
        let retired = old.as_ref().map(|request| request.id);
        drop(old);
        self.shared.wake.notify_one();
        retired
    }
    // Qualification-only Q case observation. Called once after the S2 update
    // returns and before the next original step/take, identically in A and B.
    // This reports a locked queued-state observation, NOT the replace point.
    // Caller brackets the complete call with Instant::now outside the lock.
    #[cfg(feature = "surface-recovery-qualification")]
    pub(crate) fn pending_only_q_state_for_qualification(
        &self, expected_id: u64, expected_epoch: u64,
    ) -> Option<(bool, bool)> {
        let slot = self.shared.exchange.lock().ok()?;
        if slot.closed || self.faulted() || self.epoch() != expected_epoch
            || !slot.pending.as_ref().is_some_and(|pending|
                pending.id == expected_id && pending.epoch == expected_epoch)
        {
            return None;
        }
        Some((slot.active, slot.completion.is_some()))
    }

    pub fn take(&self) -> Option<Completion<R>> {
        let result = self
            .shared
            .exchange
            .lock()
            .unwrap_or_else(|e| {
                self.shared.fault.store(true, Ordering::Release);
                e.into_inner()
            })
            .completion
            .take();
        self.shared.wake.notify_one();
        result
    }
    /// Hot cancellation never joins. Epoch exhaustion fails permanently closed.
    pub fn cancel(&self) {
        if self
            .shared
            .epoch
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |v| v.checked_add(1))
            .is_err()
        {
            self.shared.fault.store(true, Ordering::Release);
        }
        #[cfg(test)]
        pending_only_v2_test_hooks::run(
            Arc::as_ptr(&self.shared) as usize,
            pending_only_v2_test_hooks::Phase::CancelBumped,
        );
        let retired = {
            let mut slot = self.shared.exchange.lock().unwrap_or_else(|e| {
                self.shared.fault.store(true, Ordering::Release);
                e.into_inner()
            });
            (slot.pending.take(), slot.completion.take())
        };
        drop(retired);
        self.shared.wake.notify_one();
    }
}
impl<T: Send + 'static, R: Send + 'static> Drop for PlanningOwner<T, R> {
    fn drop(&mut self) {
        self.cancel();
        self.shared
            .exchange
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .closed = true;
        self.shared.wake.notify_one();
        // Exactly one join, outside all locks. Individual geometry/body calls can
        // delay teardown; no detached replacement or per-request thread exists.
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::{Duration, Instant};
    fn wait(mut condition: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(3);
        while !condition() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    #[test]
    fn one_active_latest_pending_and_one_completion() {
        let (started, starts) = mpsc::channel();
        let (release, blocked) = mpsc::channel();
        let owner = PlanningOwner::new(move |n, _: &Cancellation| {
            started.send(n).unwrap();
            blocked.recv().unwrap();
            n
        })
        .unwrap();
        owner.submit(1, 1);
        assert_eq!(starts.recv_timeout(Duration::from_secs(1)).unwrap(), 1);
        owner.submit(2, 2);
        owner.submit(3, 3);
        release.send(()).unwrap();
        wait(|| owner.shared.exchange.lock().unwrap().completion.is_some());
        assert!(
            starts.try_recv().is_err(),
            "completion provides bounded backpressure"
        );
        assert_eq!(owner.take().unwrap().value, 1);
        assert_eq!(starts.recv_timeout(Duration::from_secs(1)).unwrap(), 3);
        release.send(()).unwrap();
        wait(|| owner.shared.exchange.lock().unwrap().completion.is_some());
        assert_eq!(owner.take().unwrap().value, 3);
    }
    #[test]
    fn cancellation_retires_active_and_pending_without_joining() {
        let (started, starts) = mpsc::channel();
        let (release, blocked) = mpsc::channel();
        let owner = PlanningOwner::new(move |n, _: &Cancellation| {
            started.send(n).unwrap();
            blocked.recv().unwrap();
            n
        })
        .unwrap();
        owner.submit(1, 1);
        starts.recv().unwrap();
        owner.submit(2, 2);
        owner.cancel();
        owner.submit(3, 3);
        assert!(owner.take().is_none());
        release.send(()).unwrap();
        assert_eq!(starts.recv_timeout(Duration::from_secs(1)).unwrap(), 3);
        release.send(()).unwrap();
        wait(|| owner.shared.exchange.lock().unwrap().completion.is_some());
        assert_eq!(owner.take().unwrap().value, 3);
    }
    #[test]
    fn panic_is_visible_and_no_restart_storm() {
        let owner = PlanningOwner::<(), ()>::new(|_, _| panic!("injected planner fault")).unwrap();
        owner.submit(1, ());
        wait(|| owner.faulted());
        assert!(!owner.submit(2, ()));
        assert!(owner.take().is_none());
    }
    #[test]
    fn poison_and_unexpected_exit_are_visible_without_silent_pending() {
        let owner=PlanningOwner::<(),()>::new(|_,_|()).unwrap();
        let shared=owner.shared.clone();
        let _=std::thread::spawn(move|| {let _guard=shared.exchange.lock().unwrap();panic!("injected exchange poison");}).join();
        assert!(owner.take().is_none());assert!(owner.faulted());assert!(!owner.submit(1,()));
        drop(owner);
        let owner=PlanningOwner::<(),()>::new(|_,_|()).unwrap();
        owner.shared.exchange.lock().unwrap().closed=true;owner.shared.wake.notify_one();
        wait(||owner.faulted());assert!(!owner.submit(1,()));
    }

}
