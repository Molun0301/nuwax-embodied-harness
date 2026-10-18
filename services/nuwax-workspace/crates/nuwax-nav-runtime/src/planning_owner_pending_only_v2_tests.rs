//! Pending-only refresh v2 owner contracts; draft, NOT BUILT OR RUN.
//! Adapted from the historical ten-test owner suite; historical 10 PASS is not
//! evidence for this version. Pauses are not performance measurements or
//! PlanKernel/source-ticket/admission proofs.
use super::pending_only_v2_test_hooks::{self, Phase};
use super::{Cancellation, PlanningOwner};
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    mpsc, Arc,
};
use std::time::{Duration, Instant};

fn wait(mut predicate: impl FnMut() -> bool) {
    let end = Instant::now() + Duration::from_secs(5);
    while !predicate() {
        assert!(Instant::now() < end, "owner condition timed out");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn recv<T>(rx: &mpsc::Receiver<T>) -> T {
    rx.recv_timeout(Duration::from_secs(5)).expect("owner event timed out")
}

fn owner_identity<T: Send + 'static, R: Send + 'static>(owner: &PlanningOwner<T, R>) -> usize {
    Arc::as_ptr(&owner.shared) as usize
}

// Declare after the owner: unwind releases all gates before owner teardown.
#[derive(Default)]
struct Cleanup {
    releases: Vec<mpsc::Sender<()>>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl Cleanup {
    fn release_all(&self) {
        for release in &self.releases {
            // Each phase gate is one-shot; each compute gate is used at most
            // twice in these fixtures. Extra permits also allow unwind cleanup.
            for _ in 0..3 {
                let _ = release.send(());
            }
        }
    }

    fn join(&mut self) {
        self.release_all();
        let mut failed = false;
        for handle in self.threads.drain(..) {
            // Join every helper before asserting, even if an earlier one failed.
            failed |= handle.join().is_err();
        }
        assert!(!failed, "owner test helper panicked");
    }
}

impl Drop for Cleanup {
    fn drop(&mut self) {
        self.release_all();
        for handle in self.threads.drain(..) {
            // Best-effort unwind cleanup, not a successful-join assertion.
            let _ = handle.join();
        }
    }
}

struct Payload {
    n: u64,
    on_drop: Option<Box<dyn FnOnce() + Send>>,
}

impl Payload {
    fn plain(n: u64) -> Self {
        Self { n, on_drop: None }
    }
}

impl Drop for Payload {
    fn drop(&mut self) {
        if let Some(on_drop) = self.on_drop.take() {
            on_drop();
        }
    }
}

#[test]
fn pending_replace_is_lazy_exact_and_drops_retired_payload_outside_lock() {
    let (entered, starts) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let owner = PlanningOwner::new(move |payload: Payload, _: &Cancellation| {
        entered.send(payload.n).unwrap();
        recv(&blocked);
        payload.n
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    assert!(owner.submit(1, Payload::plain(1)));
    assert_eq!(recv(&starts), 1);
    let epoch = owner.epoch();
    let dropped = Arc::new(AtomicUsize::new(0));
    let weak = Arc::downgrade(&owner.shared);
    let observed = dropped.clone();
    assert!(owner.submit(2, Payload {
        n: 2,
        on_drop: Some(Box::new(move || {
            // Worker is held outside exchange; no competing lock user.
            let outside_lock = weak.upgrade()
                .is_some_and(|shared| shared.exchange.try_lock().is_ok());
            // Report outside Drop, so cleanup cannot cause a second panic.
            observed.store(if outside_lock { 1 } else { 2 }, Ordering::SeqCst);
        })),
    }));
    let made = std::cell::Cell::new(0);
    for (id, expected_epoch) in [(99, epoch), (2, epoch + 1)] {
        assert_eq!(owner.replace_pending_with_retired(id, expected_epoch, 3, || {
            made.set(made.get() + 1);
            Payload::plain(3)
        }), None);
    }
    assert_eq!(made.get(), 0);
    assert_eq!(dropped.load(Ordering::SeqCst), 0);
    {
        let slot = owner.shared.exchange.lock().unwrap();
        assert!(slot.active);
        assert!(slot.completion.is_none());
        let pending = slot.pending.as_ref().unwrap();
        assert_eq!((pending.id, pending.epoch, pending.value.n), (2, epoch, 2));
    }
    assert_eq!(owner.replace_pending_with_retired(2, epoch, 3, || {
        made.set(made.get() + 1);
        Payload::plain(3)
    }), Some(2));
    assert_eq!(made.get(), 1);
    assert_eq!(dropped.load(Ordering::SeqCst), 1, "retired P was not dropped outside exchange");
    assert_eq!(owner.epoch(), epoch);
    {
        let slot = owner.shared.exchange.lock().unwrap();
        assert!(slot.active);
        assert!(slot.completion.is_none());
        let pending = slot.pending.as_ref().unwrap();
        assert_eq!((pending.id, pending.epoch, pending.value.n), (3, epoch, 3));
    }
    release.send(()).unwrap();
    wait(|| owner.completion_ready_for_test());
    assert!(starts.try_recv().is_err());
    // A completion supplies original backpressure; only P is refreshed.
    assert_eq!(owner.replace_pending_with_retired(3, epoch, 4, || Payload::plain(4)), Some(3));
    let completion = owner.take().unwrap();
    assert_eq!((completion.id, completion.epoch, completion.value), (1, epoch, 1));
    assert_eq!(recv(&starts), 4);
    assert_eq!(owner.replace_pending_with_retired(4, epoch, 5, || {
        made.set(made.get() + 1);
        Payload::plain(5)
    }), None, "already active is not pending");
    assert_eq!(made.get(), 1);
    release.send(()).unwrap();
    wait(|| owner.completion_ready_for_test());
    let completion = owner.take().unwrap();
    assert_eq!((completion.id, completion.epoch, completion.value), (4, epoch, 4));
    wait(|| owner.is_idle_for_test());
    assert_eq!(owner.replace_pending_with_retired(4, epoch, 5, || {
        made.set(made.get() + 1);
        Payload::plain(5)
    }), None, "an empty owner must not create a new pending slot");
    assert_eq!(made.get(), 1);
    assert!(owner.is_idle_for_test());
    assert!(starts.try_recv().is_err());
    assert!(!owner.faulted());
}

#[test]
fn pending_replace_miss_drops_unused_factory_capture_outside_lock() {
    let (entered, starts) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let owner = PlanningOwner::new(move |n: u64, _: &Cancellation| {
        entered.send(n).unwrap();
        recv(&blocked);
        n
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    assert!(owner.submit(1, 1));
    assert_eq!(recv(&starts), 1);
    assert!(owner.submit(2, 2));
    let epoch = owner.epoch();
    let weak = Arc::downgrade(&owner.shared);
    let dropped = Arc::new(AtomicUsize::new(0));
    let observed = dropped.clone();
    let captured = Payload {
        n: 0,
        on_drop: Some(Box::new(move || {
            let outside_lock = weak.upgrade()
                .is_some_and(|shared| shared.exchange.try_lock().is_ok());
            observed.store(if outside_lock { 1 } else { 2 }, Ordering::SeqCst);
        })),
    };
    assert_eq!(owner.replace_pending_with_retired(99, epoch, 3, move || {
        drop(captured);
        panic!("miss must not invoke the factory")
    }), None);
    assert_eq!(dropped.load(Ordering::SeqCst), 1, "unused factory capture was not dropped outside exchange");
    assert_eq!(owner.epoch(), epoch);
    {
        let slot = owner.shared.exchange.lock().unwrap();
        assert!(slot.active);
        assert!(slot.completion.is_none());
        let pending = slot.pending.as_ref().unwrap();
        assert_eq!((pending.id, pending.epoch, pending.value), (2, epoch, 2));
    }
    owner.cancel();
    release.send(()).unwrap();
    wait(|| owner.is_idle_for_test());
    assert!(starts.try_recv().is_err());
    assert!(!owner.faulted());
}

#[test]
fn pending_replace_closed_then_exited_owner_does_not_invoke_factory() {
    let (entered, starts) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let owner = PlanningOwner::new(move |n: u64, _: &Cancellation| {
        entered.send(n).unwrap();
        recv(&blocked);
        n
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    assert!(owner.submit(1, 1));
    assert_eq!(recv(&starts), 1);
    assert!(owner.submit(2, 2));
    let epoch = owner.epoch();
    // Inject existing teardown flag while A is held, so closed is checked
    // before faulted() can short-circuit on the later worker exit.
    owner.shared.exchange.lock().unwrap().closed = true;
    assert!(!owner.faulted());
    assert_eq!(owner.replace_pending_with_retired(2, epoch, 3,
        || panic!("closed owner invoked factory")), None);
    assert_eq!(owner.shared.exchange.lock().unwrap().pending.as_ref().unwrap().id, 2);
    release.send(()).unwrap();
    wait(|| !owner.shared.alive.load(Ordering::Acquire));
    assert!(owner.faulted());
    assert_eq!(owner.replace_pending_with_retired(2, epoch, 3,
        || panic!("exited owner invoked factory")), None);
    assert!(!owner.submit(4, 4));
    owner.cancel();
    assert!(owner.take().is_none());
    assert!(owner.is_idle_for_test());
    assert!(starts.try_recv().is_err());
}

#[test]
fn pending_replace_real_worker_fault_retires_pending_without_restart() {
    let (entered, starts) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let owner = PlanningOwner::<u64, u64>::new(move |n, _| {
        entered.send(n).unwrap();
        recv(&blocked);
        panic!("injected compute fault after pending replacement")
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    assert!(owner.submit(1, 1));
    assert_eq!(recv(&starts), 1);
    assert!(owner.submit(2, 2));
    let epoch = owner.epoch();
    assert_eq!(owner.replace_pending_with_retired(2, epoch, 3, || 3), Some(2));
    release.send(()).unwrap();
    wait(|| !owner.shared.alive.load(Ordering::Acquire));
    assert!(owner.shared.fault.load(Ordering::Acquire));
    assert!(owner.is_idle_for_test());
    assert_eq!(owner.replace_pending_with_retired(3, epoch, 4,
        || panic!("faulted owner invoked factory")), None);
    assert!(!owner.submit(4, 4));
    assert!(owner.take().is_none());
    assert!(starts.try_recv().is_err());
}

#[test]
fn pending_replace_poison_is_recovered_as_fault_without_invoking_factory() {
    let (entered, starts) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let owner = PlanningOwner::new(move |n: u64, _: &Cancellation| {
        entered.send(n).unwrap();
        recv(&blocked);
        n
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    assert!(owner.submit(1, 1));
    assert_eq!(recv(&starts), 1);
    assert!(owner.submit(2, 2));
    let epoch = owner.epoch();
    let poisoned = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _guard = owner.shared.exchange.lock().unwrap();
        panic!("injected exchange poison while worker is outside lock");
    }));
    assert!(poisoned.is_err());
    assert!(!owner.faulted(), "replace must exercise its poisoned-lock recovery");
    assert_eq!(owner.replace_pending_with_retired(2, epoch, 3,
        || panic!("poison recovery invoked factory")), None);
    assert!(owner.shared.fault.load(Ordering::Acquire));
    {
        let slot = owner.shared.exchange.lock().unwrap_or_else(|error| error.into_inner());
        let pending = slot.pending.as_ref().unwrap();
        assert_eq!((pending.id, pending.epoch, pending.value), (2, epoch, 2));
        assert!(slot.active);
        assert!(slot.completion.is_none());
    }
    owner.cancel();
    release.send(()).unwrap();
    // The original worker catches its later poisoned-lock panic and exits.
    // Do not use helpers that unwrap this deliberately poisoned mutex.
    wait(|| !owner.shared.alive.load(Ordering::Acquire));
    assert!(owner.take().is_none());
    assert!(!owner.submit(4, 4));
    assert!(starts.try_recv().is_err());
}

#[test]
fn pending_replace_rejects_real_cancel_bumped_before_clear() {
    let (entered_compute, starts) = mpsc::channel();
    let owner = Arc::new(PlanningOwner::new(move |n: u64, _: &Cancellation| {
        entered_compute.send(n).unwrap();
        n
    }).unwrap());
    let mut cleanup = Cleanup::default();
    assert!(owner.submit(1, 1));
    assert_eq!(recv(&starts), 1);
    wait(|| owner.completion_ready_for_test());
    assert!(owner.submit(2, 2));
    let epoch = owner.epoch();
    let (entered, at_bump) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    cleanup.releases.push(release.clone());
    let cancel_owner = owner.clone();
    cleanup.threads.push(std::thread::spawn(move || {
        let _reset = pending_only_v2_test_hooks::install(
            owner_identity(&cancel_owner), Phase::CancelBumped, entered, blocked,
        );
        cancel_owner.cancel();
    }));
    recv(&at_bump);
    assert_eq!(owner.epoch(), epoch + 1);
    // e matches pending but not current; e+1 matches current but not pending.
    // These two calls independently exercise both epoch comparisons.
    for expected_epoch in [epoch, epoch + 1] {
        assert_eq!(owner.replace_pending_with_retired(2, expected_epoch, 3,
            || panic!("mismatching epoch invoked factory")), None);
    }
    {
        let slot = owner.shared.exchange.lock().unwrap();
        let pending = slot.pending.as_ref().unwrap();
        assert_eq!((pending.id, pending.epoch), (2, epoch));
        assert_eq!(slot.completion.as_ref().unwrap().id, 1);
    }
    cleanup.join();
    wait(|| owner.is_idle_for_test());
    assert!(owner.take().is_none());
    assert!(starts.try_recv().is_err());
    assert!(!owner.faulted());
}

#[derive(Clone, Copy)]
enum CancelWinner {
    CancelClears,
    WorkerDequeues,
}

fn checked_replace_racing_real_cancel(winner: CancelWinner) {
    let (entered_compute, starts) = mpsc::channel();
    let owner = Arc::new(PlanningOwner::new(move |payload: Payload, _: &Cancellation| {
        entered_compute.send((payload.n, std::thread::current().id())).unwrap();
        payload.n
    }).unwrap());
    let mut cleanup = Cleanup::default();
    assert!(owner.submit(1, Payload::plain(1)));
    let (started_id, worker_thread) = recv(&starts);
    assert_eq!(started_id, 1);
    wait(|| owner.completion_ready_for_test());
    assert!(owner.submit(2, Payload::plain(2)));
    let epoch = owner.epoch();
    let (replace_entered, at_replace) = mpsc::channel();
    let (replace_release, replace_blocked) = mpsc::channel();
    let (cancel_entered, at_cancel) = mpsc::channel();
    let (cancel_release, cancel_blocked) = mpsc::channel();
    cleanup.releases.extend([replace_release.clone(), cancel_release.clone()]);
    let made = Arc::new(AtomicUsize::new(0));
    let factory_count = made.clone();
    let (retired_payload, payload_retired) = mpsc::channel();
    let (receipt, receipts) = mpsc::channel();
    let replace_owner = owner.clone();
    cleanup.threads.push(std::thread::spawn(move || {
        let _reset = pending_only_v2_test_hooks::install(
            owner_identity(&replace_owner), Phase::ReplaceChecked,
            replace_entered, replace_blocked,
        );
        let result = replace_owner.replace_pending_with_retired(2, epoch, 3, || {
            factory_count.fetch_add(1, Ordering::SeqCst);
            Payload {
                n: 3,
                on_drop: Some(Box::new(move || {
                    // Per-payload witness, not a worker hook/counter: identify
                    // which real path removed this replacement payload.
                    let _ = retired_payload.send(std::thread::current().id());
                })),
            }
        });
        receipt.send(result).unwrap();
    }));
    recv(&at_replace); // Original exchange held: do not inspect it here.
    let cancel_owner = owner.clone();
    let cancel_handle = std::thread::spawn(move || {
        let _reset = pending_only_v2_test_hooks::install(
            owner_identity(&cancel_owner), Phase::CancelBumped,
            cancel_entered, cancel_blocked,
        );
        cancel_owner.cancel();
    });
    let cancel_thread = cancel_handle.thread().id();
    cleanup.threads.push(cancel_handle);
    recv(&at_cancel);
    assert_eq!(owner.epoch(), epoch + 1);
    replace_release.send(()).unwrap();
    assert_eq!(recv(&receipts), Some(2));
    assert_eq!(made.load(Ordering::SeqCst), 1);
    {
        let slot = owner.shared.exchange.lock().unwrap();
        let pending = slot.pending.as_ref().unwrap();
        assert_eq!((pending.id, pending.epoch, pending.value.n), (3, epoch, 3));
        let completion = slot.completion.as_ref().unwrap();
        assert_eq!((completion.id, completion.epoch), (1, epoch));
    }
    match winner {
        CancelWinner::CancelClears => {
            // Keep A completion: original backpressure forbids worker taking P.
            // Original cancel must clear both slots before worker can proceed.
            cleanup.join();
            assert_eq!(recv(&payload_retired), cancel_thread);
        }
        CancelWinner::WorkerDequeues => {
            let completion = owner.take().unwrap();
            assert_eq!((completion.id, completion.epoch), (1, epoch));
            // Cancel is still paused and cannot clear P. Only the original
            // worker can take/drop it, with no second compute entry.
            assert_eq!(recv(&payload_retired), worker_thread);
            wait(|| owner.is_idle_for_test());
            assert!(starts.try_recv().is_err());
            cleanup.join();
        }
    }
    assert!(owner.take().is_none());
    assert!(owner.is_idle_for_test());
    assert!(starts.try_recv().is_err());
    assert!(!owner.faulted());
}

#[test]
fn pending_replace_checked_before_cancel_keeps_old_epoch_cancel_clears_first() {
    checked_replace_racing_real_cancel(CancelWinner::CancelClears);
}

#[test]
fn pending_replace_checked_before_cancel_keeps_old_epoch_worker_dequeues_first() {
    checked_replace_racing_real_cancel(CancelWinner::WorkerDequeues);
}

#[test]
fn pending_replace_then_started_job_is_retired_by_original_cancel() {
    let (entered, starts) = mpsc::channel();
    let (finished, finishes) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let owner = PlanningOwner::new(move |n: u64, cancel: &Cancellation| {
        entered.send((n, cancel.expected, cancel.cancelled())).unwrap();
        recv(&blocked);
        finished.send((n, cancel.cancelled())).unwrap();
        n
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    let epoch = owner.epoch();
    assert!(owner.submit(1, 1));
    assert_eq!(recv(&starts), (1, epoch, false));
    assert!(owner.submit(2, 2));
    assert_eq!(owner.replace_pending_with_retired(2, epoch, 3, || 3), Some(2));
    release.send(()).unwrap();
    assert_eq!(recv(&finishes), (1, false));
    wait(|| owner.completion_ready_for_test());
    assert_eq!(owner.take().unwrap().id, 1);
    assert_eq!(recv(&starts), (3, epoch, false));
    owner.cancel();
    assert_eq!(owner.epoch(), epoch + 1);
    release.send(()).unwrap();
    assert_eq!(recv(&finishes), (3, true));
    wait(|| owner.is_idle_for_test());
    assert!(owner.take().is_none());
    assert!(starts.try_recv().is_err());
    assert!(!owner.faulted());
}

#[test]
fn pending_replace_owner_drop_retires_slots_and_returns_after_worker_lifetime() {
    let (entered, starts) = mpsc::channel();
    let (release, blocked) = mpsc::channel();
    let (worker_ended, worker_end) = mpsc::channel();
    let compute_lifetime = Payload {
        n: 0,
        on_drop: Some(Box::new(move || {
            let _ = worker_ended.send(std::thread::current().id());
        })),
    };
    let owner = PlanningOwner::new(move |n: u64, _: &Cancellation| {
        // Keep this witness owned by the compute closure until worker exit.
        let _keep_alive = &compute_lifetime;
        entered.send((n, std::thread::current().id())).unwrap();
        recv(&blocked);
        n
    }).unwrap();
    let mut cleanup = Cleanup::default();
    cleanup.releases.push(release.clone());
    assert!(owner.submit(1, 1));
    let (started_id, worker_thread) = recv(&starts);
    assert_eq!(started_id, 1);
    assert!(owner.submit(2, 2));
    assert_eq!(owner.replace_pending_with_retired(2, owner.epoch(), 3, || 3), Some(2));
    let shared = owner.shared.clone();
    let (cancel_entered, at_cancel) = mpsc::channel();
    let (cancel_release, cancel_blocked) = mpsc::channel();
    cleanup.releases.push(cancel_release.clone());
    let (drop_returned, drop_returns) = mpsc::channel();
    cleanup.threads.push(std::thread::spawn(move || {
        let _reset = pending_only_v2_test_hooks::install(
            owner_identity(&owner), Phase::CancelBumped, cancel_entered, cancel_blocked,
        );
        drop(owner);
        drop_returned.send(()).unwrap();
    }));
    recv(&at_cancel);
    cancel_release.send(()).unwrap();
    // Wait for original Drop to close exchange, with A still inside compute.
    wait(|| shared.exchange.lock().unwrap().closed);
    {
        let slot = shared.exchange.lock().unwrap();
        assert!(slot.active);
        assert!(slot.pending.is_none());
        assert!(slot.completion.is_none());
    }
    assert!(matches!(drop_returns.try_recv(), Err(mpsc::TryRecvError::Empty)));
    assert!(matches!(worker_end.try_recv(), Err(mpsc::TryRecvError::Empty)));
    cleanup.join(); // First release A, then join the test's drop-caller thread.
    recv(&drop_returns);
    assert_eq!(recv(&worker_end), worker_thread);
    assert!(!shared.alive.load(Ordering::Acquire));
    assert!(!shared.fault.load(Ordering::Acquire));
    assert!(starts.try_recv().is_err());
    // These witnesses establish teardown/worker lifetime. PlanningOwner::Drop
    // discards its private worker JoinHandle result, so we do not claim to have
    // independently observed that result or instrumented its join count.
}
