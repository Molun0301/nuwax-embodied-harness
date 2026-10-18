//! Explicit-qualification-only, one immutable first-dequeue observation.
//! No production field, queue, lock, wait, or worker scheduling control.
use std::cell::RefCell;
use std::marker::PhantomData;
use std::sync::{Arc, atomic::{AtomicBool, AtomicU64, Ordering}};
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug)]
pub(crate) struct FirstDequeueObservation {
    pub id: u64,
    pub epoch: u64,
    pub before: Instant,
    pub after: Instant,
}
pub(crate) struct FirstDequeueProbe {
    origin: Instant,
    id: AtomicU64,
    epoch: AtomicU64,
    before_ns: AtomicU64,
    after_ns: AtomicU64,
    ready: AtomicBool,
}
impl FirstDequeueProbe {
    pub(crate) fn new() -> Arc<Self> {
        Arc::new(Self { origin: Instant::now(), id: AtomicU64::new(0), epoch: AtomicU64::new(0),
            before_ns: AtomicU64::new(0), after_ns: AtomicU64::new(0), ready: AtomicBool::new(false) })
    }
    // The original owner worker is the sole writer, exactly once. Readers do
    // not infer pending from ready=false: the record may be in progress.
    pub(super) fn record(&self, id: u64, epoch: u64, before: Instant, after: Instant) {
        self.id.store(id, Ordering::Relaxed);
        self.epoch.store(epoch, Ordering::Relaxed);
        self.before_ns.store(before.duration_since(self.origin).as_nanos() as u64, Ordering::Relaxed);
        self.after_ns.store(after.duration_since(self.origin).as_nanos() as u64, Ordering::Relaxed);
        self.ready.store(true, Ordering::Release);
    }
    pub(crate) fn snapshot(&self) -> Option<FirstDequeueObservation> {
        if !self.ready.load(Ordering::Acquire) { return None; }
        Some(FirstDequeueObservation { id: self.id.load(Ordering::Relaxed), epoch: self.epoch.load(Ordering::Relaxed),
            before: self.origin + Duration::from_nanos(self.before_ns.load(Ordering::Relaxed)),
            after: self.origin + Duration::from_nanos(self.after_ns.load(Ordering::Relaxed)) })
    }
}
thread_local! { static NEXT_OWNER: RefCell<Option<Arc<FirstDequeueProbe>>> = const { RefCell::new(None) }; }
pub(crate) struct Reset { _thread_bound: PhantomData<*mut ()> }
impl Drop for Reset {
    fn drop(&mut self) { NEXT_OWNER.with(|slot| *slot.borrow_mut() = None); }
}
pub(crate) fn install(probe: &Arc<FirstDequeueProbe>) -> Reset {
    NEXT_OWNER.with(|slot| {
        assert!(slot.borrow().is_none(), "one scoped qualification owner at a time");
        *slot.borrow_mut() = Some(Arc::clone(probe));
    });
    Reset { _thread_bound: PhantomData }
}
// Construction consumes the registration once; later owners are not observed.
pub(super) fn take_for_new_owner() -> Option<Arc<FirstDequeueProbe>> {
    NEXT_OWNER.with(|slot| slot.borrow_mut().take())
}
