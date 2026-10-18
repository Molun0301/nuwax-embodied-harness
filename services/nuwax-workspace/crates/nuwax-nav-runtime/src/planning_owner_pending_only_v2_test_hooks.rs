//! Pending-only refresh v2 unit-test-only pauses; draft, NOT BUILT OR RUN.
//! Pauses in the original cancel method and the narrow replacement API.
//! Two fixed phases, one calling thread, one existing Shared Arc identity.
//! No callback registry, worker replacement, counters or production fields.
use std::cell::RefCell;
use std::marker::PhantomData;
use std::sync::mpsc::{Receiver, Sender};
use std::time::Duration;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Phase {
    CancelBumped,
    ReplaceChecked,
}

struct Pause {
    owner_identity: usize,
    phase: Phase,
    entered: Sender<()>,
    release: Receiver<()>,
}

thread_local! {
    static PAUSE: RefCell<Option<Pause>> = const { RefCell::new(None) };
}

// Reset must remain on its installing thread: it cleans up that thread's TLS.
pub(super) struct Reset {
    _thread_bound: PhantomData<*mut ()>,
}

impl Drop for Reset {
    fn drop(&mut self) {
        PAUSE.with(|pause| *pause.borrow_mut() = None);
    }
}

pub(super) fn install(
    owner_identity: usize,
    phase: Phase,
    entered: Sender<()>,
    release: Receiver<()>,
) -> Reset {
    PAUSE.with(|pause| {
        let mut pause = pause.borrow_mut();
        assert!(pause.is_none(), "only one owner pause per calling thread");
        *pause = Some(Pause { owner_identity, phase, entered, release });
    });
    Reset { _thread_bound: PhantomData }
}

pub(super) fn run(owner_identity: usize, phase: Phase) {
    let selected = PAUSE.with(|pause| {
        let mut pause = pause.borrow_mut();
        if pause.as_ref().is_some_and(|pause| {
            pause.owner_identity == owner_identity && pause.phase == phase
        }) {
            pause.take()
        } else {
            None
        }
    });
    // Consume exactly once and release the TLS borrow before waiting. A later
    // cancel during Drop cannot re-enter an already-used pause.
    if let Some(pause) = selected {
        pause.entered.send(()).expect("pause observer disappeared");
        pause.release.recv_timeout(Duration::from_secs(5))
            .expect("owner test pause was not released");
    }
}
