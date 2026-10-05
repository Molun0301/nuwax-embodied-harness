//! Opt-in, test-only synchronous exploration stage attribution.
//! Records are bounded; payload counts describe explicit copied cell payload only.
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc, Mutex,
};
use std::time::Instant;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub(super) enum Stage {
    RuntimeGridClone = 1,
    LockedSelection = 2,
    LockedRoutePreparation = 3,
    UnlockedRouteQueries = 4,
}
#[derive(Clone, Debug)]
pub(super) struct Record {
    pub stage: Stage,
    pub elapsed_ms: f64,
    pub copied_cell_bytes: usize,
    pub started: Instant,
    pub finished: Instant,
}
pub(super) struct Attribution {
    pub active_stage: AtomicUsize,
    pub entered: AtomicUsize,
    records: Mutex<Vec<Record>>,
    omitted: AtomicUsize,
}
impl Attribution {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            active_stage: AtomicUsize::new(0),
            entered: AtomicUsize::new(0),
            records: Mutex::new(Vec::with_capacity(128)),
            omitted: AtomicUsize::new(0),
        })
    }
    pub fn begin(self: &Arc<Self>, stage: Stage, copied_cell_bytes: usize) -> Span {
        let started = Instant::now();
        self.active_stage.store(stage as usize, Ordering::Release);
        self.entered.fetch_add(1, Ordering::Release);
        Span {
            owner: self.clone(),
            stage,
            copied_cell_bytes,
            started,
        }
    }
    pub fn snapshot(&self) -> (Vec<Record>, usize) {
        (
            self.records.lock().unwrap().clone(),
            self.omitted.load(Ordering::Relaxed),
        )
    }
}
pub(super) struct Span {
    owner: Arc<Attribution>,
    stage: Stage,
    copied_cell_bytes: usize,
    started: Instant,
}
impl Drop for Span {
    fn drop(&mut self) {
        let finished = Instant::now();
        let elapsed_ms = finished.duration_since(self.started).as_secs_f64() * 1000.;
        self.owner.active_stage.store(0, Ordering::Release);
        let mut records = self.owner.records.lock().unwrap();
        if records.len() < 128 {
            records.push(Record {
                stage: self.stage,
                elapsed_ms,
                copied_cell_bytes: self.copied_cell_bytes,
                started: self.started,
                finished,
            });
        } else {
            self.owner.omitted.fetch_add(1, Ordering::Relaxed);
        }
    }
}
#[test]
fn bounded_records_and_stage_lifetime() {
    let profile = Attribution::new();
    {
        let _span = profile.begin(Stage::LockedSelection, 250_000);
        assert_eq!(
            profile.active_stage.load(Ordering::Acquire),
            Stage::LockedSelection as usize
        );
    }
    assert_eq!(profile.active_stage.load(Ordering::Acquire), 0);
    for _ in 0..130 {
        drop(profile.begin(Stage::RuntimeGridClone, 0));
    }
    let (records, omitted) = profile.snapshot();
    assert_eq!(records.len(), 128);
    assert_eq!(omitted, 3);
    assert_eq!(profile.entered.load(Ordering::Acquire), 131);
    assert_eq!(records[0].copied_cell_bytes, 250_000);
    assert!(records
        .iter()
        .all(|r| r.finished >= r.started && r.elapsed_ms >= 0.));
}
