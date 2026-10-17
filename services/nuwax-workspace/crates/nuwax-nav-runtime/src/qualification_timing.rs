//! Explicit, bounded elapsed-time observations for qualification only.
//!
//! There is no global sink, worker, output, payload capture, or control decision.
//! Callers create a session and pass its sink explicitly. Stop producers before
//! `finish`; drops, open spans, or a busy final buffer make the report incomplete.
//! A span end means scope exit (including errors/cancellation), never success.
//! Timing includes scheduling, waiting and observation overhead; it is not CPU
//! time and must not be compared to an uninstrumented run as a performance result.

use std::sync::{Arc, Mutex, TryLockError};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Instant;

/// Fixed scalar copy of the scheduler's existing state; no freshness evaluation.
#[derive(Clone, Copy, Debug)]
pub struct SchedulerSnapshot {
    /// Existing lifecycle state.
    pub phase: crate::SchedulerPhase,
    /// Whether the scheduler owns an active lease.
    pub lease_active: bool,
    /// Whether the scheduler holds a session, without retaining its identifier.
    pub session_active: bool,
    /// Existing cached status bit, copied without recomputation.
    pub planner_inputs_fresh: bool,
    /// Existing operation sequence at observation.
    pub operation_sequence: u64,
    /// Accepted path revision.
    pub path_revision: u64,
    /// Existing receipt identities, absent if not admitted.
    pub pose_source_ns: Option<i64>,
    /// Existing map receipt identity, absent if not admitted.
    pub map_source_ns: Option<i64>,
    /// Read-only receipt ages measured at this snapshot.
    pub pose_receipt_age_ns: Option<u64>,
    /// Read-only map receipt age measured at this snapshot.
    pub map_receipt_age_ns: Option<u64>,
    /// Signed remaining pose deadline; negative means already elapsed.
    pub pose_deadline_remaining_ns: Option<i64>,
    /// Existing recent-source health flag, without recomputing freshness.
    pub recent_available: bool,
    /// Distinguishes no bound from an invalid bound with no monotonic deadline.
    pub recent_bound_present: bool,
    /// Original recent-bound Unix anchor, not a fabricated sensor timestamp.
    pub recent_anchor_unix_ns: Option<u64>,
    /// Original recent-bound latest admissible issue time.
    pub recent_latest_issue_unix_ns: Option<u64>,
    /// Original recent-bound monotonic deadline relative to this observation.
    pub recent_deadline_remaining_ns: Option<i64>,
    /// Last confirmed command represented only by three scalar velocities.
    pub last_applied: Option<[f64; 3]>,
    /// Existing control_tick staleness result; None means not evaluated here.
    pub stale: Option<bool>,
    /// Bits from the existing returned reason: pose=1, costmap=2, recent=4,
    /// other=8. This only classifies text already produced by normal control.
    pub stale_cause_bits: u8,
}

/// Fixed-size event data. Static labels must not contain user/runtime payloads.
#[derive(Clone, Copy, Debug)]
pub enum TraceData {
    /// A single observation.
    Mark,
    /// An existing scalar count; no iteration or decoding is implied.
    Count(u64),
    /// Actual requested/confirmed command scalars, not a retained payload.
    Motion([f64; 3]),
    /// Existing dispatch-bound scalar fields; absence/invalidity is preserved.
    Bound { present: bool, anchor_unix_ns: Option<u64>, latest_issue_unix_ns: Option<u64>, remaining_ns: Option<i64> },
    /// Scope entry and its session-local correlation id.
    SpanStart { id: u64 },
    /// Scope exit, whether successful, errored or cancelled.
    SpanEnd { id: u64, elapsed_ns: u64 },
    /// Read-only fixed scalar state.
    Scheduler(SchedulerSnapshot),
}

/// One bounded record; `source=0` explicitly means unavailable/not applicable.
#[derive(Clone, Copy, Debug)]
pub struct TraceRecord {
    /// Static phase label.
    pub phase: &'static str,
    /// Existing source identity, never a new timestamp used by navigation.
    pub source: i64,
    /// Elapsed monotonic nanoseconds since explicit session start.
    pub at_ns: u64,
    /// Opaque Rust thread identity at the synchronous recording point, not native tid.
    pub thread: std::thread::ThreadId,
    /// Fixed scalar observation.
    pub data: TraceData,
}

struct Shared {
    origin: Instant,
    enabled: AtomicBool,
    next_span: AtomicU64,
    open_spans: AtomicU64,
    contended: AtomicU64,
    overflow: AtomicU64,
    poisoned: AtomicU64,
    closed_race: AtomicU64,
    limit: usize,
    records: Mutex<Vec<TraceRecord>>,
}

/// Explicit recording owner. No global installation occurs.
pub struct TraceSession { sink: TraceSink }

/// Cheap explicit handle; recording never waits for the buffer lock.
#[derive(Clone)]
pub struct TraceSink { shared: Arc<Shared> }

/// Complete or explicitly incomplete observation report, printable after cleanup.
#[derive(Debug)]
pub struct TraceReport {
    /// Original std monotonic origin, for alignment with MotionEvent Instant.
    pub origin: Instant,
    /// Fixed record size, excluding Vec/Arc/mutex/allocator metadata.
    pub record_size_bytes: usize,
    /// size_of::<TraceRecord>() * capacity, a logical record-storage bound,
    /// not RSS, allocator overhead, or a claim about physical memory usage.
    pub record_capacity_bytes: usize,
    /// Records retained in insertion order; use at_ns for cross-thread ordering.
    pub records: Vec<TraceRecord>,
    /// Requested fixed record limit.
    pub capacity: usize,
    /// Records dropped because another producer held the session-local lock.
    pub dropped_contended: u64,
    /// Records dropped at the fixed capacity.
    pub dropped_overflow: u64,
    /// Records dropped after buffer poisoning.
    pub dropped_poisoned: u64,
    /// Recording attempts that raced final closure.
    pub dropped_closed_race: u64,
    /// Scopes still open when recording was stopped.
    pub open_spans_at_finish: u64,
    /// Finish refused to wait for a busy/poisoned buffer.
    pub finish_buffer_unavailable: bool,
    /// Any missing records or unfinished scopes make attribution incomplete.
    pub incomplete: bool,
}

impl TraceSession {
    /// Explicitly activate recording and allocate its entire capacity up front.
    pub fn start(capacity: usize) -> Self {
        Self { sink: TraceSink { shared: Arc::new(Shared {
            origin: Instant::now(), enabled: AtomicBool::new(true),
            next_span: AtomicU64::new(1), open_spans: AtomicU64::new(0),
            contended: AtomicU64::new(0), overflow: AtomicU64::new(0),
            poisoned: AtomicU64::new(0), closed_race: AtomicU64::new(0),
            limit: capacity, records: Mutex::new(Vec::with_capacity(capacity)),
        }) } }
    }
    /// Clone a handle for an explicitly instrumented actor or fixture.
    pub fn sink(&self) -> TraceSink { self.sink.clone() }
    /// Stop recording and extract without waiting. Call only after actor cleanup.
    pub fn finish(self) -> TraceReport {
        let shared = &self.sink.shared;
        shared.enabled.store(false, Ordering::Release);
        let open_spans_at_finish = shared.open_spans.load(Ordering::Acquire);
        let (records, finish_buffer_unavailable) = match shared.records.try_lock() {
            Ok(mut records) => (std::mem::take(&mut *records), false),
            Err(_) => (Vec::new(), true),
        };
        let mut report = TraceReport {
            origin: shared.origin,
            record_size_bytes: std::mem::size_of::<TraceRecord>(),
            record_capacity_bytes: std::mem::size_of::<TraceRecord>() * shared.limit,
            records, capacity: shared.limit,
            dropped_contended: shared.contended.load(Ordering::Relaxed),
            dropped_overflow: shared.overflow.load(Ordering::Relaxed),
            dropped_poisoned: shared.poisoned.load(Ordering::Relaxed),
            dropped_closed_race: shared.closed_race.load(Ordering::Relaxed),
            open_spans_at_finish,
            finish_buffer_unavailable, incomplete: false,
        };
        report.incomplete = report.finish_buffer_unavailable
            || report.dropped_contended != 0 || report.dropped_overflow != 0
            || report.dropped_poisoned != 0 || report.dropped_closed_race != 0
            || report.open_spans_at_finish != 0;
        report
    }
}

impl Drop for TraceSession {
    fn drop(&mut self) {
        // A forgotten finish must never leave recording enabled on surviving sinks.
        self.sink.shared.enabled.store(false, Ordering::Release);
    }
}

impl TraceSink {
    pub(crate) fn enabled(&self) -> bool { self.shared.enabled.load(Ordering::Acquire) }
    fn at(&self, now: Instant) -> u64 { nanos(now.saturating_duration_since(self.shared.origin)) }
    fn push(&self, phase: &'static str, source: i64, at_ns: u64, data: TraceData) {
        match self.shared.records.try_lock() {
            Ok(mut records) => {
                if !self.enabled() {
                    self.shared.closed_race.fetch_add(1, Ordering::Relaxed);
                } else if records.len() == self.shared.limit {
                    self.shared.overflow.fetch_add(1, Ordering::Relaxed);
                } else {
                    // The initial allocation owns every slot; no growth occurs.
                    records.push(TraceRecord { phase, source, at_ns, thread: std::thread::current().id(), data });
                }
            }
            Err(TryLockError::WouldBlock) => { self.shared.contended.fetch_add(1, Ordering::Relaxed); }
            Err(TryLockError::Poisoned(_)) => { self.shared.poisoned.fetch_add(1, Ordering::Relaxed); }
        }
    }
    /// Record a scalar phase only while explicitly enabled.
    pub fn mark(&self, phase: &'static str, source: i64) {
        if self.enabled() { self.push(phase, source, self.at(Instant::now()), TraceData::Mark); }
    }
    /// Record an already available scalar count, without allocating payloads.
    pub fn count(&self, phase: &'static str, source: i64, count: u64) {
        if self.enabled() { self.push(phase, source, self.at(Instant::now()), TraceData::Count(count)); }
    }
    /// Record an existing bound without interpreting it as a velocity lease.
    pub fn bound(&self, phase: &'static str, source: i64, bound: Option<crate::CommandDispatchBound>) {
        if !self.enabled() { return; }
        let now = Instant::now();
        let fields = bound.map(|bound| bound.qualification_timing_fields());
        self.push(phase, source, self.at(now), TraceData::Bound {
            present: fields.is_some(), anchor_unix_ns: fields.map(|(anchor, _, _)| anchor),
            latest_issue_unix_ns: fields.map(|(_, latest, _)| latest),
            remaining_ns: fields.and_then(|(_, _, deadline)| deadline).map(|deadline| remaining_ns(deadline, now)),
        });
    }
    /// Record actual command scalars; no motion is performed or evaluated.
    pub fn motion(&self, phase: &'static str, source: i64, motion: [f64; 3]) {
        if self.enabled() { self.push(phase, source, self.at(Instant::now()), TraceData::Motion(motion)); }
    }
    /// Preserve the exact Instant used to sample all fields in this snapshot.
    /// The caller must not pass a new clock sample after building the snapshot.
    pub fn snapshot_at(&self, phase: &'static str, source: i64, sampled_at: Instant, snapshot: SchedulerSnapshot) {
        if self.enabled() { self.push(phase, source, self.at(sampled_at), TraceData::Scheduler(snapshot)); }
    }
    /// Start a scope, retaining no lock. Its drop records elapsed scope time.
    pub fn span(&self, phase: &'static str, source: i64) -> SpanGuard {
        if !self.enabled() { return SpanGuard { active: None }; }
        let started = Instant::now();
        let id = self.shared.next_span.fetch_add(1, Ordering::Relaxed);
        self.shared.open_spans.fetch_add(1, Ordering::Relaxed);
        self.push(phase, source, self.at(started), TraceData::SpanStart { id });
        SpanGuard { active: Some((self.clone(), phase, source, id, started)) }
    }
}

/// RAII elapsed scope observer. It never holds the recording mutex.
pub struct SpanGuard { active: Option<(TraceSink, &'static str, i64, u64, Instant)> }
impl Drop for SpanGuard {
    fn drop(&mut self) {
        if let Some((sink, phase, source, id, started)) = self.active.take() {
            if sink.enabled() {
                let now = Instant::now();
                sink.push(phase, source, sink.at(now), TraceData::SpanEnd {
                    id, elapsed_ns: nanos(now.saturating_duration_since(started)),
                });
            } else {
                // A scope closed after finish without an end record.
                sink.shared.closed_race.fetch_add(1, Ordering::Relaxed);
            }
            sink.shared.open_spans.fetch_sub(1, Ordering::Release);
        }
    }
}

pub(crate) fn nanos(duration: std::time::Duration) -> u64 {
    duration.as_nanos().min(u128::from(u64::MAX)) as u64
}
pub(crate) fn remaining_ns(deadline: Instant, now: Instant) -> i64 {
    if deadline >= now {
        nanos(deadline.duration_since(now)).min(i64::MAX as u64) as i64
    } else {
        -(nanos(now.duration_since(deadline)).min(i64::MAX as u64) as i64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_capacity_overflow_is_explicit() {
        let session = TraceSession::start(1);
        let sink = session.sink();
        sink.mark("first", 1);
        sink.count("overflow", 2, 3);
        let report = session.finish();
        assert_eq!(report.records.len(), 1);
        assert_eq!(report.dropped_overflow, 1);
        assert!(report.incomplete);
        assert_eq!(report.record_capacity_bytes, std::mem::size_of::<TraceRecord>());
    }

    #[test]
    fn closed_sink_records_nothing_and_starts_no_scope() {
        let session = TraceSession::start(4);
        let sink = session.sink();
        let report = session.finish();
        let next = sink.shared.next_span.load(Ordering::Relaxed);
        sink.mark("disabled", 1);
        sink.count("disabled", 1, 2);
        drop(sink.span("disabled", 1));
        assert!(sink.shared.records.try_lock().unwrap().is_empty());
        assert_eq!(sink.shared.next_span.load(Ordering::Relaxed), next);
        assert_eq!(sink.shared.open_spans.load(Ordering::Relaxed), 0);
        assert!(report.records.is_empty());
        assert!(!report.incomplete);
    }

    #[test]
    fn open_scope_makes_finish_incomplete() {
        let session = TraceSession::start(4);
        let guard = session.sink().span("open", 1);
        let report = session.finish();
        assert_eq!(report.open_spans_at_finish, 1);
        assert!(report.incomplete);
        assert!(matches!(report.records[0].data, TraceData::SpanStart { .. }));
        drop(guard);
    }

    #[test]
    fn independent_explicit_sinks_have_no_global_recording() {
        let left = TraceSession::start(4);
        let right = TraceSession::start(4);
        left.sink().mark("left_only", 1);
        assert_eq!(left.finish().records.len(), 1);
        assert!(right.finish().records.is_empty());
    }

    #[test]
    fn dropping_session_disables_surviving_sink() {
        let session = TraceSession::start(4);
        let sink = session.sink();
        drop(session);
        assert!(!sink.enabled());
        sink.mark("after_drop", 1);
        assert!(sink.shared.records.try_lock().unwrap().is_empty());
    }

    #[test]
    fn busy_buffer_drops_without_waiting() {
        let session = TraceSession::start(4);
        let sink = session.sink();
        let held = sink.shared.records.try_lock().unwrap();
        sink.mark("contended", 1);
        drop(held);
        let report = session.finish();
        assert_eq!(report.dropped_contended, 1);
        assert!(report.incomplete);
    }
}
