//! Bounded, reusable ownership of exploration computation.
//!
//! This module deliberately knows nothing about the outer runtime mutex. A backend
//! owns the controller and its pending delta. Requests own immutable captured input;
//! projections are advice only. The caller still validates its basis before Accept
//! and revalidates admission before dispatching an accepted result.
//!
//! There is one worker, one replaceable request, one transaction/completion, one
//! immutable decision, and one nonreplaceable feedback-or-lifecycle operation/ack.
//! All payloads need only Send. Payloads move out of the lock before destruction.
//! Ordinary algorithm errors belong in backend payloads, including terminal feedback
//! error acknowledgements: the exchange never retries a backend operation.
//!
//! Lifecycle commands (install/begin, end/export, etc.) are explicit backend values.
//! Close admission first; submit a command, consume its acknowledgement, then open
//! the next session. Commands cannot replace each other or accepted feedback.
//! If a lifecycle submission returns Busy, the outer driver MUST retain its latest
//! desired lifecycle intent and remain alive after stop/cancelled waiters to drain
//! feedback acknowledgement and resubmit that intent. stop_admission retains the
//! revocation even while the operation slot is occupied; it does not enqueue end.
//! Shutdown settles accepted work but does not invent a backend end/export command.
//!
//! Keep ComputeOwner in the root task supervisor. try_join never waits and retains
//! an unfinished join handle. Drop is a blocking last-resort join, never a detached
//! thread; do not drop an unsettled owner on an async executor or under runtime locks.

use std::fmt;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
#[cfg(test)]
use std::time::Duration;
use tokio::sync::Notify;

static NEXT_INCARNATION: AtomicU64 = AtomicU64::new(1);

/// Errors from an algorithm are values in associated types, not exchange failures.
/// A successful prepare must retain exactly one delta, even if its projection is Err.
/// Discard must apply none of that delta. Accepted resolve may return an error value
/// in Finalized; its side effects are nevertheless settled, and it is never retried.
pub trait Backend: Send + 'static {
    type Request: Send + 'static;
    type Projection: Send + 'static;
    type Finalized: Send + 'static;
    type Feedback: Send + 'static;
    type FeedbackAck: Send + 'static;
    type Lifecycle: Send + 'static;
    type LifecycleAck: Send + 'static;

    fn prepare(&mut self, request: Self::Request, cancellation: &Cancellation) -> Self::Projection;
    fn resolve(&mut self, decision: Decision) -> Settlement<Self::Finalized>;
    fn feedback(&mut self, operation: Self::Feedback) -> Self::FeedbackAck;
    fn lifecycle(&mut self, operation: Self::Lifecycle) -> Self::LifecycleAck;
}

/// Cooperatively checked between feasible algorithm stages, never preemption.
#[derive(Clone)]
pub struct Cancellation(Arc<AtomicBool>);
impl Cancellation {
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }
    /// Borrow the owner's original flag for cooperative planner checkpoints.
    pub(crate) fn flag(&self) -> &AtomicBool {
        self.0.as_ref()
    }
    /// Share this same owner flag with synchronous map-construction checkpoints.
    pub(crate) fn shared_flag(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.0)
    }
    fn new() -> Self {
        Self(Arc::new(AtomicBool::new(false)))
    }
    fn cancel(&self) {
        self.0.store(true, Ordering::Release);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Id {
    pub incarnation: u64,
    pub sequence: u64,
    pub generation: u64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Accept,
    Discard,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecisionAck {
    pub id: Id,
    pub decision: Decision,
}
#[derive(Debug)]
pub enum Settlement<T> {
    Accepted(T),
    Discarded,
}
#[derive(Debug)]
pub struct Prepared<P> {
    pub id: Id,
    pub projection: P,
}
#[derive(Debug)]
pub struct Settled<F> {
    pub acknowledgement: DecisionAck,
    pub result: Settlement<F>,
}
#[derive(Debug)]
pub enum OperationAck<F, L> {
    Feedback { id: Id, result: F },
    Lifecycle { id: Id, result: L },
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperationKind {
    Feedback,
    Lifecycle,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Idle,
    Computing,
    PreparedAwaitResolution,
    Finalizing,
    CommittedAwaitConsumption,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperationPhase {
    Queued,
    Running,
    Acknowledged,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    Panic,
    BackendProtocol,
    CounterExhausted,
    UnexpectedExit,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OwnerError {
    Closed,
    Faulted(Fault),
    AdmissionClosed,
    AdmissionOpen,
    Busy,
    StaleTransaction,
    DecisionConflict(DecisionAck),
    CounterExhausted,
}
impl fmt::Display for OwnerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for OwnerError {}

/// Exchange-slot counts, not payload byte bounds and not external retained values.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HighWater {
    pub active: usize,
    pub latest: usize,
    pub completion: usize,
    pub resolution: usize,
    pub operation: usize,
    pub workers: usize,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub epoch: u64,
    pub incarnation: u64,
    pub generation: u64,
    pub admitting: bool,
    pub phase: Phase,
    pub active: Option<Id>,
    pub latest: Option<Id>,
    pub prepared_available: bool,
    pub settled_available: bool,
    pub decision: Option<DecisionAck>,
    pub operation: Option<(Id, OperationKind, OperationPhase)>,
    pub shutting_down: bool,
    pub closed: bool,
    pub fault: Option<Fault>,
    pub high_water: HighWater,
}

struct Request<R> {
    id: Id,
    value: R,
}
struct Active {
    id: Id,
    phase: Phase,
    decision: Option<DecisionAck>,
    cancellation: Cancellation,
}
enum Completion<P, F> {
    Prepared(Prepared<P>),
    Settled(Settled<F>),
}
enum Operation<F, L> {
    Feedback(F),
    Lifecycle(L),
}
enum OperationState<F, L, FA, LA> {
    Queued(Operation<F, L>),
    Running,
    Ack(OperationAck<FA, LA>),
}
struct OperationSlot<F, L, FA, LA> {
    id: Id,
    kind: OperationKind,
    state: OperationState<F, L, FA, LA>,
}
struct State<B: Backend> {
    incarnation: u64,
    sequence: u64,
    generation: u64,
    epoch: u64,
    admitting: bool,
    shutting_down: bool,
    closed: bool,
    fault: Option<Fault>,
    latest: Option<Request<B::Request>>,
    active: Option<Active>,
    completion: Option<Completion<B::Projection, B::Finalized>>,
    operation: Option<OperationSlot<B::Feedback, B::Lifecycle, B::FeedbackAck, B::LifecycleAck>>,
    last_decision: Option<DecisionAck>,
    high_water: HighWater,
}
struct Shared<B: Backend> {
    state: Mutex<State<B>>,
    wake: Condvar,
    changed: Notify,
}

impl<B: Backend> Shared<B> {
    fn lock(&self) -> MutexGuard<'_, State<B>> {
        // No backend code runs with this lock. Recovery is only for our own bookkeeping;
        // a backend panic is handled separately and permanently faults the owner.
        self.state.lock().unwrap_or_else(|p| p.into_inner())
    }
    fn signal(&self) {
        self.wake.notify_all();
        self.changed.notify_waiters();
    }
}
impl<B: Backend> State<B> {
    fn live(&self) -> Result<(), OwnerError> {
        if let Some(f) = self.fault {
            return Err(OwnerError::Faulted(f));
        }
        if self.closed || self.shutting_down {
            return Err(OwnerError::Closed);
        }
        Ok(())
    }
    fn id(&mut self) -> Result<Id, OwnerError> {
        let sequence = self
            .sequence
            .checked_add(1)
            .ok_or(OwnerError::CounterExhausted)?;
        self.sequence = sequence;
        Ok(Id {
            incarnation: self.incarnation,
            sequence,
            generation: self.generation,
        })
    }
    fn advance(&mut self) {
        match self.epoch.checked_add(1) {
            Some(epoch) => self.epoch = epoch,
            None => {
                self.fault = Some(Fault::CounterExhausted);
                self.admitting = false;
            }
        }
        self.high_water.active = self
            .high_water
            .active
            .max(usize::from(self.active.is_some()));
        self.high_water.latest = self
            .high_water
            .latest
            .max(usize::from(self.latest.is_some()));
        self.high_water.completion = self
            .high_water
            .completion
            .max(usize::from(self.completion.is_some()));
        self.high_water.resolution = self.high_water.resolution.max(usize::from(
            self.active.as_ref().is_some_and(|a| a.decision.is_some()),
        ));
        self.high_water.operation = self
            .high_water
            .operation
            .max(usize::from(self.operation.is_some()));
    }
    fn cancel_unresolved(&mut self) {
        if let Some(active) = &mut self.active {
            active.cancellation.cancel();
            if active.decision.is_none() {
                active.decision = Some(DecisionAck {
                    id: active.id,
                    decision: Decision::Discard,
                });
            }
        }
    }
    fn status(&self) -> Status {
        let operation = self.operation.as_ref().map(|op| {
            (
                op.id,
                op.kind,
                match &op.state {
                    OperationState::Queued(_) => OperationPhase::Queued,
                    OperationState::Running => OperationPhase::Running,
                    OperationState::Ack(_) => OperationPhase::Acknowledged,
                },
            )
        });
        Status {
            epoch: self.epoch,
            incarnation: self.incarnation,
            generation: self.generation,
            admitting: self.admitting,
            phase: self.active.as_ref().map_or(Phase::Idle, |a| a.phase),
            active: self.active.as_ref().map(|a| a.id),
            latest: self.latest.as_ref().map(|r| r.id),
            prepared_available: matches!(self.completion, Some(Completion::Prepared(_))),
            settled_available: matches!(self.completion, Some(Completion::Settled(_))),
            decision: self.active.as_ref().and_then(|a| a.decision),
            operation,
            shutting_down: self.shutting_down,
            closed: self.closed,
            fault: self.fault,
            high_water: self.high_water,
        }
    }
}

pub struct OwnerHandle<B: Backend> {
    shared: Arc<Shared<B>>,
}
impl<B: Backend> Clone for OwnerHandle<B> {
    fn clone(&self) -> Self {
        Self {
            shared: self.shared.clone(),
        }
    }
}

pub struct ComputeOwner<B: Backend> {
    handle: OwnerHandle<B>,
    worker: Option<JoinHandle<()>>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JoinStatus {
    Pending,
    Joined,
}
impl<B: Backend> ComputeOwner<B> {
    pub fn spawn(backend: B) -> Result<Self, std::io::Error> {
        let incarnation = NEXT_INCARNATION
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_add(1))
            .map_err(|_| std::io::Error::other("compute-owner incarnation exhausted"))?;
        let shared = Arc::new(Shared {
            state: Mutex::new(State {
                incarnation,
                sequence: 0,
                generation: 0,
                epoch: 0,
                admitting: false,
                shutting_down: false,
                closed: false,
                fault: None,
                latest: None,
                active: None,
                completion: None,
                operation: None,
                last_decision: None,
                high_water: HighWater {
                    workers: 1,
                    ..HighWater::default()
                },
            }),
            wake: Condvar::new(),
            changed: Notify::new(),
        });
        let worker_shared = shared.clone();
        let worker = thread::Builder::new()
            .name("exploration-compute".into())
            .spawn(move || {
                let mut backend = backend;
                let result = catch_unwind(AssertUnwindSafe(|| {
                    worker_loop(&worker_shared, &mut backend)
                }));
                if result.is_err() {
                    publish_fault(&worker_shared, Fault::Panic);
                }
                // Even controller destruction stays off the caller and exchange mutex.
                if catch_unwind(AssertUnwindSafe(|| drop(backend))).is_err() {
                    publish_fault(&worker_shared, Fault::Panic);
                }
                {
                    let mut state = worker_shared.lock();
                    state.closed = true;
                    state.admitting = false;
                    state.advance();
                }
                worker_shared.signal();
            })?;
        Ok(Self {
            handle: OwnerHandle { shared },
            worker: Some(worker),
        })
    }
    pub fn handle(&self) -> OwnerHandle<B> {
        self.handle.clone()
    }
    pub fn try_join(&mut self) -> JoinStatus {
        if self
            .worker
            .as_ref()
            .is_some_and(|worker| !worker.is_finished())
        {
            return JoinStatus::Pending;
        }
        if let Some(worker) = self.worker.take() {
            if worker.join().is_err() {
                publish_fault(&self.handle.shared, Fault::UnexpectedExit);
            }
        }
        JoinStatus::Joined
    }
}
impl<B: Backend> Drop for ComputeOwner<B> {
    fn drop(&mut self) {
        self.handle.shutdown();
        if let Some(worker) = self.worker.take() {
            if worker.join().is_err() {
                publish_fault(&self.handle.shared, Fault::UnexpectedExit);
            }
        }
    }
}

impl<B: Backend> OwnerHandle<B> {
    pub fn status(&self) -> Status {
        self.shared.lock().status()
    }

    /// Await an observable change, with registration before the predicate check.
    /// Safe with multiple waiters; no stored permit or check/register lost-wakeup race.
    pub async fn changed(&self, observed_epoch: u64) -> Status {
        loop {
            let notified = self.shared.changed.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            let status = self.status();
            if status.epoch != observed_epoch || status.closed || status.fault.is_some() {
                return status;
            }
            notified.await;
        }
    }

    #[cfg(test)]
    /// Blocking observer for native supervision/tests; never call under outer locks.
    pub fn wait_changed(&self, observed_epoch: u64, timeout: Duration) -> Status {
        let state = self.shared.lock();
        let (state, _) = self
            .shared
            .wake
            .wait_timeout_while(state, timeout, |s| {
                s.epoch == observed_epoch && !s.closed && s.fault.is_none()
            })
            .unwrap_or_else(|p| p.into_inner());
        state.status()
    }

    /// Explicit session boundary. Install/begin must already be acknowledged.
    pub fn open_admission(&self) -> Result<u64, OwnerError> {
        let generation = {
            let mut state = self.shared.lock();
            state.live()?;
            if state.admitting {
                return Err(OwnerError::AdmissionOpen);
            }
            if state.active.is_some() || state.latest.is_some() || state.operation.is_some() {
                return Err(OwnerError::Busy);
            }
            state.generation = state
                .generation
                .checked_add(1)
                .ok_or(OwnerError::CounterExhausted)?;
            state.admitting = true;
            state.advance();
            state.generation
        };
        self.shared.signal();
        Ok(generation)
    }

    /// Soft supersession never cancels/restarts the active job. Retires old input
    /// outside the exchange mutex. Only the latest successor is retained.
    pub fn submit(&self, request: B::Request) -> Result<Id, OwnerError> {
        let (id, retired) = {
            let mut state = self.shared.lock();
            state.live()?;
            if !state.admitting {
                return Err(OwnerError::AdmissionClosed);
            }
            let id = state.id()?;
            let retired = state.latest.replace(Request { id, value: request });
            state.advance();
            (id, retired)
        };
        self.shared.signal();
        drop(retired);
        Ok(id)
    }

    /// Revokes eligibility immediately, independently of held backend work. An
    /// existing Accept is immutable. The epoch is suitable for changed(epoch).
    pub fn stop_admission(&self) -> u64 {
        let (epoch, retired) = {
            let mut state = self.shared.lock();
            state.admitting = false;
            state.cancel_unresolved();
            let retired = state.latest.take();
            state.advance();
            (state.epoch, retired)
        };
        self.shared.signal();
        drop(retired);
        epoch
    }

    pub fn try_take_prepared(&self) -> Result<Option<Prepared<B::Projection>>, OwnerError> {
        let result = {
            let mut state = self.shared.lock();
            if let Some(fault) = state.fault {
                return Err(OwnerError::Faulted(fault));
            }
            if !matches!(state.completion, Some(Completion::Prepared(_))) {
                return Ok(None);
            }
            // A cancellation/feedback fence may already have decided Discard.
            if state.active.as_ref().is_some_and(|a| a.decision.is_some()) {
                return Ok(None);
            }
            let Some(Completion::Prepared(prepared)) = state.completion.take() else {
                unreachable!()
            };
            state.advance();
            prepared
        };
        self.shared.signal();
        Ok(Some(result))
    }

    /// The immutable decision write is the logical commit linearization point.
    /// Matching retries return its existing acknowledgement; conflicts never mutate it.
    pub fn resolve(&self, id: Id, decision: Decision) -> Result<DecisionAck, OwnerError> {
        let ack = {
            let mut state = self.shared.lock();
            if let Some(fault) = state.fault {
                return Err(OwnerError::Faulted(fault));
            }
            if let Some(ack) = state.last_decision.filter(|ack| ack.id == id) {
                return if ack.decision == decision {
                    Ok(ack)
                } else {
                    Err(OwnerError::DecisionConflict(ack))
                };
            }
            let Some(active) = state.active.as_mut().filter(|a| a.id == id) else {
                return Err(OwnerError::StaleTransaction);
            };
            if let Some(ack) = active.decision {
                return if ack.decision == decision {
                    Ok(ack)
                } else {
                    Err(OwnerError::DecisionConflict(ack))
                };
            }
            if active.phase != Phase::PreparedAwaitResolution {
                return Err(OwnerError::Busy);
            }
            let ack = DecisionAck { id, decision };
            active.decision = Some(ack);
            if decision == Decision::Discard {
                active.cancellation.cancel();
            }
            state.advance();
            ack
        };
        self.shared.signal();
        Ok(ack)
    }

    /// Consuming final settlement frees the transaction slot. No newer prepared
    /// projection can overwrite an unconsumed accepted/discarded settlement.
    pub fn try_take_settled(&self) -> Result<Option<Settled<B::Finalized>>, OwnerError> {
        let result = {
            let mut state = self.shared.lock();
            if let Some(fault) = state.fault {
                return Err(OwnerError::Faulted(fault));
            }
            if !matches!(state.completion, Some(Completion::Settled(_))) {
                return Ok(None);
            }
            let Some(Completion::Settled(settled)) = state.completion.take() else {
                unreachable!()
            };
            state.last_decision = Some(settled.acknowledgement);
            let active = state.active.take();
            state.advance();
            (settled, active)
        };
        self.shared.signal();
        drop(result.1);
        Ok(Some(result.0))
    }

    /// Only one accepted operation can be unresolved/unconsumed. Outer exact child,
    /// goal and terminal validation remains authoritative. Preserve the returned ID
    /// for joining a pending acknowledgement; never resubmit an operation on Err ack.
    /// Accepting feedback discards any undecided proposal and coalesced old-history
    /// request, but never changes an existing Accept. Worker resolves before feedback.
    pub fn submit_feedback(&self, feedback: B::Feedback) -> Result<Id, OwnerError> {
        let (id, retired) = {
            let mut state = self.shared.lock();
            state.live()?;
            if !state.admitting {
                return Err(OwnerError::AdmissionClosed);
            }
            if state.operation.is_some() {
                return Err(OwnerError::Busy);
            }
            let id = state.id()?;
            state.cancel_unresolved();
            let retired = state.latest.take();
            state.operation = Some(OperationSlot {
                id,
                kind: OperationKind::Feedback,
                state: OperationState::Queued(Operation::Feedback(feedback)),
            });
            state.advance();
            (id, retired)
        };
        self.shared.signal();
        drop(retired);
        Ok(id)
    }

    /// Lifecycle is an ordered, nonreplaceable operation, not a control queue.
    /// Close admission before submitting. Wait for feedback acknowledgement if Busy.
    pub fn submit_lifecycle(&self, lifecycle: B::Lifecycle) -> Result<Id, OwnerError> {
        let id = {
            let mut state = self.shared.lock();
            state.live()?;
            if state.admitting {
                return Err(OwnerError::AdmissionOpen);
            }
            if state.operation.is_some() {
                return Err(OwnerError::Busy);
            }
            let id = state.id()?;
            state.operation = Some(OperationSlot {
                id,
                kind: OperationKind::Lifecycle,
                state: OperationState::Queued(Operation::Lifecycle(lifecycle)),
            });
            state.advance();
            id
        };
        self.shared.signal();
        Ok(id)
    }

    pub fn try_take_operation_ack(
        &self,
        id: Id,
    ) -> Result<Option<OperationAck<B::FeedbackAck, B::LifecycleAck>>, OwnerError> {
        let ack = {
            let mut state = self.shared.lock();
            if let Some(fault) = state.fault {
                return Err(OwnerError::Faulted(fault));
            }
            let Some(slot) = state.operation.as_ref().filter(|op| op.id == id) else {
                return Err(OwnerError::StaleTransaction);
            };
            if !matches!(slot.state, OperationState::Ack(_)) {
                return Ok(None);
            }
            let Some(slot) = state.operation.take() else {
                unreachable!()
            };
            let OperationState::Ack(ack) = slot.state else {
                unreachable!()
            };
            state.advance();
            ack
        };
        self.shared.signal();
        Ok(Some(ack))
    }

    /// Nonblocking shutdown request. Accepted resolution/feedback/lifecycle still
    /// execute exactly once; undecided work is discarded after prepare returns.
    pub fn shutdown(&self) -> u64 {
        let (epoch, retired) = {
            let mut state = self.shared.lock();
            state.admitting = false;
            state.shutting_down = true;
            state.cancel_unresolved();
            let retired = state.latest.take();
            state.advance();
            (state.epoch, retired)
        };
        self.shared.signal();
        drop(retired);
        epoch
    }
}

enum Work<B: Backend> {
    Prepare(Request<B::Request>, Cancellation),
    Resolve(DecisionAck, Option<Completion<B::Projection, B::Finalized>>),
    Feedback(Id, B::Feedback),
    Lifecycle(Id, B::Lifecycle),
    Exit,
}

fn next_work<B: Backend>(shared: &Shared<B>) -> Work<B> {
    let mut state = shared.lock();
    loop {
        if state.fault.is_some() {
            return Work::Exit;
        }
        if let Some(active) = &mut state.active {
            if active.phase == Phase::PreparedAwaitResolution {
                if let Some(ack) = active.decision {
                    active.phase = Phase::Finalizing;
                    let retired = state.completion.take();
                    state.advance();
                    return Work::Resolve(ack, retired);
                }
                state = shared.wake.wait(state).unwrap_or_else(|p| p.into_inner());
                continue;
            }
        }
        if let Some(slot) = &mut state.operation {
            if matches!(slot.state, OperationState::Queued(_)) {
                let previous = std::mem::replace(&mut slot.state, OperationState::Running);
                let id = slot.id;
                let OperationState::Queued(op) = previous else {
                    unreachable!()
                };
                state.advance();
                return match op {
                    Operation::Feedback(value) => Work::Feedback(id, value),
                    Operation::Lifecycle(value) => Work::Lifecycle(id, value),
                };
            }
        }
        if state.shutting_down {
            return Work::Exit;
        }
        // Both settlement and operation acknowledgement are consumption barriers.
        if state.admitting && state.active.is_none() && state.operation.is_none() {
            if let Some(request) = state.latest.take() {
                let cancellation = Cancellation::new();
                state.active = Some(Active {
                    id: request.id,
                    phase: Phase::Computing,
                    decision: None,
                    cancellation: cancellation.clone(),
                });
                state.advance();
                return Work::Prepare(request, cancellation);
            }
        }
        state = shared.wake.wait(state).unwrap_or_else(|p| p.into_inner());
    }
}

fn worker_loop<B: Backend>(shared: &Shared<B>, backend: &mut B) {
    loop {
        let work = next_work(shared);
        shared.signal();
        match work {
            Work::Prepare(request, cancellation) => {
                let projection = backend.prepare(request.value, &cancellation);
                {
                    let mut state = shared.lock();
                    state.completion = Some(Completion::Prepared(Prepared {
                        id: request.id,
                        projection,
                    }));
                    state.active.as_mut().expect("active prepare").phase =
                        Phase::PreparedAwaitResolution;
                    state.advance();
                }
                shared.signal();
            }
            Work::Resolve(ack, retired) => {
                drop(retired);
                let result = backend.resolve(ack.decision);
                if !matches!(
                    (&result, ack.decision),
                    (Settlement::Accepted(_), Decision::Accept)
                        | (Settlement::Discarded, Decision::Discard)
                ) {
                    publish_fault(shared, Fault::BackendProtocol);
                    drop(result);
                    return;
                }
                {
                    let mut state = shared.lock();
                    state.completion = Some(Completion::Settled(Settled {
                        acknowledgement: ack,
                        result,
                    }));
                    state.active.as_mut().expect("active resolve").phase =
                        Phase::CommittedAwaitConsumption;
                    state.advance();
                }
                shared.signal();
            }
            Work::Feedback(id, operation) => {
                let result = backend.feedback(operation);
                publish_operation(shared, id, OperationAck::Feedback { id, result });
            }
            Work::Lifecycle(id, operation) => {
                let result = backend.lifecycle(operation);
                publish_operation(shared, id, OperationAck::Lifecycle { id, result });
            }
            Work::Exit => return,
        }
    }
}

fn publish_operation<B: Backend>(
    shared: &Shared<B>,
    id: Id,
    ack: OperationAck<B::FeedbackAck, B::LifecycleAck>,
) {
    {
        let mut state = shared.lock();
        let slot = state.operation.as_mut().expect("accepted operation");
        assert_eq!(slot.id, id);
        slot.state = OperationState::Ack(ack);
        state.advance();
    }
    shared.signal();
}
fn publish_fault<B: Backend>(shared: &Shared<B>, fault: Fault) {
    // Move all potentially large payloads out before destruction. A panic makes
    // every completion ineligible, including a previously accepted projection.
    let retired = {
        let mut state = shared.lock();
        state.fault.get_or_insert(fault);
        state.admitting = false;
        state.cancel_unresolved();
        let retired = (
            state.latest.take(),
            state.completion.take(),
            state.operation.take(),
            state.active.take(),
        );
        state.advance();
        retired
    };
    shared.signal();
    drop(retired);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::thread::ThreadId;
    use std::time::Instant;

    #[test]
    fn cancellation_flag_borrows_original_owner_token() {
        let cancellation = Cancellation::new();
        let same_token = cancellation.clone();
        assert!(std::ptr::eq(cancellation.flag(), same_token.flag()));
        assert!(std::ptr::eq(cancellation.flag(), cancellation.shared_flag().as_ref()));
        assert!(!cancellation.flag().load(Ordering::Acquire));
        same_token.cancel();
        assert!(cancellation.flag().load(Ordering::Acquire));
        assert!(cancellation.is_cancelled());
    }

    // Timeouts are deadlock watchdogs only. Every tested interleaving is established
    // by gate/phase acknowledgement, never by sleeping or assuming compute speed.
    const WATCHDOG: Duration = Duration::from_secs(5);
    #[derive(Clone, Default)]
    struct Gate(Arc<(Mutex<(bool, bool)>, Condvar)>);
    impl Gate {
        fn enter_and_wait(&self) {
            let (mutex, cv) = &*self.0;
            let mut state = mutex.lock().unwrap();
            state.0 = true;
            cv.notify_all();
            while !state.1 {
                state = cv.wait(state).unwrap();
            }
        }
        fn wait_entered(&self) {
            let (mutex, cv) = &*self.0;
            let state = mutex.lock().unwrap();
            let (state, timeout) = cv.wait_timeout_while(state, WATCHDOG, |s| !s.0).unwrap();
            assert!(
                !timeout.timed_out() && state.0,
                "worker did not enter deterministic gate"
            );
        }
        fn release(&self) {
            let (mutex, cv) = &*self.0;
            let mut state = mutex.lock().unwrap();
            state.1 = true;
            cv.notify_all();
        }
    }
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Event {
        Prepare(i32),
        Cancelled(bool),
        Accept(i32),
        Discard(i32),
        Feedback(i32),
        Lifecycle(&'static str, i32),
    }
    #[derive(Default)]
    struct Log {
        events: Vec<Event>,
        workers: Vec<ThreadId>,
    }
    struct Mock {
        log: Arc<Mutex<Log>>,
        pending: Option<Job>,
        history: i32,
        panic_resolve: bool,
        bad_resolution: bool,
    }
    struct Job {
        value: i32,
        prepare_gate: Option<Gate>,
        resolve_gate: Option<Gate>,
        panic: bool,
        drop_probe: Option<DropProbe>,
    }
    impl Job {
        fn new(value: i32) -> Self {
            Self {
                value,
                prepare_gate: None,
                resolve_gate: None,
                panic: false,
                drop_probe: None,
            }
        }
    }
    struct DropProbe(Box<dyn Fn() + Send>);
    impl Drop for DropProbe {
        fn drop(&mut self) {
            (self.0)();
        }
    }
    // Cell is Send and !Sync: the public exchange must not require Sync payloads.
    struct Projection {
        value: Cell<i32>,
    }
    struct Feedback {
        value: i32,
        gate: Option<Gate>,
        error: bool,
        panic: bool,
    }
    enum Lifecycle {
        Install,
        EndExport,
        Panic,
    }
    impl Mock {
        fn record(&self, event: Event) {
            let mut log = self.log.lock().unwrap();
            log.events.push(event);
            log.workers.push(thread::current().id());
        }
    }
    impl Backend for Mock {
        type Request = Job;
        type Projection = Projection;
        type Finalized = i32;
        type Feedback = Feedback;
        type FeedbackAck = Result<i32, &'static str>;
        type Lifecycle = Lifecycle;
        type LifecycleAck = i32;
        fn prepare(&mut self, request: Job, cancellation: &Cancellation) -> Projection {
            assert!(self.pending.is_none());
            self.record(Event::Prepare(request.value));
            if let Some(gate) = &request.prepare_gate {
                gate.enter_and_wait();
            }
            assert!(!request.panic, "deliberate prepare panic");
            self.record(Event::Cancelled(cancellation.is_cancelled()));
            let value = request.value;
            self.pending = Some(request);
            Projection {
                value: Cell::new(value),
            }
        }
        fn resolve(&mut self, decision: Decision) -> Settlement<i32> {
            assert!(!self.panic_resolve, "deliberate resolve panic");
            let request = self.pending.take().expect("exactly one prepared delta");
            if let Some(gate) = &request.resolve_gate {
                gate.enter_and_wait();
            }
            if self.bad_resolution {
                return Settlement::Discarded;
            }
            match decision {
                Decision::Accept => {
                    self.history += request.value;
                    self.record(Event::Accept(request.value));
                    Settlement::Accepted(self.history)
                }
                Decision::Discard => {
                    self.record(Event::Discard(request.value));
                    Settlement::Discarded
                }
            }
        }
        fn feedback(&mut self, feedback: Feedback) -> Self::FeedbackAck {
            assert!(self.pending.is_none());
            if let Some(gate) = feedback.gate {
                gate.enter_and_wait();
            }
            self.history += feedback.value;
            self.record(Event::Feedback(feedback.value));
            assert!(!feedback.panic, "deliberate partial-feedback panic");
            if feedback.error {
                Err("partial-source feedback error")
            } else {
                Ok(self.history)
            }
        }
        fn lifecycle(&mut self, lifecycle: Lifecycle) -> i32 {
            assert!(self.pending.is_none());
            let name = match lifecycle {
                Lifecycle::Install => "install",
                Lifecycle::EndExport => "end/export",
                Lifecycle::Panic => panic!("deliberate lifecycle panic"),
            };
            self.record(Event::Lifecycle(name, self.history));
            self.history
        }
    }
    fn fixture() -> (ComputeOwner<Mock>, OwnerHandle<Mock>, Arc<Mutex<Log>>) {
        let log = Arc::new(Mutex::new(Log::default()));
        let owner = ComputeOwner::spawn(Mock {
            log: log.clone(),
            pending: None,
            history: 0,
            panic_resolve: false,
            bad_resolution: false,
        })
        .unwrap();
        let handle = owner.handle();
        (owner, handle, log)
    }
    fn until(handle: &OwnerHandle<Mock>, predicate: impl Fn(Status) -> bool) -> Status {
        let deadline = Instant::now() + WATCHDOG;
        loop {
            let status = handle.status();
            if predicate(status) {
                return status;
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            assert!(
                !remaining.is_zero(),
                "owner did not reach expected state: {status:?}"
            );
            handle.wait_changed(status.epoch, remaining);
        }
    }
    fn prepared(handle: &OwnerHandle<Mock>) -> Prepared<Projection> {
        until(handle, |s| s.prepared_available);
        handle
            .try_take_prepared()
            .unwrap()
            .expect("prepared projection")
    }
    fn settled(handle: &OwnerHandle<Mock>) -> Settled<i32> {
        until(handle, |s| s.settled_available);
        handle.try_take_settled().unwrap().expect("settlement")
    }
    fn acknowledgement(
        handle: &OwnerHandle<Mock>,
        id: Id,
    ) -> OperationAck<Result<i32, &'static str>, i32> {
        until(handle, |s| {
            s.operation
                .is_some_and(|op| op.0 == id && op.2 == OperationPhase::Acknowledged)
        });
        handle
            .try_take_operation_ack(id)
            .unwrap()
            .expect("operation acknowledgement")
    }
    fn shutdown_and_join(mut owner: ComputeOwner<Mock>) {
        owner.handle.shutdown();
        until(&owner.handle, |s| s.closed);
        // No gates remain held. A native join removes the tiny closed/OS-exit gap
        // without sleep or polling; production supervision uses retained try_join.
        owner.worker.take().unwrap().join().unwrap();
        assert_eq!(owner.try_join(), JoinStatus::Joined);
    }
    fn events(log: &Arc<Mutex<Log>>) -> Vec<Event> {
        log.lock().unwrap().events.clone()
    }

    #[test]
    fn held_prepare_stop_is_immediate_and_discard_precedes_end_export() {
        let (owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(9);
        job.prepare_gate = Some(gate.clone());
        let id = handle.submit(job).unwrap();
        gate.wait_entered();
        let epoch = handle.stop_admission();
        let status = handle.status();
        assert_eq!(status.epoch, epoch);
        assert!(!status.admitting);
        assert_eq!(status.phase, Phase::Computing);
        assert_eq!(
            status.decision,
            Some(DecisionAck {
                id,
                decision: Decision::Discard
            })
        );
        assert_eq!(handle.submit(Job::new(1)), Err(OwnerError::AdmissionClosed));
        let end = handle.submit_lifecycle(Lifecycle::EndExport).unwrap();
        assert_eq!(handle.status().operation.unwrap().2, OperationPhase::Queued);
        gate.release();
        assert!(matches!(
            acknowledgement(&handle, end),
            OperationAck::Lifecycle { result: 0, .. }
        ));
        assert!(matches!(settled(&handle).result, Settlement::Discarded));
        assert_eq!(
            events(&log),
            vec![
                Event::Prepare(9),
                Event::Cancelled(true),
                Event::Discard(9),
                Event::Lifecycle("end/export", 0)
            ]
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn supersession_keeps_active_and_only_latest_successor_without_overwrite() {
        let (owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(1);
        job.prepare_gate = Some(gate.clone());
        let first = handle.submit(job).unwrap();
        gate.wait_entered();
        handle.submit(Job::new(2)).unwrap();
        let latest = handle.submit(Job::new(3)).unwrap();
        assert_eq!(handle.status().active, Some(first));
        assert_eq!(handle.status().latest, Some(latest));
        assert_eq!(handle.status().decision, None);
        gate.release();
        let projection = prepared(&handle);
        assert_eq!(projection.id, first);
        assert_eq!(projection.projection.value.get(), 1);
        let accepted = handle.resolve(first, Decision::Accept).unwrap();
        assert_eq!(handle.resolve(first, Decision::Accept).unwrap(), accepted);
        assert_eq!(
            handle.resolve(first, Decision::Discard),
            Err(OwnerError::DecisionConflict(accepted))
        );
        let status = until(&handle, |s| s.settled_available);
        assert_eq!(status.active, Some(first));
        assert_eq!(status.latest, Some(latest));
        assert_eq!(status.phase, Phase::CommittedAwaitConsumption);
        assert!(!events(&log).contains(&Event::Prepare(3)));
        assert!(matches!(settled(&handle).result, Settlement::Accepted(1)));
        let projection = prepared(&handle);
        assert_eq!(projection.id, latest);
        handle.resolve(latest, Decision::Discard).unwrap();
        settled(&handle);
        let starts: Vec<_> = events(&log)
            .into_iter()
            .filter(|e| matches!(e, Event::Prepare(_)))
            .collect();
        assert_eq!(starts, vec![Event::Prepare(1), Event::Prepare(3)]);
        shutdown_and_join(owner);
    }

    #[test]
    fn accepted_commit_survives_stop_and_settles_before_lifecycle() {
        let (owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(7);
        job.resolve_gate = Some(gate.clone());
        let id = handle.submit(job).unwrap();
        prepared(&handle);
        let accepted = handle.resolve(id, Decision::Accept).unwrap();
        gate.wait_entered();
        handle.stop_admission();
        assert_eq!(handle.status().decision, Some(accepted));
        assert_eq!(
            handle.resolve(id, Decision::Discard),
            Err(OwnerError::DecisionConflict(accepted))
        );
        let end = handle.submit_lifecycle(Lifecycle::EndExport).unwrap();
        gate.release();
        assert!(matches!(
            acknowledgement(&handle, end),
            OperationAck::Lifecycle { result: 7, .. }
        ));
        // End/export did not consume or overwrite accepted completion.
        assert_eq!(handle.status().active, Some(id));
        assert!(matches!(settled(&handle).result, Settlement::Accepted(7)));
        assert_eq!(
            events(&log),
            vec![
                Event::Prepare(7),
                Event::Cancelled(false),
                Event::Accept(7),
                Event::Lifecycle("end/export", 7)
            ]
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn accepted_feedback_error_is_terminal_once_and_survives_stop() {
        let (owner, handle, log) = fixture();
        let generation = handle.open_admission().unwrap();
        let gate = Gate::default();
        let feedback = handle
            .submit_feedback(Feedback {
                value: 11,
                gate: Some(gate.clone()),
                error: true,
                panic: false,
            })
            .unwrap();
        gate.wait_entered();
        handle.stop_admission();
        assert_eq!(
            handle.submit_lifecycle(Lifecycle::EndExport),
            Err(OwnerError::Busy)
        );
        assert_eq!(handle.open_admission(), Err(OwnerError::Busy));
        assert!(handle.try_take_operation_ack(feedback).unwrap().is_none());
        gate.release();
        assert!(matches!(
            acknowledgement(&handle, feedback),
            OperationAck::Feedback {
                result: Err("partial-source feedback error"),
                ..
            }
        ));
        assert!(matches!(
            handle.try_take_operation_ack(feedback),
            Err(OwnerError::StaleTransaction)
        ));
        let end = handle.submit_lifecycle(Lifecycle::EndExport).unwrap();
        assert!(matches!(
            acknowledgement(&handle, end),
            OperationAck::Lifecycle { result: 11, .. }
        ));
        let install = handle.submit_lifecycle(Lifecycle::Install).unwrap();
        acknowledgement(&handle, install);
        assert_eq!(handle.open_admission().unwrap(), generation + 1);
        let id = handle.submit(Job::new(2)).unwrap();
        prepared(&handle);
        handle.resolve(id, Decision::Accept).unwrap();
        assert!(matches!(settled(&handle).result, Settlement::Accepted(13)));
        let log = log.lock().unwrap();
        assert_eq!(
            log.events
                .iter()
                .filter(|e| **e == Event::Feedback(11))
                .count(),
            1
        );
        assert!(log.workers.iter().all(|worker| *worker == log.workers[0]));
        drop(log);
        assert_eq!(handle.status().high_water.workers, 1);
        shutdown_and_join(owner);
    }

    #[test]
    fn feedback_admission_discards_unresolved_compute_before_exact_once_mutation() {
        let (owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(40);
        job.prepare_gate = Some(gate.clone());
        let id = handle.submit(job).unwrap();
        gate.wait_entered();
        handle.submit(Job::new(50)).unwrap();
        let feedback = handle
            .submit_feedback(Feedback {
                value: 4,
                gate: None,
                error: false,
                panic: false,
            })
            .unwrap();
        let status = handle.status();
        assert!(status.latest.is_none());
        assert_eq!(status.decision.unwrap().decision, Decision::Discard);
        assert_eq!(
            handle.resolve(id, Decision::Accept),
            Err(OwnerError::DecisionConflict(DecisionAck {
                id,
                decision: Decision::Discard
            }))
        );
        handle.stop_admission();
        gate.release();
        assert!(matches!(
            acknowledgement(&handle, feedback),
            OperationAck::Feedback { result: Ok(4), .. }
        ));
        assert!(matches!(settled(&handle).result, Settlement::Discarded));
        assert_eq!(
            events(&log),
            vec![
                Event::Prepare(40),
                Event::Cancelled(true),
                Event::Discard(40),
                Event::Feedback(4)
            ]
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn accepted_resolution_precedes_feedback_even_when_stop_arrives() {
        let (owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(8);
        job.resolve_gate = Some(gate.clone());
        let id = handle.submit(job).unwrap();
        prepared(&handle);
        handle.resolve(id, Decision::Accept).unwrap();
        gate.wait_entered();
        let feedback = handle
            .submit_feedback(Feedback {
                value: 3,
                gate: None,
                error: false,
                panic: false,
            })
            .unwrap();
        handle.stop_admission();
        gate.release();
        assert!(matches!(
            acknowledgement(&handle, feedback),
            OperationAck::Feedback { result: Ok(11), .. }
        ));
        assert!(matches!(settled(&handle).result, Settlement::Accepted(8)));
        let ev = events(&log);
        assert!(
            ev.iter().position(|e| *e == Event::Accept(8)).unwrap()
                < ev.iter().position(|e| *e == Event::Feedback(3)).unwrap()
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn shutdown_retains_original_join_and_settles_held_work_without_replacement() {
        let (mut owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(99);
        job.prepare_gate = Some(gate.clone());
        handle.submit(job).unwrap();
        gate.wait_entered();
        let worker = owner.worker.as_ref().unwrap().thread().id();
        handle.shutdown();
        assert_eq!(owner.try_join(), JoinStatus::Pending);
        assert_eq!(owner.worker.as_ref().unwrap().thread().id(), worker);
        assert_eq!(handle.open_admission(), Err(OwnerError::Closed));
        assert_eq!(handle.submit(Job::new(1)), Err(OwnerError::Closed));
        gate.release();
        until(&handle, |s| s.closed);
        assert!(matches!(settled(&handle).result, Settlement::Discarded));
        assert_eq!(
            events(&log),
            vec![
                Event::Prepare(99),
                Event::Cancelled(true),
                Event::Discard(99)
            ]
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn shutdown_preserves_unconsumed_feedback_acknowledgement() {
        let (owner, handle, log) = fixture();
        handle.open_admission().unwrap();
        let feedback = handle
            .submit_feedback(Feedback {
                value: 6,
                gate: None,
                error: true,
                panic: false,
            })
            .unwrap();
        until(&handle, |s| {
            s.operation
                .is_some_and(|op| op.2 == OperationPhase::Acknowledged)
        });
        handle.shutdown();
        until(&handle, |s| s.closed);
        assert!(
            matches!(handle.try_take_operation_ack(feedback).unwrap(), Some(OperationAck::Feedback { id, result: Err("partial-source feedback error") }) if id == feedback)
        );
        assert_eq!(events(&log), vec![Event::Feedback(6)]);
        shutdown_and_join(owner);
    }

    #[test]
    fn panic_in_each_backend_phase_permanently_faults_and_blocks_export_reuse() {
        for phase in ["prepare", "resolve", "feedback", "lifecycle"] {
            let log = Arc::new(Mutex::new(Log::default()));
            let owner = ComputeOwner::spawn(Mock {
                log,
                pending: None,
                history: 0,
                panic_resolve: phase == "resolve",
                bad_resolution: false,
            })
            .unwrap();
            let handle = owner.handle();
            if phase == "lifecycle" {
                handle.submit_lifecycle(Lifecycle::Panic).unwrap();
            } else {
                handle.open_admission().unwrap();
                if phase == "feedback" {
                    handle
                        .submit_feedback(Feedback {
                            value: 1,
                            gate: None,
                            error: false,
                            panic: true,
                        })
                        .unwrap();
                } else {
                    let mut job = Job::new(1);
                    job.panic = phase == "prepare";
                    let id = handle.submit(job).unwrap();
                    if phase == "resolve" {
                        prepared(&handle);
                        handle.resolve(id, Decision::Accept).unwrap();
                    }
                }
            }
            let status = until(&handle, |s| s.fault.is_some());
            assert_eq!(status.fault, Some(Fault::Panic));
            assert!(!status.admitting);
            assert!(!status.prepared_available);
            assert!(!status.settled_available);
            assert_eq!(
                handle.open_admission(),
                Err(OwnerError::Faulted(Fault::Panic))
            );
            assert_eq!(
                handle.submit_lifecycle(Lifecycle::EndExport),
                Err(OwnerError::Faulted(Fault::Panic))
            );
            assert!(matches!(
                handle.try_take_settled(),
                Err(OwnerError::Faulted(Fault::Panic))
            ));
            shutdown_and_join(owner);
        }
    }

    #[test]
    fn mismatched_backend_settlement_is_a_permanent_protocol_fault() {
        let log = Arc::new(Mutex::new(Log::default()));
        let owner = ComputeOwner::spawn(Mock {
            log,
            pending: None,
            history: 0,
            panic_resolve: false,
            bad_resolution: true,
        })
        .unwrap();
        let handle = owner.handle();
        handle.open_admission().unwrap();
        let id = handle.submit(Job::new(1)).unwrap();
        prepared(&handle);
        handle.resolve(id, Decision::Accept).unwrap();
        assert_eq!(
            until(&handle, |s| s.fault.is_some()).fault,
            Some(Fault::BackendProtocol)
        );
        assert_eq!(
            handle.submit_lifecycle(Lifecycle::EndExport),
            Err(OwnerError::Faulted(Fault::BackendProtocol))
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn every_slot_high_water_is_one_and_retired_payload_drops_outside_lock() {
        let (owner, handle, _) = fixture();
        handle.open_admission().unwrap();
        let gate = Gate::default();
        let mut job = Job::new(1);
        job.prepare_gate = Some(gate.clone());
        handle.submit(job).unwrap();
        gate.wait_entered();
        let outside_lock = Arc::new(AtomicBool::new(false));
        let mut superseded = Job::new(2);
        let probe_handle = handle.clone();
        let observed = outside_lock.clone();
        superseded.drop_probe = Some(DropProbe(Box::new(move || {
            observed.store(
                probe_handle.shared.state.try_lock().is_ok(),
                Ordering::Release,
            );
        })));
        handle.submit(superseded).unwrap();
        for value in 3..100 {
            handle.submit(Job::new(value)).unwrap();
        }
        assert!(outside_lock.load(Ordering::Acquire));
        let feedback = handle
            .submit_feedback(Feedback {
                value: 1,
                gate: None,
                error: false,
                panic: false,
            })
            .unwrap();
        gate.release();
        acknowledgement(&handle, feedback);
        settled(&handle);
        assert_eq!(
            handle.status().high_water,
            HighWater {
                active: 1,
                latest: 1,
                completion: 1,
                resolution: 1,
                operation: 1,
                workers: 1
            }
        );
        shutdown_and_join(owner);
    }

    #[test]
    fn checked_sequence_and_generation_exhaustion_never_reuse_identifiers() {
        let (owner, handle, _) = fixture();
        handle.open_admission().unwrap();
        handle.shared.lock().sequence = u64::MAX;
        assert_eq!(
            handle.submit(Job::new(1)),
            Err(OwnerError::CounterExhausted)
        );
        assert!(handle.status().latest.is_none());
        handle.stop_admission();
        handle.shared.lock().generation = u64::MAX;
        assert_eq!(handle.open_admission(), Err(OwnerError::CounterExhausted));
        assert!(!handle.status().admitting);
        shutdown_and_join(owner);
    }

    #[tokio::test(flavor = "current_thread")]
    async fn changed_observes_preexisting_change_and_cancelled_waiter_keeps_ack() {
        let (owner, handle, _) = fixture();
        let epoch = handle.status().epoch;
        handle.open_admission().unwrap();
        let changed = tokio::time::timeout(WATCHDOG, handle.changed(epoch))
            .await
            .unwrap();
        assert_ne!(changed.epoch, epoch);
        let gate = Gate::default();
        let feedback = handle
            .submit_feedback(Feedback {
                value: 5,
                gate: Some(gate.clone()),
                error: true,
                panic: false,
            })
            .unwrap();
        gate.wait_entered();
        let waiting_handle = handle.clone();
        let epoch = handle.status().epoch;
        let waiter = tokio::spawn(async move { waiting_handle.changed(epoch).await });
        tokio::task::yield_now().await;
        waiter.abort();
        assert!(waiter.await.unwrap_err().is_cancelled());
        handle.stop_admission();
        gate.release();
        loop {
            let status = handle.status();
            if status
                .operation
                .is_some_and(|op| op.2 == OperationPhase::Acknowledged)
            {
                break;
            }
            tokio::time::timeout(WATCHDOG, handle.changed(status.epoch))
                .await
                .unwrap();
        }
        assert!(matches!(
            handle.try_take_operation_ack(feedback).unwrap(),
            Some(OperationAck::Feedback { result: Err(_), .. })
        ));
        shutdown_and_join(owner);
    }
}
