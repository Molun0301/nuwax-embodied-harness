//! Opt-in, test-only compute-owner wall-time attribution.
//!
//! Register this module with `#[cfg(test)]`. All storage is fixed-size and
//! thread-local; only the thread calling `begin` records stages. Nested stages
//! are inclusive and MUST NOT be summed as disjoint control cost. This measures
//! elapsed `Instant` time, including scheduling/preemption, never thread CPU.
//! Stage entry/drop performs no formatting, allocation, hashing or locking.

use std::cell::RefCell;
use std::marker::PhantomData;
use std::rc::Rc;
use std::time::Instant;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(usize)]
pub(crate) enum Stage {
    RouteCurrentBody,
    CurrentViewRefresh,
    CurrentBodyValidation,
    RouteProgress,
    PlanConsumeAdmission,
    PlanSchedule,
    ControllerStep,
    RouteIntent,
    GuardSetup,
    CandidateRankingSweeps,
    GuardCleanup,
}

impl Stage {
    pub(crate) const COUNT: usize = 11;
    pub(crate) const ALL: [Self; Self::COUNT] = [
        Self::RouteCurrentBody,
        Self::CurrentViewRefresh,
        Self::CurrentBodyValidation,
        Self::RouteProgress,
        Self::PlanConsumeAdmission,
        Self::PlanSchedule,
        Self::ControllerStep,
        Self::RouteIntent,
        Self::GuardSetup,
        Self::CandidateRankingSweeps,
        Self::GuardCleanup,
    ];

    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::RouteCurrentBody => "route_current_body",
            Self::CurrentViewRefresh => "current_view_refresh",
            Self::CurrentBodyValidation => "current_body_validation",
            Self::RouteProgress => "route_progress",
            Self::PlanConsumeAdmission => "plan_consume_admission",
            Self::PlanSchedule => "plan_schedule",
            Self::ControllerStep => "controller_step",
            Self::RouteIntent => "route_intent",
            Self::GuardSetup => "guard_setup",
            Self::CandidateRankingSweeps => "candidate_ranking_sweeps",
            Self::GuardCleanup => "guard_cleanup",
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Record {
    /// Inclusive per-stage wall duration, accumulated across repeated calls.
    pub(crate) wall_nanos: [u64; Stage::COUNT],
    pub(crate) calls: [u32; Stage::COUNT],
}

impl Record {
    const EMPTY: Self = Self {
        wall_nanos: [0; Stage::COUNT],
        calls: [0; Stage::COUNT],
    };
}

struct State {
    enabled: bool,
    epoch: u64,
    record: Record,
}

thread_local! {
    static PROFILE: RefCell<State> = const { RefCell::new(State {
        enabled: false,
        epoch: 0,
        record: Record::EMPTY,
    }) };
}

pub(crate) fn begin() {
    PROFILE.with(|profile| {
        let mut state = profile.borrow_mut();
        assert!(!state.enabled, "control wall profile already active on this thread");
        state.epoch = state.epoch.wrapping_add(1);
        state.record = Record::EMPTY;
        state.enabled = true;
    });
}

/// Call only after all stage guards for the step have been dropped.
pub(crate) fn end() -> Record {
    PROFILE.with(|profile| {
        let mut state = profile.borrow_mut();
        assert!(state.enabled, "control wall profile is not active on this thread");
        state.enabled = false;
        std::mem::take(&mut state.record)
    })
}

#[must_use = "hold the guard for the complete stage, including intended cleanup"]
pub(crate) struct Guard {
    stage: Stage,
    started: Option<(Instant, u64)>,
    // A timer must be dropped on its original compute-owner thread. Rc is only
    // a zero-sized marker here; no Rc allocation or reference count is created.
    _same_thread: PhantomData<Rc<()>>,
}

pub(crate) fn guard(stage: Stage) -> Guard {
    let epoch = PROFILE.with(|profile| {
        let state = profile.borrow();
        state.enabled.then_some(state.epoch)
    });
    Guard {
        stage,
        started: epoch.map(|epoch| (Instant::now(), epoch)),
        _same_thread: PhantomData,
    }
}

impl Drop for Guard {
    fn drop(&mut self) {
        let Some((started, epoch)) = self.started else { return; };
        let nanos = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
        PROFILE.with(|profile| {
            let mut state = profile.borrow_mut();
            // A guard retained past end must not contaminate a later step.
            if state.enabled && state.epoch == epoch {
                let index = self.stage as usize;
                state.record.wall_nanos[index] = state.record.wall_nanos[index].saturating_add(nanos);
                state.record.calls[index] = state.record.calls[index].saturating_add(1);
            }
        });
    }
}

#[test]
fn disabled_guards_and_prior_steps_do_not_leak() {
    let disabled = guard(Stage::RouteProgress);
    begin();
    drop(disabled);
    assert_eq!(end().calls, [0; Stage::COUNT]);

    begin();
    let stale = guard(Stage::RouteProgress);
    assert_eq!(end().calls, [0; Stage::COUNT]);
    begin();
    drop(stale);
    assert_eq!(end().calls, [0; Stage::COUNT]);
}

#[test]
fn nested_stages_are_inclusive_and_repeated_calls_accumulate() {
    begin();
    {
        let _outer = guard(Stage::ControllerStep);
        for _ in 0..2 {
            let _inner = guard(Stage::GuardSetup);
        }
    }
    let record = end();
    assert_eq!(record.calls[Stage::ControllerStep as usize], 1);
    assert_eq!(record.calls[Stage::GuardSetup as usize], 2);
    assert!(record.wall_nanos[Stage::ControllerStep as usize]
        >= record.wall_nanos[Stage::GuardSetup as usize]);
}

#[test]
fn stages_on_other_threads_are_not_recorded() {
    begin();
    std::thread::spawn(|| { let _guard = guard(Stage::PlanSchedule); }).join().unwrap();
    assert_eq!(end().calls, [0; Stage::COUNT]);
}
