//! Test-only observations; never a planning decision or reusable permission.
use nuwax_nav_pct::SearchStats;
use nuwax_nav_scan::{LocalPlanFailureKind, LocalTrajectory};
use std::{
    cell::RefCell,
    time::{Duration, Instant},
};

#[derive(Clone, Debug, Default)]
pub(crate) struct SearchWork {
    pub branch: usize,
    pub expanded: u64,
    pub candidates: u64,
    pub sweeps: u64,
    pub nodes: u64,
    pub frontier_peak: u64,
}
#[derive(Clone, Debug)]
pub(crate) struct PctWork {
    pub branch: usize,
    pub wall: Duration,
    pub stats: Option<SearchStats>,
}
#[derive(Clone, Debug)]
pub(crate) struct Branch {
    pub detour: bool,
    pub follow_reference: bool,
    pub straight: bool,
    pub reference_changed: bool,
    pub reference: Vec<[f64; 3]>,
    pub cached_negative: bool,
    pub wall: Duration,
    pub partial: Option<bool>,
    pub failure: Option<LocalPlanFailureKind>,
}
#[derive(Clone, Debug, Default)]
pub(crate) struct Frame {
    pub branches: Vec<Branch>,
    // No derived reference was recomputed for these exact input cache hits.
    pub reused_failures: Vec<(bool, LocalPlanFailureKind)>,
    pub pct: Vec<PctWork>,
    pub searches: Vec<SearchWork>,
    pub profile_events: u64,
    pub profile_calls: [u64; 3],
    pub profile_wall_ms: [f64; 3],
}
#[derive(Clone, Debug)]
pub(crate) struct Admission {
    pub at: Instant,
    pub request_id: u64,
    pub partial: bool,
    pub covered_end: f64,
    pub requested_end: f64,
}
#[derive(Default)]
struct State {
    frame: Option<Frame>,
    admissions: Option<Vec<Admission>>,
}
thread_local! { static STATE: RefCell<State> = RefCell::new(State::default()); }

pub(crate) fn begin() {
    STATE.with(|s| {
        let mut s = s.borrow_mut();
        assert!(s.frame.is_none());
        s.frame = Some(Frame::default());
    });
}
pub(crate) fn end() -> Frame {
    STATE.with(|s| s.borrow_mut().frame.take().expect("active priority probe"))
}
pub(crate) fn begin_control() {
    STATE.with(|s| {
        let mut s = s.borrow_mut();
        assert!(s.admissions.is_none());
        s.admissions = Some(Vec::new());
    });
}
pub(crate) fn end_control() -> Vec<Admission> {
    STATE.with(|s| {
        s.borrow_mut()
            .admissions
            .take()
            .expect("active admission probe")
    })
}
pub(crate) fn clock() -> Option<Instant> {
    STATE.with(|s| s.borrow().frame.as_ref().map(|_| Instant::now()))
}
pub(crate) fn pct(at: Option<Instant>, stats: Option<SearchStats>) {
    if let Some(at) = at {
        STATE.with(|s| {
            if let Some(f) = s.borrow_mut().frame.as_mut() {
                f.pct.push(PctWork {
                    branch: f.branches.len(),
                    wall: at.elapsed(),
                    stats,
                });
            }
        });
    }
}
pub(crate) fn reused_failure(detour: bool, kind: LocalPlanFailureKind) {
    STATE.with(|s| {
        if let Some(f)=s.borrow_mut().frame.as_mut() {f.reused_failures.push((detour,kind));}
    });
}
pub(crate) fn branch_begin(
    detour: bool,
    follow_reference: bool,
    straight: bool,
    original: &[[f64; 3]],
    reference: &[[f64; 3]],
    cached_negative: bool,
) -> Option<Instant> {
    STATE.with(|s| {
        let mut s = s.borrow_mut();
        let f = s.frame.as_mut()?;
        f.branches.push(Branch {
            detour,
            follow_reference,
            straight,
            reference_changed: !super::exact_points(original, reference),
            reference: reference.to_vec(),
            cached_negative,
            wall: Duration::ZERO,
            partial: None,
            failure: None,
        });
        Some(Instant::now())
    })
}
pub(crate) fn branch_end(
    at: Option<Instant>,
    result: &Result<LocalTrajectory, nuwax_nav_scan::LocalPlanError>,
) {
    if let Some(at) = at {
        STATE.with(|s| {
            if let Some(f) = s.borrow_mut().frame.as_mut() {
                let branch = f.branches.last_mut().unwrap();
                branch.wall = at.elapsed();
                match result {
                    Ok(path) => branch.partial = Some(path.partial),
                    Err(e) => branch.failure = Some(e.kind),
                }
            }
        });
    }
}

// Uses only existing tracing events, on the worker which actually computes.
// No new dependency and no work estimates inferred from candidate counts.
pub(crate) struct Subscriber;
#[derive(Default)]
struct Fields {
    work: SearchWork,
    event: Option<String>,
    request_id: Option<u64>,
    partial: bool,
    covered_end: f64,
    requested_end: f64,
    calls: [u64; 3],
    milliseconds: [f64; 3],
}
impl tracing::field::Visit for Fields {
    fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
    fn record_str(&mut self, f: &tracing::field::Field, v: &str) {
        if f.name() == "event" {
            self.event = Some(v.to_owned());
        }
    }
    fn record_bool(&mut self, f: &tracing::field::Field, v: bool) {
        if f.name() == "partial" {
            self.partial = v;
        }
    }
    fn record_u64(&mut self, f: &tracing::field::Field, v: u64) {
        match f.name() {
            "expanded" => self.work.expanded = v,
            "candidates" => self.work.candidates = v,
            "sweeps" => self.work.sweeps = v,
            "nodes" => self.work.nodes = v,
            "frontier_peak" => self.work.frontier_peak = v,
            "request_id" => self.request_id = Some(v),
            "view_calls" => self.calls[0] = v,
            "project_calls" => self.calls[1] = v,
            "sweep_calls" => self.calls[2] = v,
            _ => {}
        }
    }
    fn record_f64(&mut self, f: &tracing::field::Field, v: f64) {
        match f.name() {
            "covered_end" => self.covered_end = v,
            "requested_end" => self.requested_end = v,
            "view_ms" => self.milliseconds[0] = v,
            "project_ms" => self.milliseconds[1] = v,
            "sweep_ms" => self.milliseconds[2] = v,
            _ => {}
        }
    }
}
impl tracing::Subscriber for Subscriber {
    fn enabled(&self, m: &tracing::Metadata<'_>) -> bool {
        matches!(
            m.target(),
            "navigation.scan.search" | "navigation.scan.profile" | "navigation.scan"
        )
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        let mut fields = Fields::default();
        event.record(&mut fields);
        STATE.with(|s| {
            let mut s = s.borrow_mut();
            if let Some(f) = s.frame.as_mut() {
                match event.metadata().target() {
                    "navigation.scan.search" => {
                        fields.work.branch = f.branches.len().saturating_sub(1);
                        f.searches.push(fields.work);
                    }
                    "navigation.scan.profile" => {
                        f.profile_events += 1;
                        for i in 0..3 {
                            f.profile_calls[i] += fields.calls[i];
                            f.profile_wall_ms[i] += fields.milliseconds[i];
                        }
                    }
                    _ => {}
                }
            }
            if fields.event.as_deref() == Some("navigation.scan.candidate_admitted") {
                if let (Some(admissions), Some(request_id)) =
                    (s.admissions.as_mut(), fields.request_id)
                {
                    admissions.push(Admission {
                        at: Instant::now(),
                        request_id,
                        partial: fields.partial,
                        covered_end: fields.covered_end,
                        requested_end: fields.requested_end,
                    });
                }
            }
        });
    }
}
