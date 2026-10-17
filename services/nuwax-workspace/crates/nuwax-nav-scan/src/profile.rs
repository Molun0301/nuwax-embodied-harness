//! 仅诊断特性启用的阶段计时；不参与候选选择或运动许可。
use crate::{BodyPose, ScanConfig};
use nuwax_nav_types_messages::MotionKinematics;
use std::{cell::RefCell, time::Instant};

#[derive(Default)]
struct Counters {
    active: bool,
    calls: [u64; 3],
    failures: [u64; 3],
    milliseconds: [f64; 3],
    success: bool,
}
thread_local! {static COUNTERS:RefCell<Counters>=RefCell::new(Counters::default());}
pub(crate) fn record(stage: usize, began: Instant, failed: bool) {
    let elapsed = began.elapsed().as_secs_f64() * 1000.0;
    COUNTERS.with(|c| {
        let mut c = c.borrow_mut();
        if c.active {
            c.calls[stage] += 1;
            c.failures[stage] += u64::from(failed);
            c.milliseconds[stage] += elapsed;
        }
    });
}
pub(crate) fn success() {
    COUNTERS.with(|c| c.borrow_mut().success = true);
}
pub(crate) struct Scope {
    began: Instant,
    input: serde_json::Value,
}
impl Scope {
    pub(crate) fn begin(
        start: BodyPose,
        reference: &[[f64; 3]],
        motion: &MotionKinematics,
        config: &ScanConfig,
        final_yaw: Option<f64>,
        arrival: Option<([f64; 3], f64)>,
        follow_reference: bool,
        prefer_straight: bool,
    ) -> Self {
        #[cfg(any(test,feature="local-guidance-qualification"))] PHASE_RECORDS.with(|phases| phases.borrow_mut().clear());
        COUNTERS.with(|c| {
            *c.borrow_mut() = Counters {
                active: true,
                ..Default::default()
            }
        });
        Self {
            began: Instant::now(),
            input: serde_json::json!({"start":start,"reference":reference,
            "motion":{"forward":motion.max_forward_mps,"angular":motion.max_angular_rps,
                "radius":motion.minimum_turning_radius_m,"can_rotate":motion.can_rotate_in_place},
            "config":{"horizon":config.horizon_m,"iterations":config.optimization_iterations,"corridor":config.reference_corridor_m},
            "final_yaw":final_yaw,"arrival":arrival,"follow_reference":follow_reference,"prefer_straight":prefer_straight}),
        }
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        COUNTERS.with(|c| {
            let mut c=c.borrow_mut();c.active=false;
            tracing::info!(target:"navigation.scan.profile",event="navigation.scan.profile",
                elapsed_ms=self.began.elapsed().as_secs_f64()*1000.0,input=%self.input,success=c.success,
                view_calls=c.calls[0],view_ms=c.milliseconds[0],view_failures=c.failures[0],
                project_calls=c.calls[1],project_ms=c.milliseconds[1],project_failures=c.failures[1],
                sweep_calls=c.calls[2],sweep_ms=c.milliseconds[2],sweep_failures=c.failures[2],
                "近场规划阶段诊断");
        });
    }
}

#[cfg(test)]
pub(crate) fn calls() -> [u64; 3] { COUNTERS.with(|c| c.borrow().calls) }


// Opt-in observations only. These scopes partition the existing call; they never
// replay a projection, grant clearance, or select a candidate.
#[derive(Clone, Copy, Default)]
struct Snapshot {
    calls: [u64; 3],
    milliseconds: [f64; 3],
    success: bool,
}
fn snapshot() -> Snapshot {
    COUNTERS.with(|c| {
        let c = c.borrow();
        Snapshot { calls: c.calls, milliseconds: c.milliseconds, success: c.success }
    })
}
#[cfg(any(test,feature="local-guidance-qualification"))]
#[derive(Clone, Debug)]
pub struct PhaseRecord {
    pub phase: &'static str,
    pub exit: &'static str,
    pub selection: &'static str,
    pub complete: bool,
    pub elapsed_ms: f64,
    pub calls: [u64; 3],
    pub milliseconds: [f64; 3],
}
#[cfg(any(test,feature="local-guidance-qualification"))]
thread_local! { static PHASE_RECORDS: RefCell<Vec<PhaseRecord>> = const { RefCell::new(Vec::new()) }; }
#[cfg(any(test,feature="local-guidance-qualification"))]
pub(crate) fn take_phases() -> Vec<PhaseRecord> {
    PHASE_RECORDS.with(|phases| std::mem::take(&mut *phases.borrow_mut()))
}
pub(crate) struct Phase {
    phase: &'static str,
    exit: &'static str,
    selection: &'static str,
    began: Instant,
    before: Snapshot,
}
impl Phase {
    pub(crate) fn begin(phase: &'static str) -> Self {
        Self { phase, exit: "return_or_error", selection: "none",
            began: Instant::now(), before: snapshot() }
    }
    pub(crate) fn next(&mut self, phase: &'static str) {
        self.exit = "continued";
        self.emit();
        self.phase = phase;
        self.exit = "return_or_error";
        self.selection = "none";
        self.before = snapshot();
        self.began = Instant::now();
    }
    pub(crate) fn finish(&mut self, exit: &'static str) { self.exit = exit; }
    pub(crate) fn selected(&mut self, selection: &'static str) { self.selection = selection; }
    fn emit(&self) {
        let elapsed_ms = self.began.elapsed().as_secs_f64() * 1000.0;
        let after = snapshot();
        let calls = std::array::from_fn::<_,3,_>(|i| after.calls[i] - self.before.calls[i]);
        let milliseconds = std::array::from_fn::<_,3,_>(|i| after.milliseconds[i] - self.before.milliseconds[i]);
        let complete = self.exit == "full" || after.success && !self.before.success;
        tracing::info!(target: "navigation.scan.phase", event="navigation.scan.phase",
            phase=self.phase, phase_exit=self.exit, selection=self.selection, complete,
            elapsed_ms, view_calls=calls[0], project_calls=calls[1], sweep_calls=calls[2],
            view_ms=milliseconds[0], project_ms=milliseconds[1], sweep_ms=milliseconds[2],
            "existing Scan phase observations");
        #[cfg(any(test,feature="local-guidance-qualification"))] PHASE_RECORDS.with(|phases| phases.borrow_mut().push(PhaseRecord {
            phase: self.phase, exit: self.exit, selection: self.selection, complete,
            elapsed_ms, calls, milliseconds,
        }));
    }
}
impl Drop for Phase { fn drop(&mut self) { self.emit(); } }
