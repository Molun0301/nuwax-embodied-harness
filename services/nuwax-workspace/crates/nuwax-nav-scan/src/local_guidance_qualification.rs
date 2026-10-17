//! Diagnostic-only access to the existing phase records and normal Scan outputs.
use crate::{BodyPose, ScanConfig};
use nuwax_nav_mls_planner::SurfaceSnapshot;
pub use crate::profile::PhaseRecord;

/// Expanded states, generated candidates, admission sweeps, nodes, frontier peak.
#[derive(Clone, Debug, Default)]
pub struct Observation {
    pub phases: Vec<PhaseRecord>,
    pub work: [usize; 5],
}
thread_local! { static WORK: std::cell::Cell<[usize;5]> = const { std::cell::Cell::new([0;5]) }; }
pub(crate) fn record_work(work:[usize;5]) { WORK.set(work); }
/// Called on the same CPU owner immediately before the existing Scan call.
pub fn reset() { WORK.set([0;5]); let _=crate::profile::take_phases(); }
/// Called on that same owner after Scan returns, including errors/cancellation.
pub fn take() -> Observation {
    Observation { phases:crate::profile::take_phases(), work:WORK.get() }
}
/// Reuse the existing full intermediate-window handoff predicate for an audit.
/// Partial candidates do not claim this contract and must remain labelled partial.
pub fn full_handoff(map:&SurfaceSnapshot,start:BodyPose,end:BodyPose,
    reference:&[[f64;3]],config:&ScanConfig)->bool {
    let Some(model)=map.body_envelope() else {return false;};
    let length=reference.windows(2).map(|p|(p[1][0]-p[0][0]).hypot(p[1][1]-p[0][1])).sum();
    let tangent=reference.windows(2).rfind(|p|(p[1][0]-p[0][0]).hypot(p[1][1]-p[0][1])>1e-9)
        .map_or(crate::yaw(start.1),|p|(p[1][1]-p[0][1]).atan2(p[1][0]-p[0][0]));
    crate::local_search::reference_handoff(end,reference,model.body_to_support_z_m,
        (config.reference_corridor_m*0.9).min(config.horizon_m*0.3),
        (model.body_to_support_z_m+model.z_bounds_m[1])*0.5,length,tangent)
}
