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
