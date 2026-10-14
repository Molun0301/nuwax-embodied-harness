//! PCT reference geometry, refined by the existing forward-motion primitives.
//! Missing support is not a gate; a reference alone is never an executable route.
use super::*;
use std::sync::{atomic::AtomicBool, Arc};

/// Keep reference resource failures and bounded motion refinement distinct from
/// physical/input failures, without parsing the compatibility String API.
#[derive(Debug)]
pub enum ReferencePlanningFailure {
    Search(nuwax_nav_pct::PlanningFailure),
    RefinementIncomplete(String),
    Invalid(String),
}
impl ReferencePlanningFailure {
    pub fn is_inconclusive(&self) -> bool {
        matches!(
            self,
            Self::RefinementIncomplete(_)
                | Self::Search(
                    nuwax_nav_pct::PlanningFailure::BudgetExhausted(_)
                        | nuwax_nav_pct::PlanningFailure::SearchDomainLimited(_)
                )
        )
    }
}
impl From<String> for ReferencePlanningFailure {
    fn from(message: String) -> Self {
        Self::Invalid(message)
    }
}
impl From<&str> for ReferencePlanningFailure {
    fn from(message: &str) -> Self {
        Self::Invalid(message.into())
    }
}
impl std::fmt::Display for ReferencePlanningFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Search(error) => std::fmt::Display::fmt(error, f),
            Self::RefinementIncomplete(message) | Self::Invalid(message) => f.write_str(message),
        }
    }
}

/// Bind the existing compute owner's flag to PCT construction and query checks.
/// This shares the original flag and restores the prior thread-local scope.
pub fn with_owner_cancellation<T>(flag: Arc<AtomicBool>, operation: impl FnOnce() -> T) -> T {
    nuwax_nav_pct::with_planning_cancellation(
        &nuwax_nav_pct::PlanningCancellation::from_flag(flag),
        operation,
    )
}

/// Derive a reference with the caller's existing PCT cache, then validate every
/// actual body-motion segment with the existing forward/biarc primitives.
/// Output length and endpoint describe that refined route, not the yaw-free guide.
#[allow(clippy::too_many_arguments)]
pub fn plan_reference_cancellable(
    map: &SurfaceSnapshot,
    planner: &nuwax_nav_pct::PctPlanner,
    start: BodyPose,
    goal: BodyPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    boundary: &[[f64; 2]],
    budget: usize,
    cancelled: Option<&AtomicBool>,
) -> Result<Vec<BodyPose>, String> {
    plan_reference_typed(
        map, planner, start, goal, options, limits, boundary, budget, cancelled,
    )
    .map_err(|error| error.to_string())
}

#[allow(clippy::too_many_arguments)]
pub fn plan_reference_typed(
    map: &SurfaceSnapshot,
    planner: &nuwax_nav_pct::PctPlanner,
    start: BodyPose,
    goal: BodyPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    boundary: &[[f64; 2]],
    budget: usize,
    cancelled: Option<&AtomicBool>,
) -> Result<Vec<BodyPose>, ReferencePlanningFailure> {
    let stopped = || {
        cancelled.is_some_and(|flag| flag.load(std::sync::atomic::Ordering::Acquire))
            || nuwax_nav_pct::planning_cancelled()
    };
    let check = || {
        if stopped() {
            Err("三维规划已取消".to_string())
        } else {
            Ok(())
        }
    };
    // Keep the public mission query's validation-error precedence: invalid
    // actuator limits remain a real input error even if cancellation also arrives.
    if let Some(limits) = limits {
        limits.validate()?;
    }
    options.validate()?;
    check()?;
    let motion = limits
        .map(MotionLimits::kinematics)
        .map(nuwax_nav_runtime::surface_follower::forward_navigation_kinematics);
    let options = GoalOptions {
        allow_reverse: false,
        ..options
    };
    map.body_envelope().ok_or("三维完整机体标定缺失")?;
    map.validate_body_pose(start.0, start.1)
        .map_err(|error| format!("起点机体包络: {error}"))?;
    let reference = planner
        .body_route_with_limits_typed(
            map,
            start,
            goal,
            options,
            boundary,
            cancelled,
            nuwax_nav_pct::SearchLimits { max_states: budget },
        )
        .map_err(ReferencePlanningFailure::Search)?;
    check()?;
    let mut path = vec![start];
    // A zero-distance reference still has to satisfy requested final yaw. Never
    // append a nominal endpoint or rotate a non-rotating platform by assignment.
    let targets = if reference.len() == 1 {
        &reference[..]
    } else {
        &reference[1..]
    };
    // Long clear PCT shortcuts are still refined by the bounded primitive.
    // Split at its existing half-source-voxel sample scale; keep the action cap.
    let max_span = f64::from(map.source_voxel_size_m()) * 0.5 * (MAX_ACTION_SAMPLES - 1) as f64;
    for (index, target) in targets.iter().enumerate() {
        check()?;
        let origin = *path.last().expect("refinement retains its start");
        let length = (target.0[0] - origin.0[0]).hypot(target.0[1] - origin.0[1]);
        let pieces = (length / max_span).ceil().max(1.0) as usize;
        for piece in 1..=pieces {
            check()?;
            let last = index + 1 == targets.len() && piece == pieces;
            let fraction = piece as f64 / pieces as f64;
            let target = (
                std::array::from_fn(|axis| {
                    origin.0[axis] + fraction * (target.0[axis] - origin.0[axis])
                }),
                if last && options.requires_orientation() {
                    goal.1
                } else {
                    target.1
                },
            );
            let from = *path.last().expect("refinement retains its start");
            let segment = connect_forward_motion(
                map,
                from,
                target,
                motion.as_ref(),
                boundary,
                last && options.requires_orientation(),
                &stopped,
            );
            check()?;
            let segment = segment.ok_or_else(|| ReferencePlanningFailure::RefinementIncomplete(
                "ResourceLimited: PCT参考尚未完成满足运动约束的机体细化；未证明无路".into()))?;
            path.extend(segment.into_iter().skip(1));
        }
    }
    check()?;
    let end = *path.last().expect("refinement retains its start");
    if position_distance(end, goal) > options.position_tolerance_m + 1e-9
        || options.requires_orientation()
            && angle(planar(end).yaw - planar(goal).yaw).abs() > options.yaw_tolerance_rad + 1e-9
    {
        return Err(ReferencePlanningFailure::RefinementIncomplete(
            "ResourceLimited: 实际机体接续未进入原目标位置或朝向容差；未证明无路".into(),
        ));
    }
    Ok(path)
}

#[cfg(test)]
#[path = "surface_oriented_reference_tests.rs"]
mod tests;
