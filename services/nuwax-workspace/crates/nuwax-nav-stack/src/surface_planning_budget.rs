//! Resource progression belongs to the existing planning owner's failed input.
//! It is not a terrain verdict, a second job owner, or a new request deadline.
use super::*;

pub(super) const INITIAL_FULL_SEARCH_STATES: usize = 30_000;
pub(super) fn initial_budget(config: &SurfaceNavigationConfig) -> usize {
    INITIAL_FULL_SEARCH_STATES.min(config.follower.pct.search_limits().max_states)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FailureKind {
    Input,
    BudgetExhausted { attempted: usize },
    SearchDomainLimited,
}
pub(super) struct FailedPlanningInput {
    source: Arc<SurfaceSnapshot>,
    start: Pose3,
    kind: FailureKind,
}
impl FailedPlanningInput {
    pub(super) fn new(
        source: Arc<SurfaceSnapshot>,
        start: Pose3,
        attempted: usize,
        error: &NavStackError,
    ) -> Self {
        let kind = match error {
            NavStackError::Planning(nuwax_nav_pct::PlanningFailure::BudgetExhausted(_)) => {
                FailureKind::BudgetExhausted { attempted }
            }
            NavStackError::Planning(nuwax_nav_pct::PlanningFailure::SearchDomainLimited(_)) => {
                FailureKind::SearchDomainLimited
            }
            _ => FailureKind::Input,
        };
        Self {
            source,
            start,
            kind,
        }
    }
    pub(super) fn resource_limited(&self) -> bool {
        matches!(self.kind, FailureKind::BudgetExhausted { .. })
    }
    pub(super) fn domain_limited(&self) -> bool {
        matches!(self.kind, FailureKind::SearchDomainLimited)
    }
    pub(super) fn can_increase(&self, config: &SurfaceNavigationConfig) -> bool {
        matches!(self.kind, FailureKind::BudgetExhausted { attempted }
            if attempted < config.follower.pct.search_limits().max_states)
    }
    pub(super) fn next_budget(
        &self,
        current: &SurfaceSnapshot,
        start: Pose3,
        config: &SurfaceNavigationConfig,
    ) -> Option<usize> {
        if !nuwax_nav_pct::same_source_geometry(&self.source, current) {
            return Some(initial_budget(config));
        }
        match self.kind {
            FailureKind::BudgetExhausted { attempted } => {
                let maximum = config.follower.pct.search_limits().max_states;
                if attempted < maximum || meaningful_start_change(self.start, start, config) {
                    // A moved start retries at the ceiling; it cannot reset the
                    // same goal/geometry to the already exhausted smaller budget.
                    Some(maximum)
                } else {
                    None
                }
            }
            FailureKind::SearchDomainLimited => {
                // More search work cannot inspect unmaterialized geometry.
                meaningful_start_change(self.start, start, config).then(|| initial_budget(config))
            }
            FailureKind::Input => {
                // Preserve the previous non-resource failure memo semantics.
                let same_observations = match (self.source.observations(), current.observations()) {
                    (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                    (None, None) => true,
                    _ => false,
                };
                (!same_observations || self.start != start).then(|| initial_budget(config))
            }
        }
    }
}
fn meaningful_start_change(
    previous: Pose3,
    current: Pose3,
    config: &SurfaceNavigationConfig,
) -> bool {
    if previous.position.distance(current.position) >= config.follower.scan.refresh_distance_m {
        return true;
    }
    // Reuse the existing orientation tolerance, including tilt, with quaternion
    // sign/scaling invariance. This only schedules a new attempt, not motion.
    let values = |q: Quaternion| [q.x, q.y, q.z, q.w];
    let a = values(previous.orientation);
    let b = values(current.orientation);
    let norm = |q: [f64; 4]| q.into_iter().fold(0.0_f64, f64::hypot);
    let an = norm(a);
    let bn = norm(b);
    if !an.is_finite() || !bn.is_finite() || an <= 0.0 || bn <= 0.0 {
        return previous != current;
    }
    let dot = a
        .into_iter()
        .zip(b)
        .map(|(a, b)| (a / an) * (b / bn))
        .sum::<f64>()
        .abs()
        .clamp(0.0, 1.0);
    2.0 * dot.acos() >= config.follower.controller.orientation_tolerance
}
