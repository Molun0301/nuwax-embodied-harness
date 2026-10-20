//! Source-owned pre-stop terminal evidence. This module is nested under the
//! SurfaceFollower owner so only that owner can construct a terminal payload.
use std::sync::Arc;
use nuwax_nav_mls_planner::surface::{BodyGroundEvidence, BodyGroundSourceRelation};
use nuwax_nav_types_messages::{
    assess_goal_region, ArrivalBoundary, ArrivalGeometry, GoalOptions,
    GoalRegionAssessment, GroundHeightIntent, GroundLayerResult,
};
use crate::{GroundExecutionGoal, GroundInitialCapture, RuntimeError};
use crate::replan_feedback::{ActualExecutionContext, GroundIntent};

/// Opaque evidence that the unique execution owner actually consumed the
/// original Ground terminal. It is not a stopped result or motion authority.
/// No public constructor permits a caller to manufacture a matched layer.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundArrivalEvidence {
    goal: GroundExecutionGoal,
    target_yaw: f64,
    intent: GroundIntent,
    // This captured actual stays at the committed terminal A. Requalification
    // at B must not silently turn an unconsumed navigation address into actual.
    actual: Arc<ActualExecutionContext>,
    navigation_evidence: BodyGroundEvidence,
    relation: GroundLayerResult,
}
impl GroundArrivalEvidence {
    /// Only the Surface owner descendants can enter this constructor, after the
    /// current generation/binding terminal gate. Recheck the private consumed
    /// address and immutable original target here as well.
    pub(super) fn from_consumed(goal: GroundExecutionGoal, target_yaw: f64, intent: GroundIntent,
        actual: Arc<ActualExecutionContext>, navigation_evidence: BodyGroundEvidence,
        options: GoalOptions) -> Result<Option<Self>, String> {
        let basis = actual.navigation.as_ref().ok_or("Ground arrival has no consumed basis")?;
        if !goal.matches_reference(intent.window().owner())
            || actual.odometry.frame_id != goal.frame_id {
            return Err("Ground arrival does not consume the original target terminal".into());
        }
        if !basis.is_terminal_for(&intent) { return Ok(None); }
        let relation = actual.ground_evidence.as_ref()
            .map_or(GroundLayerResult::Unresolved, |evidence| evidence.relation);
        let event = Self { goal, target_yaw, intent, actual, navigation_evidence, relation };
        // An incomplete layer is still a terminal event. Stop and a fresh actual
        // sample decide the terminal result; it must not become a traversal gate.
        if event.region(&event.actual, options, ArrivalBoundary::Strict, relation)
            == GoalRegionAssessment::OutsideRegion { return Ok(None); }
        Ok(Some(event))
    }
    /// Original typed target, never derived from body Z or resolved height.
    pub fn goal(&self) -> &GroundExecutionGoal { &self.goal }
    /// Original finite terminal intent retained across stop.
    pub(crate) fn intent(&self) -> &GroundIntent { &self.intent }
    /// Actual committed input from before the original fence.
    pub(crate) fn actual(&self) -> &Arc<ActualExecutionContext> { &self.actual }
    /// Frozen current owned basis becomes a legal Initial for real drift
    /// consumption and connector admission; this never restores an old command.
    pub(crate) fn initial_capture(&self) -> Result<GroundInitialCapture, RuntimeError> {
        let state = self.actual.navigation.as_ref()
            .ok_or_else(||RuntimeError::Planner("Ground arrival lost its actual basis".into()))?
            .continuation().map_err(RuntimeError::Planner)?;
        GroundInitialCapture::from_actual(state, Arc::clone(&self.actual))
    }
    fn region(&self, current: &ActualExecutionContext, options: GoalOptions,
        boundary: ArrivalBoundary, relation: GroundLayerResult) -> GoalRegionAssessment {
        let pose = &current.odometry.pose;
        assess_goal_region(&ArrivalGeometry::Ground {
            actual_xy: [pose.position.x, pose.position.y], target_xy: self.goal.xy,
            actual_yaw: pose.orientation.yaw(), target_yaw: self.target_yaw,
            height: GroundHeightIntent { requested_ground_z: self.goal.requested_ground_z,
                resolved_ground_z: self.intent.resolved_ground_z() },
        }, &options, boundary, relation)
    }
    /// Requalify the retained terminal against a current paired observation.
    /// Call outside worker input locks. This does one source-only selection at
    /// actual support XY; it runs no walk, full sweep, planner or cursor commit.
    pub(crate) fn assess(&self, current: &ActualExecutionContext, options: GoalOptions,
        boundary: ArrivalBoundary, source_retired: bool) -> GoalRegionAssessment {
        if current.odometry.frame_id != self.goal.frame_id
            || crate::validate_map_pose(&current.odometry, "ground_arrival.actual").is_err() {
            return GoalRegionAssessment::OutsideRegion;
        }
        let geometry = self.region(current, options, boundary, GroundLayerResult::Unresolved);
        // Drift uses the original reacquisition path before considering layer
        // incompleteness. A layer verdict cannot mask an XY/yaw miss.
        if geometry == GoalRegionAssessment::OutsideRegion { return geometry; }
        let relation = if source_retired || current.odometry.frame_id != self.goal.frame_id {
            GroundLayerResult::Unresolved
        } else { self.current_relation(current).unwrap_or(GroundLayerResult::Unresolved) };
        self.region(current, options, boundary, relation)
    }
    fn current_relation(&self, current: &ActualExecutionContext) -> Option<GroundLayerResult> {
        let observed = current.ground_evidence.as_ref()?;
        let previous_observed = self.actual.ground_evidence.as_ref()?;
        let state = self.actual.navigation.as_ref()?.walk_state().ok()?;
        let query = state.bind_body_query(&current.source).ok()?;
        let p = current.odometry.pose.position; let q = current.odometry.pose.orientation;
        let navigation = state.with_body_navigation_at([p.x,p.y], |navigation|
            query.navigation_evidence_at(([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]),
                &current.ground, navigation)).ok()?.ok()?;
        match navigation.relation_to(&observed.observation) {
            BodyGroundSourceRelation::Same => Some(GroundLayerResult::Matched),
            BodyGroundSourceRelation::KnownDifferent => Some(GroundLayerResult::KnownConflict),
            BodyGroundSourceRelation::Unresolved => {
                // A carried/copied source cannot create a relation. Both earlier
                // Same and KnownConflict survive the same proven pure-carry path.
                // New direct Known, a semantic seam or any unobserved refresh
                // interval breaks carry and therefore yields Unresolved.
                let pure_carry = !navigation.is_direct() && !observed.observation.is_direct()
                    && navigation.carry_intact() && observed.carry_intact
                    && observed.observation.carry_intact()
                    && self.navigation_evidence.same_source(&navigation)
                    && previous_observed.observation.same_source(&observed.observation);
                Some(if pure_carry { self.relation } else { GroundLayerResult::Unresolved })
            }
        }
    }
}
