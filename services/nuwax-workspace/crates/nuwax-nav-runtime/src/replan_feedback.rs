//! Reuse already successful work after a correlated execution refusal.
//!
//! This memo is not a motion/path admission check or a no-path cache. Owners
//! retain their original stop proof, deadline, cancellation and result tickets.
use std::sync::Arc;
use nuwax_nav_mls_planner::{surface::BodyGroundState, SurfaceSnapshot};
use nuwax_nav_pct::{RoutePenalty, SearchLimits};
use nuwax_nav_pct::ground_route::{GroundRouteGoal, GroundRouteWindow, GroundWalkState, OwnedGroundRouteWindow, OwnedGroundWalkState};
use nuwax_nav_scan::GroundCandidateBinding;
use nuwax_nav_types_messages::{GoalOptions, PoseStamped, Twist2};
use crate::PathExecutionIdentity;

/// A source-owned address actually consumed by the executor. This is not an
/// arrival proof; its binding retains the original candidate sampling grid.
#[derive(Clone, Debug)]
pub struct GroundActualBasis {address:GroundActualAddress}
#[derive(Clone,Debug)]
enum GroundActualAddress {
    /// Established synchronously before the first asynchronous request.
    Initial(OwnedGroundWalkState),
    Candidate {binding:Arc<GroundCandidateBinding>,edge:usize,fraction:f64,actual_xy:[f64;2]},
}
impl GroundActualBasis {
    pub(crate) fn initial(state:OwnedGroundWalkState)->Self {Self {address:GroundActualAddress::Initial(state)}}
    pub(crate) fn into_initial(self)->Option<OwnedGroundWalkState> {
        match self.address {GroundActualAddress::Initial(state)=>Some(state),_=>None}
    }
    pub(crate) fn new(binding:Arc<GroundCandidateBinding>,edge:usize,fraction:f64,actual_xy:[f64;2])->Self {
        Self {address:GroundActualAddress::Candidate {binding,edge,fraction,actual_xy}}
    }
    pub(crate) fn continuation(&self)->Result<OwnedGroundWalkState,String> {
        match &self.address {
            GroundActualAddress::Initial(state)=>Ok(state.clone()),
            GroundActualAddress::Candidate {binding,edge,fraction,actual_xy}=>binding.candidate()
                .continuation_at(*edge,*fraction,*actual_xy)
                .ok_or_else(||"ground actual address is outside its retained candidate".into()),
        }
    }
    pub(crate) fn walk_state(&self)->Result<GroundWalkState<'_>,String> {
        match &self.address {
            GroundActualAddress::Initial(state)=>Ok(state.walk_state()),
            GroundActualAddress::Candidate {binding,edge,fraction,actual_xy}=>binding.candidate()
                .walk_state_at(*edge,*fraction,*actual_xy)
                .ok_or_else(||"ground actual address is outside its retained candidate".into()),
        }
    }
    pub(crate) fn matches_binding(&self,binding:&Arc<GroundCandidateBinding>)->bool {
        matches!(&self.address,GroundActualAddress::Candidate {binding:current,..} if Arc::ptr_eq(current,binding))
    }
}
impl PartialEq for GroundActualBasis {
    fn eq(&self,other:&Self)->bool {
        match (&self.address,&other.address) {
            (GroundActualAddress::Initial(a),GroundActualAddress::Initial(b))=>a==b,
            (GroundActualAddress::Candidate {binding:a,edge:ae,fraction:af,actual_xy:axy},
             GroundActualAddress::Candidate {binding:b,edge:be,fraction:bf,actual_xy:bxy})=>
                Arc::ptr_eq(a,b) && ae==be && af.to_bits()==bf.to_bits()
                    && axy.map(f64::to_bits)==bxy.map(f64::to_bits),
            _=>false,
        }
    }
}
impl Eq for GroundActualBasis {}

/// Original finite Ground intent, distinct from the current actual incoming
/// address. Height absence and explicit zero remain different inputs.
#[derive(Clone, Debug)]
pub struct GroundIntent {
    window: OwnedGroundRouteWindow,
    goal: GroundRouteGoal,
    resolved_ground_z: Option<f64>,
}
impl GroundIntent {
    pub(crate) fn new(window: GroundRouteWindow<'_>, goal: GroundRouteGoal,
        resolved_ground_z: Option<f64>) -> Self {
        Self { window:window.freeze(), goal, resolved_ground_z }
    }
    pub(crate) fn window(&self) -> GroundRouteWindow<'_> { self.window.window() }
    pub(crate) fn goal(&self) -> GroundRouteGoal { self.goal }
    pub(crate) fn resolved_ground_z(&self) -> Option<f64> { self.resolved_ground_z }
    pub(crate) fn same_intent(&self, other: &Self) -> bool {
        let a=self.window(); let b=other.window();
        let address=|p:nuwax_nav_pct::ground_route::GroundRoutePosition<'_>| {
            let (connection,sample,fraction)=p.indices();(connection,sample,fraction.to_bits())
        };
        a.from().window_to(b.from()).is_some()
            && address(a.from())==address(b.from()) && address(a.to())==address(b.to())
            && self.goal.xy.map(f64::to_bits)==other.goal.xy.map(f64::to_bits)
            && self.goal.requested_ground_z.map(f64::to_bits)==other.goal.requested_ground_z.map(f64::to_bits)
            && self.resolved_ground_z.map(f64::to_bits)==other.resolved_ground_z.map(f64::to_bits)
    }
}
impl PartialEq for GroundIntent {
    fn eq(&self, other: &Self) -> bool { self.same_intent(other) }
}
impl Eq for GroundIntent {}

/// Actual inputs consumed by the execution owner, including its recent overlay
/// and carried ground. A candidate command is never an applied observation.
#[derive(Clone)]
pub struct ActualExecutionContext {
    /// Exact source owned by the executor, not the planner's terrain-only source.
    pub source: Arc<SurfaceSnapshot>,
    /// Full measured pose and frame; timestamps do not change the physical input.
    pub odometry: PoseStamped,
    /// Existing carried association, including unknown-ground continuation.
    pub ground: BodyGroundState,
    /// Committed navigation address; predicting a command never advances it.
    pub navigation: Option<GroundActualBasis>,
    /// Actual APPLIED value already consumed by the execution owner. Admission
    /// may reuse a same-version prior observation, never an unconsumed latch.
    pub applied: Twist2,
}
impl std::fmt::Debug for ActualExecutionContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ActualExecutionContext").field("odometry", &self.odometry)
            .field("ground", &self.ground).field("applied", &self.applied).finish_non_exhaustive()
    }
}

impl PartialEq for ActualExecutionContext {
    fn eq(&self, other: &Self) -> bool { same_actual(self, other) }
}

/// Sticky in-process status, scoped by the original admitted path identity.
/// Running contexts help capture an initial attempt; only the original ZERO
/// receipt transition may set `zero_confirmed_rejection`.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceReplanFeedback {
    /// Existing owner/goal/planner/path correlation, not an input digest.
    pub path_identity: Arc<PathExecutionIdentity>,
    /// Context from the same version-checked result as ReplanRequired.
    pub rejected_context: Option<Arc<ActualExecutionContext>>,
    /// Most recent actual context consumed by the current execution owner.
    pub current_context: Option<Arc<ActualExecutionContext>>,
    /// True only after the original refusal-to-ZERO acknowledgement path.
    pub zero_confirmed_rejection: bool,
}

/// Inputs actually consumed by one successful global computation. The owner
/// retires this memo on a goal/domain/owner change. Output revision is NOT input.
#[derive(Clone)]
pub struct SurfaceReplanInput {
    /// Planner source, separately from the executor's recent-overlay source.
    pub source: Arc<SurfaceSnapshot>,
    /// Global search start, including full quaternion and frame.
    pub start: PoseStamped,
    /// Sealed goal pose and frame.
    pub goal: PoseStamped,
    /// Original goal options.
    pub options: GoalOptions,
    /// Original boundary, in original order.
    pub boundary: Arc<Vec<[f64; 2]>>,
    /// Actual execution inputs at request capture; absent proof never holds work.
    pub actual: Option<Arc<ActualExecutionContext>>,
    /// Internal Ground intent; existing public Body requests leave this absent.
    pub ground_intent: Option<GroundIntent>,
    /// This attempt's unchanged search limits.
    pub limits: SearchLimits,
    /// Explicit body-to-support calibration for the chosen entry point.
    pub offset: f64,
    /// Existing route/execution mode, fixed by the owner configuration.
    pub local_planning_enabled: bool,
    /// The immutable snapshot also passed to PCT, not a second read of its costs.
    pub penalties: Arc<Vec<RoutePenalty>>,
}

/// Exact full pose/frame comparison, excluding transport timestamps only.
pub fn same_pose(a: &PoseStamped, b: &PoseStamped) -> bool {
    let body = |p: &PoseStamped| {
        let p = p.pose;
        ([p.position.x,p.position.y,p.position.z],
            [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    };
    a.frame_id == b.frame_id
        && crate::route_planning::pose_bits(body(a)) == crate::route_planning::pose_bits(body(b))
}

/// Reuse the local planner's existing exact, ordered cost comparison.
pub fn same_penalties(a: &[RoutePenalty], b: &[RoutePenalty]) -> bool {
    crate::route_planning::exact_costs(a, b)
}

/// Complete actual state comparison; no hash, pose rounding or tolerances.
pub fn same_actual(a: &ActualExecutionContext, b: &ActualExecutionContext) -> bool {
    a.source.same_body_query_view(&b.source) && same_pose(&a.odometry, &b.odometry)
        && a.ground == b.ground && a.navigation == b.navigation
        && [a.applied.linear.x,a.applied.linear.y,a.applied.angular_z].map(f64::to_bits)
            == [b.applied.linear.x,b.applied.linear.y,b.applied.angular_z].map(f64::to_bits)
}
impl SurfaceReplanInput {
    /// Missing actual context is deliberately inconclusive, not a new gate.
    pub fn same_complete_input(&self, other: &Self) -> bool {
        let actual_equal = self.actual.as_ref().zip(other.actual.as_ref())
            .is_some_and(|(a,b)| same_actual(a,b) && (self.ground_intent.is_none()
                || a.navigation.is_some() && b.navigation.is_some()));
        actual_equal && self.ground_intent == other.ground_intent
            && self.source.same_body_query_view(&other.source)
            && same_pose(&self.start, &other.start) && same_pose(&self.goal, &other.goal)
            && self.options.allow_reverse == other.options.allow_reverse
            && self.options.position_tolerance_m.to_bits() == other.options.position_tolerance_m.to_bits()
            && self.options.yaw_tolerance_rad.to_bits() == other.options.yaw_tolerance_rad.to_bits()
            && self.boundary.len() == other.boundary.len()
            && self.boundary.iter().zip(other.boundary.iter())
                .all(|(a,b)| a.map(f64::to_bits) == b.map(f64::to_bits))
            && self.limits.max_states == other.limits.max_states
            && self.offset.to_bits() == other.offset.to_bits()
            && self.local_planning_enabled == other.local_planning_enabled
            && same_penalties(&self.penalties, &other.penalties)
    }
}

/// One published success, not an unbounded history or a blocked-state database.
/// K is the original path identity (production) or existing plan counter (sim).
pub struct SuccessfulReplanMemo<K> {
    published: Option<(K, SurfaceReplanInput)>,
}
impl<K> Default for SuccessfulReplanMemo<K> {
    fn default() -> Self { Self { published: None } }
}
impl<K: PartialEq> SuccessfulReplanMemo<K> {
    /// Call only after the original publish/admission succeeds, never on Err or
    /// an obsolete task's completion. Existing tickets still retire late results.
    pub fn published(&mut self, path: K, input: SurfaceReplanInput) {
        self.published = Some((path, input));
    }
    /// Reuse only a correlated, actual execution refusal after its original stop
    /// confirmation. The caller supplies the unmodified rejected context, or
    /// None for an initial wait, missing proof, stale owner or pending planner.
    /// A success at A followed by refusal at B cannot suppress a retry back at A.
    pub fn should_reuse(
        &self,
        rejection: Option<(&K, &ActualExecutionContext)>,
        input: &SurfaceReplanInput,
    ) -> bool {
        self.published.as_ref().zip(rejection)
            .is_some_and(|((path, previous), (rejected_path, rejected_context))| {
                path == rejected_path
                    && input.actual.as_deref()
                        .is_some_and(|current| same_actual(rejected_context, current))
                    && previous.same_complete_input(input)
            })
    }
    /// Original owner/goal/domain retirement, not a new synthetic generation.
    pub fn clear(&mut self) { self.published = None; }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{NavFrameId, Point3, Pose3, Quaternion, Timestamp};
    fn input() -> SurfaceReplanInput {
        let source = Arc::new(crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                fixed_collision:None,
                footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,
                max_step_up_m:0.2,max_step_down_m:0.2,max_cross_slope_deg:18.,support_uncertainty_m:0.,
            }).unwrap());
        let start = PoseStamped { frame_id:NavFrameId::Map, stamp:Timestamp::default(),
            pose:Pose3 { position:Point3::new(0.5,1.0,0.5), orientation:Quaternion::IDENTITY } };
        let ground = source.body_query().start_ground_state(([0.5,1.0,0.5],[0.,0.,0.,1.])).unwrap();
        let actual = Arc::new(ActualExecutionContext { source:source.clone(), odometry:start.clone(), ground, navigation:None, applied:Twist2::ZERO });
        SurfaceReplanInput { source, start:start.clone(), goal:start, options:GoalOptions::default(),
            boundary:Arc::new(vec![]), actual:Some(actual), ground_intent:None, limits:SearchLimits{max_states:256},
            offset:0.4, local_planning_enabled:false, penalties:Arc::new(vec![]) }
    }
    #[test]
    fn only_correlated_published_success_with_complete_actual_input_is_reused() {
        let input=input(); let mut memo=SuccessfulReplanMemo::default();
        assert!(!memo.should_reuse(Some((&7,input.actual.as_deref().unwrap())),&input));
        memo.published(7,input.clone());
        assert!(memo.should_reuse(Some((&7,input.actual.as_deref().unwrap())),&input));
        assert!(!memo.should_reuse(None,&input));
        assert!(!memo.should_reuse(Some((&8,input.actual.as_deref().unwrap())),&input));
        let mut absent=input.clone(); absent.actual=None;
        assert!(!memo.should_reuse(Some((&7,input.actual.as_deref().unwrap())),&absent));
        memo.clear(); assert!(!memo.should_reuse(Some((&7,input.actual.as_deref().unwrap())),&input));
    }
    #[test]
    fn refusal_at_b_does_not_hold_restored_a_even_when_published_input_is_a() {
        let a=input();
        let actual_a=a.actual.as_deref().unwrap();
        let mut memo=SuccessfulReplanMemo::default();
        memo.published(7,a.clone());
        assert!(memo.should_reuse(Some((&7,actual_a)),&a), "A/A/A positive control");
        for dependency in 0..4 {
            let mut actual_b=actual_a.clone();
            match dependency {
                0 => actual_b.odometry.pose.position.z=f64::from_bits(
                    actual_b.odometry.pose.position.z.to_bits()+1),
                1 => actual_b.applied.linear.x=f64::from_bits(1),
                2 => actual_b.ground=actual_b.source.body_query()
                    .start_ground_state(([99.,99.,0.7],[0.,0.,0.,1.])).unwrap(),
                _ => actual_b.source=Arc::new(actual_b.source
                    .with_recent_obstacle_points(1,1,0,&[],[(0,[2.,2.,0.7])]).unwrap()),
            }
            assert!(!same_actual(actual_a,&actual_b));
            // P was published with A, then refused with B. The current planner
            // input is exactly A again: comparing only previous/current is wrong.
            assert!(a.same_complete_input(&a));
            assert!(!memo.should_reuse(Some((&7,&actual_b)),&a),
                "rejection-only dependency {dependency} must not suppress retry at A");
        }
        assert!(!memo.should_reuse(None,&a), "missing refusal context falls back");
        let mut missing=a.clone(); missing.actual=None;
        assert!(!memo.should_reuse(Some((&7,actual_a)),&missing));
        // A later actual refusal at A can reuse P, without poisoning the memo.
        assert!(memo.should_reuse(Some((&7,actual_a)),&a));
    }
    #[test]
    fn timestamps_do_not_change_input_but_single_bits_and_real_dependencies_do() {
        let original=input(); let mut memo=SuccessfulReplanMemo::default(); memo.published(1,original.clone());
        let mut changed=original.clone(); changed.start.stamp=Timestamp::from_unix(2,0);
        Arc::make_mut(changed.actual.as_mut().unwrap()).odometry.stamp=Timestamp::from_unix(2,0);
        assert!(memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
        for dimension in 0..7 {
            let mut changed=original.clone(); let pose=&mut changed.start.pose;
            let value=match dimension {0=>&mut pose.position.x,1=>&mut pose.position.y,2=>&mut pose.position.z,
                3=>&mut pose.orientation.x,4=>&mut pose.orientation.y,5=>&mut pose.orientation.z,_=>&mut pose.orientation.w};
            *value=f64::from_bits(value.to_bits()+1);
            assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
        }
        let mut changed=original.clone();
        Arc::make_mut(changed.actual.as_mut().unwrap()).applied.linear.x=f64::from_bits(1);
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
        let mut changed=original.clone(); changed.options.allow_reverse=true;
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
        let mut changed=original.clone(); changed.limits.max_states+=1;
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
        let mut changed=original.clone(); changed.penalties=Arc::new(vec![RoutePenalty {center:[1.,0.,0.],radius:0.2,weight:16.}]);
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
    }
    #[test]
    fn ground_and_recent_overlay_remain_independent_real_dependencies() {
        let original=input(); let mut memo=SuccessfulReplanMemo::default(); memo.published(1,original.clone());
        let mut changed=original.clone();
        let actual=Arc::make_mut(changed.actual.as_mut().unwrap());
        actual.ground=actual.source.body_query()
            .start_ground_state(([99.,99.,0.7],[0.,0.,0.,1.])).unwrap();
        assert_ne!(actual.ground,original.actual.as_ref().unwrap().ground);
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed));
        let mut changed=original.clone();
        let actual=Arc::make_mut(changed.actual.as_mut().unwrap());
        actual.source=Arc::new(actual.source.with_recent_obstacle_points(1,1,0,&[],[(0,[2.,2.,0.7])]).unwrap());
        assert!(changed.source.same_body_query_view(&original.source), "planner terrain is unchanged");
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed), "executor-only recent input wakes retry");
        let mut changed=original.clone(); changed.source=Arc::new(original.source.fork_for_planning());
        assert!(!memo.should_reuse(Some((&1,original.actual.as_deref().unwrap())),&changed), "a different immutable owner is conservative change");
    }
    #[test]
    fn owners_with_different_path_key_types_share_the_same_decisions() {
        let base=input(); let mut sim=SuccessfulReplanMemo::default(); let mut production=SuccessfulReplanMemo::default();
        sim.published(1,base.clone()); production.published("owner:path1",base.clone());
        let mut changed=base.clone(); changed.start.pose.position.z=f64::from_bits(base.start.pose.position.z.to_bits()+1);
        for candidate in [&base,&changed,&base] {
            assert_eq!(sim.should_reuse(Some((&1,base.actual.as_deref().unwrap())),candidate),production.should_reuse(Some((&"owner:path1",base.actual.as_deref().unwrap())),candidate));
        }
        sim.clear(); production.clear();
        assert!(!sim.should_reuse(Some((&1,base.actual.as_deref().unwrap())),&base));
        assert!(!production.should_reuse(Some((&"owner:path1",base.actual.as_deref().unwrap())),&base));
    }
    #[test]
    fn new_path_retires_old_refusal_without_turning_it_into_no_path() {
        let input=input(); let mut memo=SuccessfulReplanMemo::default();
        memo.published(1,input.clone()); assert!(memo.should_reuse(Some((&1,input.actual.as_deref().unwrap())),&input));
        memo.published(2,input.clone());
        assert!(!memo.should_reuse(Some((&1,input.actual.as_deref().unwrap())),&input)); assert!(memo.should_reuse(Some((&2,input.actual.as_deref().unwrap())),&input));
    }
}
