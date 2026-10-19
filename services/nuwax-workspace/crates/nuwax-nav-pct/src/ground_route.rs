//! Opt-in planned Ground references. Public goal/wire defaults are not changed.
//! A planned association is neither an actual observation nor arrival evidence.
use crate::{PctPlanner, PlanningFailure, RoutePenalty, SearchLimits, SearchStats, Tomogram};
use crate::reference::Reference;
use crate::search::{RouteReferenceTrace, checkpoint};
use nuwax_nav_mls_planner::{SurfaceSnapshot, surface::{BodyNavigationSegment,
    NavigationGroundSample, NavigationPlanningFork, SourceGroundRecord, SurfaceBodyQueryView}};
use nuwax_types::navigation::{GroundHeightIntent, GroundLayerResult};
use std::sync::{Arc, atomic::AtomicBool};

type BodyPose = ([f64;3],[f64;4]);

/// Internal admission-deterministic predicate. Only this module can construct
/// it, from a frozen request's MLS view/actual transform/boundary. MLS caches may
/// change, but there is no caller callback, output witness or cancellation poll.
pub(crate) struct GroundPhysicalAdmitter<'a> {
    body: &'a SurfaceBodyQueryView,
    actual: BodyPose,
    from: [f64;3],
    start_xy: [f64;2],
    initial_z: f64,
    anchored: bool,
    boundary: &'a [[f64;2]],
}
impl GroundPhysicalAdmitter<'_> {
    fn physical(&self,p:[f64;3])->BodyPose {
        if p==self.from {self.actual} else {
            ([self.actual.0[0]+(p[0]-self.start_xy[0]),self.actual.0[1]+(p[1]-self.start_xy[1]),
                self.actual.0[2]+if self.anchored {p[2]-self.initial_z}else{0.0}],self.actual.1)
        }
    }
    pub(crate) fn accepts(&self,a:[f64;3],b:[f64;3])->bool {
        let result=self.body.validate_unanchored_motion(self.physical(a),self.physical(b),self.boundary,&mut None).is_ok();
        #[cfg(test)]
        physical_test_capture::record(a,b,result);
        result
    }
}

#[cfg(test)]
pub(crate) mod physical_test_capture {
    use std::cell::RefCell;
    pub type Call=([[u64;3];2],bool);
    thread_local! {static CALLS:RefCell<Option<Vec<Call>>>=const {RefCell::new(None)};}
    pub(super) fn record(a:[f64;3],b:[f64;3],result:bool) {
        CALLS.with(|v|if let Some(v)=v.borrow_mut().as_mut(){v.push(([a.map(f64::to_bits),b.map(f64::to_bits)],result));});
    }
    pub(crate) fn capture<T>(call:impl FnOnce()->T)->(T,Vec<Call>) {
        struct Reset(Option<Vec<Call>>);
        impl Drop for Reset {fn drop(&mut self){CALLS.with(|v|*v.borrow_mut()=self.0.take());}}
        let _reset=Reset(CALLS.with(|v|v.replace(Some(Vec::new()))));
        let result=call();(result,CALLS.with(|v|v.take().unwrap()))
    }
}


/// Independent navigation start. None is genuinely Unanchored, never body Z
/// minus calibration. A supplied height may be a carried navigation reference.
#[derive(Clone, Copy, Debug)]
pub struct GroundRouteStart {
    pub xy: [f64;2],
    pub ground_z: Option<f64>,
}
#[derive(Clone, Copy, Debug)]
pub struct GroundRouteGoal {
    pub xy: [f64;2],
    pub requested_ground_z: Option<f64>,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundRoutePoint {
    pub xy: [f64;2],
    pub ground_z: Option<f64>,
}
#[derive(Debug)]
pub enum GroundRouteFailure {
    Planning(PlanningFailure),
    /// The target column has known references, none matching the explicit
    /// requested height under the existing source association rule.
    KnownLayerConflict(GroundHeightIntent),
}
impl From<PlanningFailure> for GroundRouteFailure {
    fn from(error:PlanningFailure)->Self {Self::Planning(error)}
}
impl From<String> for GroundRouteFailure {
    fn from(error:String)->Self {Self::Planning(error.into())}
}
impl From<&str> for GroundRouteFailure {
    fn from(error:&str)->Self {Self::Planning(error.into())}
}

#[derive(Clone, Copy, Debug)]
pub enum PlannedGroundAssociation<'a> {
    /// Borrowed from the exact Tomogram/source retained by this route. A legacy
    /// occupancy-derived graph cell has no native record, hence source=None.
    Known { source:Option<SourceGroundRecord<'a>> },
    Carried,
    Unanchored,
}
#[derive(Clone, Copy, Debug)]
pub struct PlannedGroundSample<'a> {
    pub point:GroundRoutePoint,
    pub association:PlannedGroundAssociation<'a>,
}
#[derive(Clone, Copy, Debug)]
pub struct GroundConnection {
    pub point_span:[usize;2],
    pub steps:usize,
}
/// Owner-coupled B1 trace. No bare cell id escapes for reuse in a newer map.
#[derive(Clone, Debug)]
pub struct GroundRouteReference {
    map:Arc<Tomogram>,
    trace:Arc<RouteReferenceTrace>,
}
impl GroundRouteReference {
    pub fn connections(&self)->impl Iterator<Item=GroundConnection>+'_ {
        self.trace.connections.iter().map(|c|GroundConnection {point_span:c.point_span,steps:c.steps})
    }
    /// Resolve an original discrete evaluator sample. This only dereferences
    /// the retained selected id; it does not select a layer from current body Z
    /// or query a replacement source. Collapsed parent tails remain addressable.
    pub fn sample(&self,connection:usize,index:usize)->Option<PlannedGroundSample<'_>> {
        let c=self.trace.connections.get(connection)?;
        if index>c.steps {return None;}
        let reference=self.trace.changes[c.changes.clone()].iter().rev()
            .find(|change|change.sample_index<=index)?.reference;
        let t=index as f64/c.steps as f64;
        let xy=std::array::from_fn(|i|c.evaluated_from[i]+t*(c.evaluated_to[i]-c.evaluated_from[i]));
        let association=match reference {
            Reference::Known(id)=>PlannedGroundAssociation::Known {source:self.map.source_ground_record(id)},
            Reference::Unknown {..}=>PlannedGroundAssociation::Carried,
            Reference::Unanchored {..}=>PlannedGroundAssociation::Unanchored,
        };
        Some(PlannedGroundSample {point:GroundRoutePoint {xy,
            ground_z:self.map.reference_height_at(reference,xy)},association})
    }
    pub fn matches_source(&self,source:&SurfaceSnapshot)->bool {self.map.matches_surface(source)}
}
#[derive(Clone, Debug)]
pub struct PlannedGroundRoute {
    pub points:Vec<GroundRoutePoint>,
    /// Fixed actual attitude and initial physical bias, checked by MLS on each
    /// admitted/shortened segment. Later steering/yaw still needs Scan checks.
    pub guide_body_poses:Vec<BodyPose>,
    pub height:GroundHeightIntent,
    pub planned_layer:GroundLayerResult,
    pub reference:GroundRouteReference,
    pub stats:SearchStats,
}
impl PctPlanner {
    /// Internal migration seam, opt-in only. Uses the original search, resource
    /// limits, captured penalty snapshot and trace evaluator. No execution event
    /// or public goal default is changed by producing this planned result.
    pub fn ground_route_with_limits_and_penalties(
        &self,source:&SurfaceSnapshot,actual:BodyPose,start:GroundRouteStart,goal:GroundRouteGoal,
        xy_tolerance:f64,boundary:&[[f64;2]],cancelled:Option<&AtomicBool>,radius:Option<f64>,
        limits:SearchLimits,penalties:&[RoutePenalty],
    )->Result<PlannedGroundRoute,GroundRouteFailure> {
        self.ground_route_impl::<true>(source,actual,start,goal,xy_tolerance,boundary,cancelled,radius,limits,penalties)
    }
    #[cfg(test)]
    pub(crate) fn ground_route_original_order(
        &self,source:&SurfaceSnapshot,actual:BodyPose,start:GroundRouteStart,goal:GroundRouteGoal,
        xy_tolerance:f64,boundary:&[[f64;2]],cancelled:Option<&AtomicBool>,radius:Option<f64>,
        limits:SearchLimits,penalties:&[RoutePenalty],
    )->Result<PlannedGroundRoute,GroundRouteFailure> {
        self.ground_route_impl::<false>(source,actual,start,goal,xy_tolerance,boundary,cancelled,radius,limits,penalties)
    }
    fn ground_route_impl<const EARLY:bool>(
        &self,source:&SurfaceSnapshot,actual:BodyPose,start:GroundRouteStart,goal:GroundRouteGoal,
        xy_tolerance:f64,boundary:&[[f64;2]],cancelled:Option<&AtomicBool>,radius:Option<f64>,
        limits:SearchLimits,penalties:&[RoutePenalty],
    )->Result<PlannedGroundRoute,GroundRouteFailure> {
        checkpoint(cancelled)?;
        if start.xy.into_iter().chain(goal.xy).chain(start.ground_z).chain(goal.requested_ground_z)
            .any(|value|!value.is_finite()) {
            return Err("Ground reference height must be finite when present".into());
        }
        let body=source.body_query();
        // Actual geometry validation uses the complete pose, with no navigation
        // height inferred from its Z. This also validates the original model.
        body.validate_unanchored_motion(actual,actual,boundary,&mut None)
            .map_err(|error|error.to_string())?;
        let map=self.map(source)?;
        checkpoint(cancelled)?;
        let initial_z=match start.ground_z {
            Some(h)=>map.cell_at([start.xy[0],start.xy[1],h])
                .and_then(|id|map.ground_height_at(id,start.xy)).unwrap_or(h),
            None=>actual.0[2], // private physical coordinate, never exported as ground
        };
        let target=goal.requested_ground_z.and_then(|h|map.cell_at([goal.xy[0],goal.xy[1],h]));
        if goal.requested_ground_z.is_some() && target.is_none()
            && map.columns.get(&map.point_column([goal.xy[0],goal.xy[1],initial_z]))
                .is_some_and(|ids|!ids.is_empty()) {
            return Err(GroundRouteFailure::KnownLayerConflict(GroundHeightIntent {
                requested_ground_z:goal.requested_ground_z,resolved_ground_z:None,
            }));
        }
        let from=[start.xy[0],start.xy[1],initial_z];
        let to=[goal.xy[0],goal.xy[1],goal.requested_ground_z.unwrap_or(initial_z)];
        let admitter=GroundPhysicalAdmitter {body:&body,actual,from,start_xy:start.xy,
            initial_z,anchored:start.ground_z.is_some(),boundary};
        // Existing penalties are in actual body coordinates. Convert this one
        // captured snapshot once, without reading the planner's mutable costs.
        let penalties:Vec<_>=penalties.iter().map(|p|RoutePenalty {
            center:[p.center[0]-actual.0[0]+start.xy[0],p.center[1]-actual.0[1]+start.xy[1],
                p.center[2]-actual.0[2]+initial_z],radius:p.radius,weight:p.weight,
        }).collect();
        let traced=map.plan_ground_physical::<EARLY>(from,to,xy_tolerance,boundary,cancelled,radius,&penalties,
            &admitter,limits,start.ground_z.is_none().then_some(actual.0[2]),
            if start.ground_z.is_some(){target}else{None})?;
        let last=traced.trace.changes.last().expect("successful recorded route has an initial sample").reference;
        let end=*traced.route.points.last().unwrap();
        let resolved_ground_z=match last {
            Reference::Known(_)=>map.reference_height_at(last,[end[0],end[1]]),
            _=>None,
        };
        let planned_layer=match (last,target) {
            // A positive XY region may stop at a known neighboring center while
            // the exact requested column is missing. That does not resolve an
            // explicit layer request, even though this endpoint has a source h.
            (_,None) if goal.requested_ground_z.is_some()=>GroundLayerResult::Unresolved,
            (Reference::Known(id),Some(target)) if id!=target=>GroundLayerResult::KnownConflict,
            (Reference::Known(_),_)=>GroundLayerResult::Matched,
            _=>GroundLayerResult::Unresolved,
        };
        let points=traced.route.points.iter().map(|p|GroundRoutePoint {xy:[p[0],p[1]],
            ground_z:start.ground_z.map(|_|p[2])}).collect();
        let guide_body_poses=traced.route.points.iter().copied().map(|p|admitter.physical(p)).collect();
        Ok(PlannedGroundRoute {points,guide_body_poses,
            height:GroundHeightIntent {requested_ground_z:goal.requested_ground_z,resolved_ground_z},
            planned_layer,reference:GroundRouteReference {map,trace:traced.trace},stats:traced.route.stats})
    }
}

/// An address borrows its original owner. It never resolves an id in another map.
#[derive(Clone, Copy, Debug)]
pub struct GroundRoutePosition<'a> {
    owner: &'a GroundRouteReference,
    connection: usize,
    sample: usize,
    fraction: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct GroundRouteWindow<'a> {
    from: GroundRoutePosition<'a>,
    to: GroundRoutePosition<'a>,
}
/// Frozen intent, not consumed progress. A detour may have no completed join.
#[derive(Clone, Debug)]
pub struct OwnedGroundRouteWindow {
    owner: GroundRouteReference,
    from: (usize,usize,f64),
    to: (usize,usize,f64),
}
impl GroundRouteReference {
    pub fn position(&self,connection:usize,sample:usize,fraction:f64)->Option<GroundRoutePosition<'_>> {
        let c=self.trace.connections.get(connection)?;
        if sample>c.steps || !fraction.is_finite() || !(0.0..1.0).contains(&fraction)
            || sample==c.steps && fraction!=0.0 {return None;}
        Some(GroundRoutePosition {owner:self,connection,sample,fraction})
    }
    pub fn full_window(&self)->Option<GroundRouteWindow<'_>> {
        let last=self.trace.connections.len().checked_sub(1)?;
        self.position(0,0,0.0)?.window_to(self.position(last,self.trace.connections[last].steps,0.0)?)
    }
    pub fn freeze_candidate(&self,segments:Vec<GroundWalkSegment<'_>>)
        ->Result<GroundCandidateReference,PlanningFailure> {
        self.freeze_candidate_impl(segments,None)
    }
    /// A stationary accepted candidate retains its real incoming address without
    /// inventing an executed connection or a second source query.
    pub fn freeze_candidate_from(&self,incoming:GroundWalkState<'_>,segments:Vec<GroundWalkSegment<'_>>)
        ->Result<GroundCandidateReference,PlanningFailure> {
        if !incoming.same_reference_owner(self) {return Err("candidate reference owner changed".into());}
        self.freeze_candidate_impl(segments,Some((incoming.reference,incoming.reference_sample_xy,incoming.xy)))
    }
    fn freeze_candidate_impl(&self,segments:Vec<GroundWalkSegment<'_>>,
        initial:Option<(Reference,[f64;2],[f64;2])>)->Result<GroundCandidateReference,PlanningFailure> {
        let mut connections=Vec::with_capacity(segments.len());let mut changes=Vec::new();
        let mut previous:Option<GroundWalkState<'_>>=None;
        for segment in segments {
            if previous.is_none() && initial.is_some_and(|(reference,xy,actual)|
                (reference,xy,actual)!=(segment.from.reference,segment.from.reference_sample_xy,segment.from.xy)) {
                return Err("candidate first segment differs from its actual incoming address".into());
            }
            if !std::ptr::eq(segment.from.map,self.map.as_ref()) {return Err("candidate reference owner changed".into());}
            if previous.is_some_and(|end|end.reference!=segment.from.reference || end.xy!=segment.from.xy
                || end.reference_sample_xy!=segment.from.reference_sample_xy) {
                return Err("candidate reference segments are disconnected".into());
            }
            previous=Some(segment.to);
            let start=changes.len();changes.extend(segment.trace.changes);
            connections.push(GroundWalkConnection {from:segment.from.xy,to:segment.to.xy,
                reference_sample_xy:segment.from.reference_sample_xy,
                steps:segment.trace.steps,changes:start..changes.len()});
        }
        Ok(GroundCandidateReference {map:Arc::clone(&self.map),
            recording:CandidateRecording::Recorded(Arc::new(GroundCandidateTrace {connections,changes,initial}))})
    }
}
impl<'a> GroundRoutePosition<'a> {
    pub fn is_terminal(self)->bool {
        self.connection+1==self.owner.trace.connections.len()
            && self.sample==self.owner.trace.connections[self.connection].steps && self.fraction==0.0
    }
    pub fn indices(self)->(usize,usize,f64) {(self.connection,self.sample,self.fraction)}
    pub fn walk_state(self)->GroundWalkState<'a> {
        let c=&self.owner.trace.connections[self.connection];
        let reference=self.owner.trace.changes[c.changes.clone()].iter().rev()
            .find(|change|change.sample_index<=self.sample).expect("valid original sample").reference;
        let sample_t=self.sample as f64/c.steps as f64;
        let t=(self.sample as f64+self.fraction)/c.steps as f64;
        GroundWalkState {map:&self.owner.map,reference,
            reference_sample_xy:std::array::from_fn(|i|c.evaluated_from[i]+sample_t*(c.evaluated_to[i]-c.evaluated_from[i])),
            xy:std::array::from_fn(|i|c.evaluated_from[i]+t*(c.evaluated_to[i]-c.evaluated_from[i]))}
    }
    /// Capture an original address for a request. Actual XY is independent of
    /// the evaluator sample used by the retained navigation association.
    pub fn continuation(self,actual_xy:[f64;2])->Option<OwnedGroundWalkState> {
        if !actual_xy.into_iter().all(f64::is_finite) {return None;}
        let state=self.walk_state();
        Some(OwnedGroundWalkState {map:Arc::clone(&self.owner.map),reference:state.reference,
            reference_sample_xy:state.reference_sample_xy,actual_xy})
    }
    pub fn window_to(self,to:Self)->Option<GroundRouteWindow<'a>> {
        (Arc::ptr_eq(&self.owner.map,&to.owner.map) && Arc::ptr_eq(&self.owner.trace,&to.owner.trace))
            .then_some(GroundRouteWindow {from:self,to})
    }
}
impl<'a> GroundRouteWindow<'a> {
    pub fn from(self)->GroundRoutePosition<'a> {self.from}
    pub fn to(self)->GroundRoutePosition<'a> {self.to}
    pub fn owner(self)->&'a GroundRouteReference {self.from.owner}
    pub fn freeze(self)->OwnedGroundRouteWindow {
        OwnedGroundRouteWindow {owner:self.from.owner.clone(),from:self.from.indices(),to:self.to.indices()}
    }
}
impl OwnedGroundRouteWindow {
    pub fn window(&self)->GroundRouteWindow<'_> {
        GroundRouteWindow {
            from:self.owner.position(self.from.0,self.from.1,self.from.2).expect("frozen original start"),
            to:self.owner.position(self.to.0,self.to.1,self.to.2).expect("frozen original end"),
        }
    }
}
/// Small owned continuation for crossing a request/command lifetime. It owns
/// only the original Tomogram Arc and the exact small incoming association.
#[derive(Clone,Debug)]
pub struct OwnedGroundWalkState {
    map:Arc<Tomogram>,reference:Reference,reference_sample_xy:[f64;2],actual_xy:[f64;2],
}
impl OwnedGroundWalkState {
    pub fn walk_state(&self)->GroundWalkState<'_> {
        GroundWalkState {map:&self.map,reference:self.reference,
            reference_sample_xy:self.reference_sample_xy,xy:self.actual_xy}
    }
    pub fn semantic_key(&self)->GroundWalkKey {self.walk_state().semantic_key()}
    pub fn same_reference_owner(&self,owner:&GroundRouteReference)->bool {
        self.walk_state().same_reference_owner(owner)
    }
    pub fn planning_fork(&self,current:&SurfaceSnapshot)->Result<NavigationPlanningFork,PlanningFailure> {
        let source=self.map.source.as_ref().ok_or("recorded navigation has no physical source")?;
        current.fork_for_navigation_planning(source).map_err(|error|PlanningFailure::from(error.to_string()))
    }
    pub fn bind_body_query(&self,current:&SurfaceSnapshot)->Result<SurfaceBodyQueryView,PlanningFailure> {
        self.walk_state().bind_body_query(current)
    }
}
impl PartialEq for OwnedGroundWalkState {
    fn eq(&self,other:&Self)->bool {self.semantic_key()==other.semantic_key()}
}
impl Eq for OwnedGroundWalkState {}

#[derive(Clone,Copy,Debug)]
pub struct GroundWalkState<'a> {
    map:&'a Tomogram,reference:Reference,reference_sample_xy:[f64;2],xy:[f64;2],
}
#[derive(Clone,Copy,Debug,PartialEq,Eq,Hash)]
pub struct GroundWalkKey {owner:usize,reference:Reference,reference_sample_xy:[u64;2],xy:[u64;2]}
impl<'a> GroundWalkState<'a> {
    pub fn xy(self)->[f64;2] {self.xy}
    pub fn reference_sample_xy(self)->[f64;2] {self.reference_sample_xy}
    pub fn same_reference_owner(self,owner:&GroundRouteReference)->bool {
        std::ptr::eq(self.map,owner.map.as_ref())
    }
    /// Bind once for the invocation without creating an owned continuation.
    pub fn bind_body_query(self,current:&SurfaceSnapshot)->Result<SurfaceBodyQueryView,PlanningFailure> {
        let source=self.map.source.as_ref().ok_or("recorded navigation has no physical source")?;
        current.body_query_with_navigation_basis(source).map_err(|error|PlanningFailure::from(error.to_string()))
    }
    /// Height at the original evaluator sample, never at a shifted actual XY.
    /// Unanchored remains None; physical height cannot fill it in.
    pub fn planned_ground_z(self)->Option<f64> {self.map.reference_height_at(self.reference,self.reference_sample_xy)}
    pub fn semantic_key(self)->GroundWalkKey {
        GroundWalkKey {owner:std::ptr::from_ref(self.map) as usize,reference:self.reference,
            reference_sample_xy:self.reference_sample_xy.map(f64::to_bits),xy:self.xy.map(f64::to_bits)}
    }
    pub fn walk_xy(self,to:[f64;2],boundary:&[[f64;2]],cancelled:Option<&AtomicBool>)
        ->Result<GroundWalkSegment<'a>,PlanningFailure> {
        checkpoint(cancelled)?;
        let mut scratch=Some(crate::reference::SegmentTrace::default());
        let result=self.map.reference_walk_recorded::<true>(self.xy,to,None,boundary,cancelled,
            &mut scratch,Some(self.reference));
        checkpoint(cancelled)?;
        let (_,_,reference)=result.ok_or("candidate XY reference is incompatible with its source")?;
        // Keep the evaluator's own final sample arithmetic, including rounding.
        let reference_sample_xy=std::array::from_fn(|axis|self.xy[axis]+1.0*(to[axis]-self.xy[axis]));
        Ok(GroundWalkSegment {from:self,to:Self {map:self.map,reference,reference_sample_xy,xy:to},trace:scratch.unwrap()})
    }
}
#[derive(Debug)]
pub struct GroundWalkSegment<'a> {from:GroundWalkState<'a>,to:GroundWalkState<'a>,trace:crate::reference::SegmentTrace}
impl<'a> GroundWalkSegment<'a> {
    pub fn end(&self)->GroundWalkState<'a> {self.to}

    fn reference_at(&self,index:usize)->Reference {
        let at=self.trace.changes.partition_point(|change|change.sample_index<=index);
        self.trace.changes[at-1].reference
    }
    fn sample_xy(&self,index:usize)->[f64;2] {
        if index==0 {return self.from.reference_sample_xy;}
        let t=index as f64/self.trace.steps as f64;
        std::array::from_fn(|axis|self.from.xy[axis]+t*(self.to.xy[axis]-self.from.xy[axis]))
    }
    pub(crate) fn navigation_at(&self,t:f64)->NavigationGroundSample<'a> {
        let (index,_)=original_grid_sample(self.trace.steps,t).expect("bounded navigation fraction");
        let reference=self.reference_at(index);let xy=self.sample_xy(index);
        navigation_sample(self.from.map,reference,xy)
    }
    pub(crate) fn navigation_changes(&self)->Vec<f64> {
        let mut changes:Vec<_>=self.trace.changes.iter().skip(1)
            .map(|change|change.sample_index as f64/self.trace.steps as f64).collect();
        // Reference changes alone omit a genuine LegacyPlane's discrete height
        // changes inside one id. Add just those changes, without re-evaluation.
        for (at,change) in self.trace.changes.iter().enumerate() {
            let Reference::Known(id)=change.reference else {continue;};
            let Some(SourceGroundRecord::LegacyPlane(record))=self.from.map.source_ground_record(id) else {continue;};
            let end=self.trace.changes.get(at+1).map_or(self.trace.steps+1,|next|next.sample_index);
            let mut previous=SourceGroundRecord::LegacyPlane(record).height_at(self.sample_xy(change.sample_index));
            for index in change.sample_index+1..end {
                let next=SourceGroundRecord::LegacyPlane(record).height_at(self.sample_xy(index));
                if next!=previous {changes.push(index as f64/self.trace.steps as f64);}
                previous=next;
            }
        }
        changes.sort_by(f64::total_cmp);changes.dedup();
        changes
    }
    #[cfg(test)]
    pub(crate) fn navigation_steps(&self)->usize {self.trace.steps}
    /// Borrow this already recorded walk for one H3 edge. The callback and
    /// fractions stay on this call's stack; the physical projection owns only
    /// the support interpretation it actually produced. No second walk runs.
    pub fn with_body_navigation<R>(&self,
        run:impl FnOnce(&SurfaceBodyQueryView,&BodyNavigationSegment<'_,'_>)->R)
        ->Result<R,PlanningFailure> {
        let source=self.from.map.source.as_ref().ok_or("recorded navigation has no physical source")?;
        if matches!(self.from.reference,Reference::Known(_))
            && self.from.map.reference_height_at(self.from.reference,self.from.reference_sample_xy).is_none() {
            return Err("recorded navigation start has no source-local sample height".into());
        }
        let changes=self.navigation_changes();
        let sample=|t|self.navigation_at(t);
        let navigation=BodyNavigationSegment::from_recorded(source,self.from.xy,self.to.xy,&changes,&sample);
        Ok(run(&source.body_query(),&navigation))
    }
}

#[cfg(test)]
pub(crate) fn source_less_navigation_adapter_rejected()->bool {
    let map=Tomogram::from_columns(0.1,[],0.4,0.0,0.1,0.1,20.0).unwrap();
    let from=GroundWalkState {map:&map,reference:Reference::Unanchored{column:[0,0]},
        reference_sample_xy:[0.025,0.025],xy:[0.025,0.025]};
    let edge=from.walk_xy([0.125,0.025],&[],None).unwrap();
    edge.with_body_navigation(|_,_|()).is_err()
}
// Compare the evaluator's original division, never floor(t*steps): rounded
// multiplication can move an exact boundary to the wrong discrete sample.
// The residual fraction is measured within those same divided endpoints.
fn original_grid_sample(steps:usize,t:f64)->Option<(usize,f64)> {
    if steps==0 || !t.is_finite() || !(0.0..=1.0).contains(&t) {return None;}
    let (mut low,mut high)=(0,steps);
    while low<high {
        let mid=low+(high-low)/2+1;
        if mid as f64/steps as f64<=t {low=mid;}else{high=mid-1;}
    }
    let fraction=if low==steps {0.0}else {
        let from=low as f64/steps as f64;let to=(low+1) as f64/steps as f64;
        ((t-from)/(to-from)).clamp(0.0,1.0_f64.next_down())
    };
    Some((low,fraction))
}
#[derive(Debug)]
struct GroundWalkConnection {
    from:[f64;2],to:[f64;2],reference_sample_xy:[f64;2],steps:usize,changes:std::ops::Range<usize>,
}
impl GroundWalkConnection {
    fn sample_xy(&self,index:usize)->[f64;2] {
        if index==0 {return self.reference_sample_xy;}
        let t=index as f64/self.steps as f64;
        std::array::from_fn(|axis|self.from[axis]+t*(self.to[axis]-self.from[axis]))
    }
}
#[derive(Debug)]
struct GroundCandidateTrace {connections:Vec<GroundWalkConnection>,changes:Vec<crate::reference::TraceChange>,
    initial:Option<(Reference,[f64;2],[f64;2])>}
// A leaf always owns an original recording, never another composition.
#[derive(Clone,Debug)]
struct RecordedSlice {trace:Arc<GroundCandidateTrace>,edge:usize,from:f64,to:f64}
#[derive(Clone,Debug)]
enum CandidateRecording {Recorded(Arc<GroundCandidateTrace>),Slices(Arc<[RecordedSlice]>)}
#[derive(Clone,Copy)]
struct BorrowedSlice<'a> {trace:&'a Arc<GroundCandidateTrace>,edge:usize,from:f64,to:f64}
impl<'a> BorrowedSlice<'a> {
    fn connection(self)->&'a GroundWalkConnection {&self.trace.connections[self.edge]}
    fn reference(self,index:usize)->Reference {
        let c=self.connection();let changes=&self.trace.changes[c.changes.clone()];
        changes[changes.partition_point(|v|v.sample_index<=index)-1].reference
    }
    fn owned(self)->RecordedSlice {RecordedSlice {trace:Arc::clone(self.trace),edge:self.edge,from:self.from,to:self.to}}
    fn original_fraction(self,t:f64)->f64 {
        if t==0.0 {self.from}else if t==1.0 {self.to}else {self.from+t*(self.to-self.from)}
    }
    // Compare boundaries in the caller's cropped coordinate system. Mapping an
    // exact event back through affine arithmetic can otherwise round below it.
    fn decode(self,t:f64)->Option<(usize,f64)> {
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {return None;}
        let steps=self.connection().steps;
        if t==0.0 || t==1.0 || self.from==self.to {
            return original_grid_sample(steps,self.original_fraction(t));
        }
        let span=self.to-self.from;let mut lo=0;let mut hi=steps;
        while lo<hi {
            let mid=lo+(hi-lo)/2+1;let at=mid as f64/steps as f64;
            if at<=self.from || at<=self.to && (at-self.from)/span<=t {lo=mid;}else {hi=mid-1;}
        }
        if lo==steps {return Some((lo,0.0));}
        let at=(lo as f64/steps as f64-self.from)/span;
        let next=((lo+1) as f64/steps as f64-self.from)/span;
        let fraction=((t-at)/(next-at)).clamp(0.0,1.0_f64.next_down());
        Some((lo,fraction))
    }
    fn crop(self,from:f64,to:f64)->Option<Self> {
        if !from.is_finite() || !to.is_finite() || from<0.0 || from>to || to>1.0 {return None;}
        let exact=|t| {
            let (index,fraction)=self.decode(t)?;
            let steps=self.connection().steps as f64;
            let at=index as f64/steps;
            // Affine rounding can put a just-left cut exactly on the next
            // gridpoint. Retain its decoded side at the nearest representable
            // original fraction; actual cut geometry is carried separately.
            Some(if fraction==0.0 {at}else {
                self.original_fraction(t).clamp(at.next_up(),((index+1) as f64/steps).next_down())
            })
        };
        Some(Self {from:exact(from)?,to:exact(to)?,..self})
    }
}
fn navigation_sample(map:&Tomogram,reference:Reference,xy:[f64;2])->NavigationGroundSample<'_> {
    match reference {
        Reference::Known(id)=>match map.source_ground_record(id) {
            Some(record)=>NavigationGroundSample::KnownNative {record,reference_xy:xy},
            None=>NavigationGroundSample::KnownLegacy {reference_xy:xy,
                height:map.reference_height_at(reference,xy).expect("recorded known sample")},
        },
        Reference::Unknown{height,..}=>NavigationGroundSample::Carried {height:f64::from_bits(height)},
        Reference::Unanchored{..}=>NavigationGroundSample::Unanchored,
    }
}
/// Original planned recordings, optionally retained as flat directed slices.
/// Neither an original recording nor a splice asserts actual layer or clearance.
#[derive(Clone,Debug)]
pub struct GroundCandidateReference {map:Arc<Tomogram>,recording:CandidateRecording}
impl GroundCandidateReference {
    fn leaf(&self,edge:usize)->Option<BorrowedSlice<'_>> {
        match &self.recording {
            CandidateRecording::Recorded(trace)=>trace.connections.get(edge)
                .map(|_|BorrowedSlice {trace,edge,from:0.0,to:1.0}),
            CandidateRecording::Slices(leaves)=>leaves.get(edge)
                .map(|v|BorrowedSlice {trace:&v.trace,edge:v.edge,from:v.from,to:v.to}),
        }
    }
    pub fn connection_count(&self)->usize {match &self.recording {
        CandidateRecording::Recorded(trace)=>trace.connections.len(),CandidateRecording::Slices(v)=>v.len(),
    }}
    /// Original recording geometry/grid for diagnostics. On a cropped edge,
    /// these are NOT newly sampled execution endpoints or execution steps.
    pub fn connection(&self,index:usize)->Option<([f64;2],[f64;2],usize)> {
        let c=self.leaf(index)?.connection();Some((c.from,c.to,c.steps))
    }
    pub fn original_range(&self,edge:usize)->Option<(usize,[f64;2])> {
        let v=self.leaf(edge)?;Some((v.edge,[v.from,v.to]))
    }
    pub fn original_sample(&self,edge:usize,fraction:f64)->Option<(usize,f64)> {self.leaf(edge)?.decode(fraction)}
    pub fn original_slice_sample(&self,edge:usize,from:f64,to:f64,progress:f64)->Option<(usize,f64)> {
        self.leaf(edge)?.crop(from,to)?.decode(progress)
    }
    pub fn walk_state_at(&self,edge:usize,fraction:f64,actual_xy:[f64;2])->Option<GroundWalkState<'_>> {
        if !actual_xy.into_iter().all(f64::is_finite) {return None;}
        if let CandidateRecording::Recorded(trace)=&self.recording {
            if trace.connections.is_empty() && edge==0 && fraction==0.0 {
                let (reference,reference_sample_xy,_)=trace.initial?;
                return Some(GroundWalkState {map:&self.map,reference,reference_sample_xy,xy:actual_xy});
            }
        }
        let leaf=self.leaf(edge)?;let (index,_)=leaf.decode(fraction)?;
        Some(GroundWalkState {map:&self.map,reference:leaf.reference(index),
            reference_sample_xy:leaf.connection().sample_xy(index),xy:actual_xy})
    }
    pub fn continuation_at(&self,edge:usize,fraction:f64,actual_xy:[f64;2])->Option<OwnedGroundWalkState> {
        let state=self.walk_state_at(edge,fraction,actual_xy)?;
        Some(OwnedGroundWalkState {map:Arc::clone(&self.map),reference:state.reference,
            reference_sample_xy:state.reference_sample_xy,actual_xy})
    }
    /// A diagnostic sample on the original leaf grid, including portions which
    /// a retained execution range may not consume.
    pub fn sample(&self,connection:usize,index:usize)->Option<PlannedGroundSample<'_>> {
        let leaf=self.leaf(connection)?;let c=leaf.connection();if index>c.steps {return None;}
        let reference=leaf.reference(index);let xy=c.sample_xy(index);
        let association=match reference {
            Reference::Known(id)=>PlannedGroundAssociation::Known {source:self.map.source_ground_record(id)},
            Reference::Unknown {..}=>PlannedGroundAssociation::Carried,
            Reference::Unanchored {..}=>PlannedGroundAssociation::Unanchored,
        };
        Some(PlannedGroundSample {point:GroundRoutePoint {xy,ground_z:self.map.reference_height_at(reference,xy)},association})
    }
    pub fn constant_association(&self,connection:usize)->bool {
        let Some(leaf)=self.leaf(connection)else{return false;};
        let c=leaf.connection();
        if leaf.from==0.0 && leaf.to==1.0 {
            // The unchanged full edge keeps its original constant-cost check.
            if c.changes.len()!=1 {return false;}
            let reference=leaf.trace.changes[c.changes.start].reference;
            let first=self.map.reference_height_at(reference,c.sample_xy(0));
            return first==self.map.reference_height_at(reference,c.sample_xy(1))
                && first==self.map.reference_height_at(reference,c.sample_xy(c.steps));
        }
        let (first,_)=leaf.decode(0.0).unwrap();let (last,_)=leaf.decode(1.0).unwrap();
        let reference=leaf.reference(first);
        let height=self.map.reference_height_at(reference,leaf.connection().sample_xy(first));
        (first..=last).all(|i|leaf.reference(i)==reference
            && self.map.reference_height_at(reference,leaf.connection().sample_xy(i))==height)
    }
    pub fn matches_source(&self,source:&SurfaceSnapshot)->bool {self.map.matches_surface(source)}
    /// Keep prefix and suffix leaves, without querying or re-recording either.
    pub fn prepend_to_suffix(&self,prefix:&Self,edge:usize,fraction:f64)->Result<Self,PlanningFailure> {
        if !Arc::ptr_eq(&self.map,&prefix.map) {return Err("splice reference owner changed".into());}
        let first=self.leaf(edge).and_then(|v|v.crop(fraction,1.0)).ok_or("invalid retained suffix address")?;
        let mut leaves=Vec::with_capacity(prefix.connection_count()+self.connection_count()-edge);
        for i in 0..prefix.connection_count() {leaves.push(prefix.leaf(i).unwrap().owned());}
        leaves.push(first.owned());
        for i in edge+1..self.connection_count() {leaves.push(self.leaf(i).unwrap().owned());}
        Ok(Self {map:Arc::clone(&self.map),recording:CandidateRecording::Slices(leaves.into())})
    }
    /// A boundary between separately recorded/cropped pieces is not discarded
    /// merely because its two geometric poses happen to be equal.
    pub fn is_splice_boundary(&self,right:usize)->bool {
        if right==0 {return false;}
        let Some(a)=self.leaf(right-1)else{return false;};let Some(b)=self.leaf(right)else{return false;};
        !Arc::ptr_eq(a.trace,b.trace) || a.edge+1!=b.edge || a.to!=1.0 || b.from!=0.0
    }
    /// Borrow original samples for actual physical geometry. No evaluator walk.
    pub fn with_body_navigation_slice<R>(&self,edge:usize,from:f64,to:f64,
        actual_from_xy:[f64;2],actual_to_xy:[f64;2],
        run:impl FnOnce(&BodyNavigationSegment<'_,'_>)->R)->Result<R,PlanningFailure> {
        let leaf=self.leaf(edge).and_then(|v|v.crop(from,to)).ok_or("invalid retained navigation slice")?;
        let source=self.map.source.as_ref().ok_or("recorded navigation has no physical source")?;
        let c=leaf.connection();let (first,_)=leaf.decode(0.0).unwrap();let (last,_)=leaf.decode(1.0).unwrap();
        let mut changes=Vec::new();let mut previous=(leaf.reference(first),
            self.map.reference_height_at(leaf.reference(first),c.sample_xy(first)));
        for index in first+1..=last {
            let next=(leaf.reference(index),self.map.reference_height_at(leaf.reference(index),c.sample_xy(index)));
            if next!=previous && leaf.to>leaf.from {
                changes.push((index as f64/c.steps as f64-leaf.from)/(leaf.to-leaf.from));
            }
            previous=next;
        }
        let sample=|t| {let (index,_)=leaf.decode(t).expect("bounded original sample");
            navigation_sample(&self.map,leaf.reference(index),c.sample_xy(index))};
        let navigation=BodyNavigationSegment::from_recorded(source,actual_from_xy,actual_to_xy,&changes,&sample);
        Ok(run(&navigation))
    }
    /// Explicit left→right transition at the same actual pose. The original H3
    /// kernel, not a new equality/height gate, decides whether it can be used.
    pub fn with_body_navigation_seam<R>(&self,right:usize,xy:[f64;2],
        run:impl FnOnce(&BodyNavigationSegment<'_,'_>)->R)->Result<R,PlanningFailure> {
        let left=right.checked_sub(1).and_then(|e|self.walk_state_at(e,1.0,xy)).ok_or("invalid splice left address")?;
        let right=self.walk_state_at(right,0.0,xy).ok_or("invalid splice right address")?;
        let source=self.map.source.as_ref().ok_or("recorded navigation has no physical source")?;
        let sample=|t| {let s=if t<1.0 {left}else {right};navigation_sample(s.map,s.reference,s.reference_sample_xy)};
        let changes=[1.0];let navigation=BodyNavigationSegment::from_recorded(source,xy,xy,&changes,&sample);
        Ok(run(&navigation))
    }
    pub fn end_matches(&self,original:GroundRoutePosition<'_>)->bool {
        let target=original.walk_state();
        if !std::ptr::eq(self.map.as_ref(),original.owner.map.as_ref()) {return false;}
        if self.connection_count()==0 {
            return self.walk_state_at(0,0.0,target.xy).is_some_and(|state|
                state.reference==target.reference && state.reference_sample_xy==target.reference_sample_xy
                    && match &self.recording {CandidateRecording::Recorded(t)=>t.initial.is_some_and(|v|v.2==target.xy),_=>false});
        }
        let leaf=self.leaf(self.connection_count()-1).unwrap();let c=leaf.connection();
        let xy=if leaf.to==1.0 {c.to}else {std::array::from_fn(|i|c.from[i]+leaf.to*(c.to[i]-c.from[i]))};
        self.walk_state_at(self.connection_count()-1,1.0,xy).is_some_and(|s|s.reference==target.reference && xy==target.xy)
    }
}

#[cfg(test)]
#[path="ground_candidate_tests.rs"]
mod candidate_tests;
