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
            trace:Arc::new(GroundCandidateTrace {connections,changes,initial})})
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
        match reference {
            Reference::Known(id)=>match self.from.map.source_ground_record(id) {
                Some(record)=>NavigationGroundSample::KnownNative {record,reference_xy:xy},
                None=>NavigationGroundSample::KnownLegacy {reference_xy:xy,
                    height:self.from.map.reference_height_at(reference,xy).expect("recorded known sample")},
            },
            Reference::Unknown{height,..}=>NavigationGroundSample::Carried {height:f64::from_bits(height)},
            Reference::Unanchored{..}=>NavigationGroundSample::Unanchored,
        }
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
/// The selected candidate's own planned walk. It does not assert actual layer or clearance.
#[derive(Clone,Debug)]
pub struct GroundCandidateReference {map:Arc<Tomogram>,trace:Arc<GroundCandidateTrace>}
impl GroundCandidateReference {
    pub fn connection_count(&self)->usize {self.trace.connections.len()}
    pub fn connection(&self,index:usize)->Option<([f64;2],[f64;2],usize)> {
        let c=self.trace.connections.get(index)?;Some((c.from,c.to,c.steps))
    }
    /// Decode an address on the retained evaluator grid. Slices and future
    /// continuations use this same right-continuous decoder.
    pub fn original_sample(&self,edge:usize,fraction:f64)->Option<(usize,f64)> {
        original_grid_sample(self.trace.connections.get(edge)?.steps,fraction)
    }
    /// Borrow an accepted candidate address for one control invocation. This
    /// does not move its cursor or choose another layer from actual geometry.
    pub fn walk_state_at(&self,edge:usize,fraction:f64,actual_xy:[f64;2])->Option<GroundWalkState<'_>> {
        if !actual_xy.into_iter().all(f64::is_finite) {return None;}
        if self.trace.connections.is_empty() && edge==0 && fraction==0.0 {
            let (reference,reference_sample_xy,_)=self.trace.initial?;
            return Some(GroundWalkState {map:&self.map,reference,reference_sample_xy,xy:actual_xy});
        }
        let c=self.trace.connections.get(edge)?;
        let (index,_)=original_grid_sample(c.steps,fraction)?;
        let reference=self.trace.changes[c.changes.clone()].iter().rev()
            .find(|change|change.sample_index<=index)?.reference;
        Some(GroundWalkState {map:&self.map,reference,reference_sample_xy:c.sample_xy(index),xy:actual_xy})
    }
    /// Freeze only when the seed must outlive the borrowed original candidate.
    pub fn continuation_at(&self,edge:usize,fraction:f64,actual_xy:[f64;2])->Option<OwnedGroundWalkState> {
        let state=self.walk_state_at(edge,fraction,actual_xy)?;
        Some(OwnedGroundWalkState {map:Arc::clone(&self.map),reference:state.reference,
            reference_sample_xy:state.reference_sample_xy,actual_xy})
    }
    pub fn sample(&self,connection:usize,index:usize)->Option<PlannedGroundSample<'_>> {
        let c=self.trace.connections.get(connection)?;
        if index>c.steps {return None;}
        let reference=self.trace.changes[c.changes.clone()].iter().rev()
            .find(|change|change.sample_index<=index)?.reference;
        let xy=c.sample_xy(index);
        let association=match reference {
            Reference::Known(id)=>PlannedGroundAssociation::Known {source:self.map.source_ground_record(id)},
            Reference::Unknown {..}=>PlannedGroundAssociation::Carried,
            Reference::Unanchored {..}=>PlannedGroundAssociation::Unanchored,
        };
        Some(PlannedGroundSample {point:GroundRoutePoint {xy,ground_z:self.map.reference_height_at(reference,xy)},association})
    }
    pub fn constant_association(&self,connection:usize)->bool {
        self.trace.connections.get(connection).is_some_and(|c| {
            if c.changes.len()!=1 {return false;}
            let reference=self.trace.changes[c.changes.start].reference;
            let first=self.map.reference_height_at(reference,c.sample_xy(0));
            first==self.map.reference_height_at(reference,c.sample_xy(1))
                && first==self.map.reference_height_at(reference,c.sample_xy(c.steps))
        })
    }
    pub fn matches_source(&self,source:&SurfaceSnapshot)->bool {self.map.matches_surface(source)}
    /// Source-local equality for joins, not a floating-height floor comparison.
    pub fn end_matches(&self,original:GroundRoutePosition<'_>)->bool {
        let target=original.walk_state();
        std::ptr::eq(self.map.as_ref(),original.owner.map.as_ref()) &&
            if self.trace.connections.is_empty() {
                self.trace.initial.is_some_and(|(reference,sample_xy,xy)|reference==target.reference
                    && sample_xy==target.reference_sample_xy && xy==target.xy)
            } else {
                self.trace.changes.last().is_some_and(|last|last.reference==target.reference)
                    && self.trace.connections.last().is_some_and(|last|last.to==target.xy)
            }
    }
}

#[cfg(test)]
#[path="ground_candidate_tests.rs"]
mod candidate_tests;
