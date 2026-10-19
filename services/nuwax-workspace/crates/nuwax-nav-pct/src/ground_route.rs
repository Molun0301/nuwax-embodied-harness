//! Opt-in planned Ground references. Public goal/wire defaults are not changed.
//! A planned association is neither an actual observation nor arrival evidence.
use crate::{PctPlanner, PlanningFailure, RoutePenalty, SearchLimits, SearchStats, Tomogram};
use crate::reference::Reference;
use crate::search::{RouteReferenceTrace, checkpoint};
use nuwax_nav_mls_planner::{SurfaceSnapshot, surface::SourceGroundRecord};
use nuwax_types::navigation::{GroundHeightIntent, GroundLayerResult};
use std::sync::{Arc, atomic::AtomicBool};

type BodyPose = ([f64;3],[f64;4]);

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
        let physical=|p:[f64;3]| if p==from {actual} else {
            ([actual.0[0]+(p[0]-start.xy[0]),actual.0[1]+(p[1]-start.xy[1]),
                actual.0[2]+if start.ground_z.is_some(){p[2]-initial_z}else{0.0}],actual.1)
        };
        let accepts=|a,b|body.validate_unanchored_motion(physical(a),physical(b),boundary,&mut None).is_ok();
        // Existing penalties are in actual body coordinates. Convert this one
        // captured snapshot once, without reading the planner's mutable costs.
        let penalties:Vec<_>=penalties.iter().map(|p|RoutePenalty {
            center:[p.center[0]-actual.0[0]+start.xy[0],p.center[1]-actual.0[1]+start.xy[1],
                p.center[2]-actual.0[2]+initial_z],radius:p.radius,weight:p.weight,
        }).collect();
        let traced=map.plan_ground_traced(from,to,xy_tolerance,boundary,cancelled,radius,&penalties,
            &accepts,limits,start.ground_z.is_none().then_some(actual.0[2]),
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
        let guide_body_poses=traced.route.points.iter().copied().map(physical).collect();
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
        let mut connections=Vec::with_capacity(segments.len());let mut changes=Vec::new();
        let mut previous:Option<GroundWalkState<'_>>=None;
        for segment in segments {
            if !std::ptr::eq(segment.from.map,self.map.as_ref()) {return Err("candidate reference owner changed".into());}
            if previous.is_some_and(|end|end.reference!=segment.from.reference || end.xy!=segment.from.xy) {
                return Err("candidate reference segments are disconnected".into());
            }
            previous=Some(segment.to);
            let start=changes.len();changes.extend(segment.trace.changes);
            connections.push(GroundWalkConnection {from:segment.from.xy,to:segment.to.xy,
                steps:segment.trace.steps,changes:start..changes.len()});
        }
        Ok(GroundCandidateReference {map:Arc::clone(&self.map),
            trace:Arc::new(GroundCandidateTrace {connections,changes})})
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
        let t=(self.sample as f64+self.fraction)/c.steps as f64;
        GroundWalkState {map:&self.owner.map,reference,
            xy:std::array::from_fn(|i|c.evaluated_from[i]+t*(c.evaluated_to[i]-c.evaluated_from[i]))}
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
#[derive(Clone,Copy,Debug)]
pub struct GroundWalkState<'a> {map:&'a Tomogram,reference:Reference,xy:[f64;2]}
#[derive(Clone,Copy,Debug,PartialEq,Eq,Hash)]
pub struct GroundWalkKey {owner:usize,reference:Reference,xy:[u64;2]}
impl<'a> GroundWalkState<'a> {
    pub fn xy(self)->[f64;2] {self.xy}
    pub fn semantic_key(self)->GroundWalkKey {
        GroundWalkKey {owner:std::ptr::from_ref(self.map) as usize,reference:self.reference,xy:self.xy.map(f64::to_bits)}
    }
    pub fn walk_xy(self,to:[f64;2],boundary:&[[f64;2]],cancelled:Option<&AtomicBool>)
        ->Result<GroundWalkSegment<'a>,PlanningFailure> {
        checkpoint(cancelled)?;
        let mut scratch=Some(crate::reference::SegmentTrace::default());
        let result=self.map.reference_walk_recorded::<true>(self.xy,to,None,boundary,cancelled,
            &mut scratch,Some(self.reference));
        checkpoint(cancelled)?;
        let (_,_,reference)=result.ok_or("candidate XY reference is incompatible with its source")?;
        Ok(GroundWalkSegment {from:self,to:Self {map:self.map,reference,xy:to},trace:scratch.unwrap()})
    }
}
#[derive(Debug)]
pub struct GroundWalkSegment<'a> {from:GroundWalkState<'a>,to:GroundWalkState<'a>,trace:crate::reference::SegmentTrace}
impl<'a> GroundWalkSegment<'a> {pub fn end(&self)->GroundWalkState<'a> {self.to}}
#[derive(Debug)]
struct GroundWalkConnection {from:[f64;2],to:[f64;2],steps:usize,changes:std::ops::Range<usize>}
#[derive(Debug)]
struct GroundCandidateTrace {connections:Vec<GroundWalkConnection>,changes:Vec<crate::reference::TraceChange>}
/// The selected candidate's own planned walk. It does not assert actual layer or clearance.
#[derive(Clone,Debug)]
pub struct GroundCandidateReference {map:Arc<Tomogram>,trace:Arc<GroundCandidateTrace>}
impl GroundCandidateReference {
    pub fn connection_count(&self)->usize {self.trace.connections.len()}
    pub fn connection(&self,index:usize)->Option<([f64;2],[f64;2],usize)> {
        let c=self.trace.connections.get(index)?;Some((c.from,c.to,c.steps))
    }
    pub fn sample(&self,connection:usize,index:usize)->Option<PlannedGroundSample<'_>> {
        let c=self.trace.connections.get(connection)?;
        if index>c.steps {return None;}
        let reference=self.trace.changes[c.changes.clone()].iter().rev()
            .find(|change|change.sample_index<=index)?.reference;
        let t=index as f64/c.steps as f64;
        let xy=std::array::from_fn(|i|c.from[i]+t*(c.to[i]-c.from[i]));
        let association=match reference {
            Reference::Known(id)=>PlannedGroundAssociation::Known {source:self.map.source_ground_record(id)},
            Reference::Unknown {..}=>PlannedGroundAssociation::Carried,
            Reference::Unanchored {..}=>PlannedGroundAssociation::Unanchored,
        };
        Some(PlannedGroundSample {point:GroundRoutePoint {xy,ground_z:self.map.reference_height_at(reference,xy)},association})
    }
    pub fn constant_association(&self,connection:usize)->bool {
        self.trace.connections.get(connection).is_some_and(|c|c.changes.len()==1)
    }
    pub fn matches_source(&self,source:&SurfaceSnapshot)->bool {self.map.matches_surface(source)}
    /// Source-local equality for joins, not a floating-height floor comparison.
    pub fn end_matches(&self,original:GroundRoutePosition<'_>)->bool {
        std::ptr::eq(self.map.as_ref(),original.owner.map.as_ref())
            && self.trace.changes.last().is_some_and(|last|last.reference==original.walk_state().reference)
            && self.trace.connections.last().is_some_and(|last|last.to==original.walk_state().xy)
    }
}

#[cfg(test)]
#[path="ground_candidate_tests.rs"]
mod candidate_tests;
