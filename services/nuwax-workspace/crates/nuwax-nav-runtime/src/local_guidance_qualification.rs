//! One optional observer, at most one pinned real source. No controller access.
//! Capturing/replaying is not a live A/B and never publishes the replay result.
#![allow(missing_docs)]
use crate::route_planning::{body, PlanRequest};
use crate::SurfaceSnapshot;
use nuwax_nav_pct::Tomogram;
use nuwax_nav_scan::{BodyPose, LocalPlanError, LocalPlanFailureKind, LocalTrajectory,
    ProgressPrefixPolicy, ScanConfig, local_guidance_qualification as scan_observation};
use nuwax_nav_types_messages::{MotionKinematics, Path3D};
use std::sync::{Arc, Mutex, Weak};
use std::time::Instant;

static ACTIVE:Mutex<Option<Weak<Mutex<State>>>>=Mutex::new(None);
#[derive(Default,Debug)]
/// Counts stop when the first eligible snapshot is retained. No request history.
pub struct Counts {
    pub target_calls:usize, pub nominal:usize, pub global_curved:usize,
    pub remaining_search:usize, pub cancelled:usize, pub eligible:usize, pub reference_misses:usize,
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum CaptureWindow { CurvedNominalHeuristic, FirstPartialBranch }
struct State { goal:[u64;3], window:CaptureWindow, counts:Counts, captured:Option<Captured> }
/// One scoped observer. Run the ignored selector alone with --test-threads=1.
pub struct CaptureSession { state:Arc<Mutex<State>> }
impl CaptureSession {
    pub fn start(goal:[f64;3])->Self {
        Self::start_window(goal,CaptureWindow::CurvedNominalHeuristic)
    }
    /// First actual partial result for this target; no h/fallback/source-stage gate.
    pub fn first_partial_branch(goal:[f64;3])->Self {
        Self::start_window(goal,CaptureWindow::FirstPartialBranch)
    }
    fn start_window(goal:[f64;3],window:CaptureWindow)->Self {
        let mut active=ACTIVE.lock().unwrap();
        assert!(active.as_ref().and_then(Weak::upgrade).is_none(),"qualification already active");
        let state=Arc::new(Mutex::new(State {goal:goal.map(f64::to_bits),window,counts:Counts::default(),captured:None}));
        *active=Some(Arc::downgrade(&state)); Self {state}
    }
    /// Take evidence only after owners have shut down, so replays add no live CPU load.
    pub fn finish(self)->(Counts,Option<Captured>) {
        let mut state=self.state.lock().unwrap();
        (std::mem::take(&mut state.counts),state.captured.take())
    }
}
impl Drop for CaptureSession {
    fn drop(&mut self) {
        let mut active=ACTIVE.lock().unwrap();
        if active.as_ref().and_then(Weak::upgrade).is_some_and(|state|Arc::ptr_eq(&state,&self.state)) {*active=None;}
    }
}
pub(crate) struct Token { state:Arc<Mutex<State>>, started:Instant }
pub(crate) fn begin(r:&PlanRequest)->Option<Token> {
    let state=ACTIVE.lock().unwrap().as_ref()?.upgrade()?;
    let target=r.reference_source.as_ref()?.goal.as_ref()?.pose.position;
    { let guard=state.lock().unwrap();
      if guard.captured.is_some() || guard.goal!=[target.x,target.y,target.z].map(f64::to_bits) {return None;} }
    scan_observation::reset(); Some(Token {state,started:Instant::now()})
}
#[derive(Clone,Debug)]
pub struct Output {
    pub partial:Option<bool>, pub poses:Vec<BodyPose>, pub smoothed:Option<bool>,
    pub error:Option<LocalPlanError>, pub elapsed_ms:f64,
}
impl Output {
    fn from_result(result:&Result<LocalTrajectory,LocalPlanError>)->Self {
        match result {
            Ok(t)=>Self {partial:Some(t.partial),poses:t.poses.clone(),smoothed:Some(t.smoothed),error:None,elapsed_ms:t.elapsed_ms},
            Err(e)=>Self {partial:None,poses:Vec::new(),smoothed:None,error:Some(e.clone()),elapsed_ms:0.0},
        }
    }
}
/// Exactly one immutable input; no synthesized reference, source, pose or motion.
pub struct Captured {
    pub started:Instant, pub finished:Instant, pub generation:u64, pub window:CaptureWindow,
    /// Original request owner, retained for the external source ledger.
    pub source:Arc<SurfaceSnapshot>,
    /// Clone of the actual private map passed to Scan in compute_inner.
    /// fork_for_planning preserves immutable inputs, not snapshot-owner identity.
    pub planning_source:SurfaceSnapshot,
    pub tomogram:Arc<Tomogram>, pub global_route:Arc<Path3D>, pub pose:nuwax_nav_types_messages::PoseStamped, pub start:BodyPose,
    /// Materialized request input before this branch's PCT/recent guidance.
    pub request_reference:Vec<[f64;3]>,
    /// Exact normalized reference supplied to this Scan call.
    pub reference:Vec<[f64;3]>, pub boundary:Arc<Vec<[f64;2]>>,
    pub motion:MotionKinematics, pub config:ScanConfig,
    pub final_yaw:Option<f64>, pub arrival:Option<([f64;3],f64)>,
    pub progress_reference:Option<Vec<[f64;3]>>, pub minimum_progress:Option<f64>,
    pub reference_progress:f64, pub reference_end:f64,
    pub request_detour:bool, pub branch_detour:bool, pub request_straight:bool,
    pub follow_reference:bool, pub prefer_straight:bool,
    pub original:Output, pub observation:scan_observation::Observation,
}
pub(crate) fn finish(token:Option<Token>,r:&PlanRequest,map:&SurfaceSnapshot,tomogram:&Arc<Tomogram>,
    reference:&[[f64;3]],motion:&MotionKinematics,detour:bool,follow_reference:bool,prefer_straight:bool,
    progress:Option<ProgressPrefixPolicy<'_>>,result:&Result<LocalTrajectory,LocalPlanError>) {
    let Some(token)=token else {return;};
    let observation=scan_observation::take();
    finish_with_observation(token,r,map,tomogram,reference,motion,detour,follow_reference,
        prefer_straight,progress,result,observation);
}
// A probe Miss is neither a partial result nor a failed full search. Consume its
// phase records without feeding either existing capture-admission transition.
pub(crate) fn finish_reference_miss(token:Option<Token>) {
    let Some(token)=token else {return;};
    let _observation=scan_observation::take();
    let mut state=token.state.lock().unwrap();
    state.counts.target_calls+=1;
    state.counts.reference_misses+=1;
}
// Production finish supplies only the actual Scan observation above. Keeping this
// private transition separate lets unit tests exercise storage with explicitly
// synthetic admission counters, without relaxing a real global h qualification.
fn finish_with_observation(token:Token,r:&PlanRequest,map:&SurfaceSnapshot,tomogram:&Arc<Tomogram>,
    reference:&[[f64;3]],motion:&MotionKinematics,detour:bool,follow_reference:bool,prefer_straight:bool,
    progress:Option<ProgressPrefixPolicy<'_>>,result:&Result<LocalTrajectory,LocalPlanError>,
    observation:scan_observation::Observation) {
    let mut state=token.state.lock().unwrap();
    state.counts.target_calls+=1;
    match state.window {
        CaptureWindow::CurvedNominalHeuristic => {
            if detour || follow_reference {return;}
            state.counts.nominal+=1;
            if r.straight {return;}
            state.counts.global_curved+=1;
            if !observation.phases.iter().any(|p|p.phase=="search_remaining") {return;}
            state.counts.remaining_search+=1;
            if result.as_ref().err().is_some_and(|e|e.kind==LocalPlanFailureKind::Cancelled) {
                state.counts.cancelled+=1;return;
            }
            // Root-only success/failure has no non-root ordering contest. Keep it out.
            if observation.work[0]<=1 || prefer_straight
                || r.scan.search_expansion_limit!=nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT {return;}
        }
        CaptureWindow::FirstPartialBranch => {
            if !result.as_ref().is_ok_and(|trajectory|trajectory.partial) {return;}
        }
    }
    state.counts.eligible+=1;
    if state.captured.is_some() {return;}
    let Some(global)=r.reference_source.as_ref() else {return;};
    // compute_inner passes this pair directly: r.source is the source owner;
    // map is its actual fork_for_planning view used by the Scan call. Retain both
    // by construction, not by an owner-equality assertion or partial-field proof.
    // The existing fork contract shares immutable inputs and isolates caches.
    state.captured=Some(Captured {started:token.started,finished:Instant::now(),generation:r.generation,window:state.window,
        source:r.source.clone(),planning_source:map.clone(),tomogram:tomogram.clone(),global_route:global.route.clone(),pose:r.pose.clone(),
        start:body(&r.pose),request_reference:r.reference.clone(),reference:reference.to_vec(),boundary:r.boundary.clone(),motion:motion.clone(),config:r.scan.clone(),
        final_yaw:r.final_yaw,arrival:r.arrival,progress_reference:progress.map(|p|p.reference.to_vec()),
        minimum_progress:progress.map(|p|p.minimum_distance_m),reference_progress:r.progress,reference_end:r.end,
        request_detour:r.detour,branch_detour:detour,request_straight:r.straight,
        follow_reference,prefer_straight,original:Output::from_result(result),observation});
}
impl Captured {
    /// Frozen, offline normal-Scan replay. The single changed argument is h's existing flag.
    pub fn replay(&self,follow_reference:bool)->(Output,scan_observation::Observation) {
        assert_eq!(self.window,CaptureWindow::CurvedNominalHeuristic,
            "FirstPartialBranch is evidence only, not an h replay qualification");
        assert!(!self.follow_reference && !self.prefer_straight);
        assert_eq!(self.config.search_expansion_limit,nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT);
        let map=self.planning_source.fork_for_planning();
        // A fresh cache owner over the captured immutable inputs. It intentionally
        // does not compare equal under same_body_query_view's owner-identity test.
        let progress=self.progress_reference.as_deref().zip(self.minimum_progress)
            .map(|(reference,minimum_distance_m)|ProgressPrefixPolicy {reference,minimum_distance_m});
        scan_observation::reset();
        let began=Instant::now();
        let result=nuwax_nav_scan::plan_with_progress(&map,&self.tomogram,self.start,&self.reference,
            &self.motion,&self.config,self.final_yaw,self.arrival,&self.boundary,
            follow_reference,self.prefer_straight,progress);
        let mut output=Output::from_result(&result);
        output.elapsed_ms=began.elapsed().as_secs_f64()*1000.0;
        let observation=scan_observation::take();
        self.audit_output(&output).expect("normal Scan output broke original geometric/endpoint contract");
        (output,observation)
    }
    pub fn audit_output(&self,output:&Output)->Result<(),String> {
        if output.error.is_some() {return Ok(());}
        let first=*output.poses.first().ok_or("empty successful candidate")?;
        let end=*output.poses.last().unwrap();
        if first.0.map(f64::to_bits)!=self.start.0.map(f64::to_bits)
            || first.1.map(f64::to_bits)!=self.start.1.map(f64::to_bits) {return Err("changed exact start".into());}
        let map=self.planning_source.fork_for_planning();
        map.validate_body_pose(first.0,first.1).map_err(|e|e.to_string())?;
        for p in output.poses.windows(2) {map.validate_body_motion(p[0],p[1],&self.boundary).map_err(|e|e.to_string())?;}
        if output.partial==Some(false) {
            if let Some((goal,tolerance))=self.arrival {
                let d=(0..3).map(|i|(end.0[i]-goal[i]).powi(2)).sum::<f64>().sqrt();
                if d>=tolerance {return Err("full result outside unchanged arrival ball".into());}
                if let Some(target)=self.final_yaw {
                    let q=end.1; let yaw=(2.0*(q[3]*q[2]+q[0]*q[1])).atan2(1.0-2.0*(q[1]*q[1]+q[2]*q[2]));
                    if (yaw-target).sin().atan2((yaw-target).cos()).abs()>=0.1 {return Err("full result lost terminal yaw".into());}
                }
            } else if !scan_observation::full_handoff(&map,self.start,end,&self.reference,&self.config) {
                return Err("full result lost original intermediate handoff".into());
            }
        }
        // A partial stays partial; its unchanged normal Scan progress policy is not an arrival claim.
        Ok(())
    }
}

#[cfg(test)]
mod synthetic_capture_replay_seam_tests {
    use super::*;
    use crate::route_planning::ReferenceSource;
    use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::{GoalOptions, MotionModel, NavFrameId, Point3, Pose3,
        PoseStamped, Quaternion, Timestamp};
    use std::collections::BTreeMap;

    // Reuse the runtime's ordinary clear surface; no tuned obstacle/fallback case.
    // Only the capture-admission phase/work records below are synthetic.
    fn fixture()->(PlanRequest,MotionKinematics) {
        let source=crate::surface_follower::test_snapshot(0.0,false)
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,
                max_step_up_m:0.2,max_step_down_m:0.2,max_cross_slope_deg:18.0,
                support_uncertainty_m:0.0,
            }).unwrap();
        let mut observations=SurfaceObservations::new(source.source_voxel_size_m(),32).unwrap();
        let mut blocks=BTreeMap::<_,BTreeMap<_,_>>::new();
        for (x,y,z) in source.cell_keys() {
            let key=[x,y,z];
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation {observed_ns:1_000_000_000,hits:2});
        }
        for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
        let source=Arc::new(source.with_observations(Some(Arc::new(observations)))
            .with_recent_obstacle_points(1,1_000_000_000,0,&[0],
                [(0,[0.55,1.05,0.05])]).unwrap());
        let pose=|x,y|PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.5),orientation:Quaternion::IDENTITY}};
        let route=Arc::new(Path3D {poses:vec![pose(0.5,1.0),pose(1.5,1.05),pose(2.5,1.0)]});
        let mut cumulative=vec![0.0];
        for pair in route.poses.windows(2) {
            cumulative.push(cumulative.last().unwrap()+pair[0].pose.position.distance(pair[1].pose.position));
        }
        let mut request=PlanRequest {source,pose:route.poses[0].clone(),boundary:Arc::new(vec![]),
            reference:vec![],reference_source:Some(ReferenceSource {route:route.clone(),
                cumulative:Arc::new(cumulative),goal:route.poses.last().cloned(),
                options:GoalOptions {allow_reverse:false,position_tolerance_m:0.35,yaw_tolerance_rad:0.15},
                offset:0.4,progress:0.0}),final_yaw:None,arrival:None,end:0.0,progress:0.0,
            generation:1,straight:false,detour:false,scan:ScanConfig::default()};
        request.materialize_reference().unwrap();
        assert!(!request.straight);
        let motion=MotionKinematics {model:MotionModel::Differential,max_forward_mps:0.5,
            max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:0.8,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true};
        (request,motion)
    }
    fn local_token(request:&PlanRequest)->(Arc<Mutex<State>>,Token) {
        let goal=request.reference_source.as_ref().unwrap().goal.as_ref().unwrap().pose.position;
        let state=Arc::new(Mutex::new(State {goal:[goal.x,goal.y,goal.z].map(f64::to_bits),
            window:CaptureWindow::CurvedNominalHeuristic,counts:Counts::default(),captured:None}));
        // Do not register with ACTIVE or influence another test/real capture session.
        let token=Token {state:state.clone(),started:Instant::now()};
        (state,token)
    }
    #[test]
    fn reference_miss_is_counted_without_capture_or_failure() {
        let (request,_)=fixture();
        for window in [CaptureWindow::CurvedNominalHeuristic,CaptureWindow::FirstPartialBranch] {
            let (state,token)=local_token(&request);
            state.lock().unwrap().window=window;
            scan_observation::reset();
            finish_reference_miss(Some(token));
            let state=state.lock().unwrap();
            assert_eq!(state.counts.target_calls,1);
            assert_eq!(state.counts.reference_misses,1);
            assert_eq!(state.counts.eligible,0);
            assert_eq!(state.counts.cancelled,0);
            assert_eq!(state.counts.remaining_search,0);
            assert!(state.captured.is_none());
        }
    }
    #[test]
    fn real_begin_and_probe_miss_preserve_observation_ownership() {
        // Reuse the existing clear fixture and the existing excluded-boundary
        // drain setup. This is an observer lifecycle test, not a partial-route
        // benefit witness or captured whole-kernel replay.
        let (mut request,motion)=fixture();
        request.boundary=Arc::new(vec![[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
        let map=request.source.fork_for_planning();
        let tomogram=Arc::new(Tomogram::local_from_surface(&map,
            [request.pose.pose.position.x,request.pose.pose.position.y],request.scan.horizon_m+1.5).unwrap());
        assert_eq!(request.scan.search_expansion_limit,nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT);
        let full=||nuwax_nav_scan::plan_with_progress(&map,&tomogram,body(&request.pose),
            &request.reference,&motion,&request.scan,request.final_yaw,request.arrival,
            &request.boundary,false,false,None);
        scan_observation::reset();
        assert_eq!(full().unwrap_err().kind,LocalPlanFailureKind::SearchExhausted);
        // No ownership token: must leave another consumer's real observation.
        finish_reference_miss(None);
        let unrelated=scan_observation::take();
        assert!(unrelated.work[0]>0,"seed must run real local search, not fabricate WORK");
        assert!(unrelated.phases.iter().any(|phase|phase.phase=="search_remaining"));
        // Seed once more so both nonzero WORK and real phases are pending when
        // the actual begin path resets them. No diagnostic counter injection.
        assert_eq!(full().unwrap_err().kind,LocalPlanFailureKind::SearchExhausted);
        let goal=request.reference_source.as_ref().unwrap().goal.as_ref().unwrap().pose.position;
        let session=CaptureSession::first_partial_branch([goal.x,goal.y,goal.z]);
        let token=begin(&request).expect("same-goal real begin must own this call");
        let cleared=scan_observation::take();
        assert_eq!(cleared.work,[0;5]);
        assert!(cleared.phases.is_empty(),"begin must clear prior phases before probe's own Scope");
        let result=nuwax_nav_scan::probe_reference_candidates(&map,&tomogram,body(&request.pose),
            &request.reference,&motion,&request.scan,request.final_yaw,request.arrival,
            &request.boundary,true,false,None);
        assert!(matches!(result,Ok(nuwax_nav_scan::ReferenceCandidateOutcome::Miss)),"{result:?}");
        finish_reference_miss(Some(token));
        let consumed=scan_observation::take();
        assert_eq!(consumed.work,[0;5],"probe must not inherit previous N search work");
        assert!(consumed.phases.is_empty(),"owned Miss finish must consume its real probe phases");
        let (counts,captured)=session.finish();
        assert_eq!(counts.target_calls,1);
        assert_eq!(counts.reference_misses,1);
        assert_eq!(counts.eligible,0);
        assert!(captured.is_none(),"a real reference Miss is not a partial result");
    }
    fn synthetic_remaining_observation()->scan_observation::Observation {
        scan_observation::Observation {
            phases:vec![scan_observation::PhaseRecord {phase:"search_remaining",
                exit:"synthetic_capture_replay_seam",selection:"synthetic_no_algorithm_claim",
                complete:false,elapsed_ms:0.0,calls:[0;3],milliseconds:[0.0;3]}],
            work:[2,0,0,0,0],
        }
    }
    fn real_scan_on_fork(request:&PlanRequest,motion:&MotionKinematics)
        ->(SurfaceSnapshot,Arc<Tomogram>,Result<LocalTrajectory,LocalPlanError>) {
        let map=request.source.fork_for_planning();
        assert!(!request.source.same_body_query_view(&map),"a shared clone would mask the regression");
        assert!(!request.source.same_geometry(&map));
        assert!(!Arc::ptr_eq(request.source.observations().unwrap(),map.observations().unwrap()));
        let tomogram=Arc::new(Tomogram::local_from_surface(&map,
            [request.pose.pose.position.x,request.pose.pose.position.y],request.scan.horizon_m+1.5).unwrap());
        let progress=Some(ProgressPrefixPolicy {reference:&request.reference,minimum_distance_m:0.2});
        scan_observation::reset();
        let result=nuwax_nav_scan::plan_with_progress(&map,&tomogram,body(&request.pose),
            &request.reference,motion,&request.scan,request.final_yaw,request.arrival,
            &request.boundary,false,false,progress);
        let _actual_observation=scan_observation::take();
        assert!(result.is_ok(),"normal Scan must return real geometry for this seam: {result:?}");
        (map,tomogram,result)
    }
    #[test]
    fn synthetic_admission_captures_real_fork_and_replays_normal_scan() {
        let (request,motion)=fixture();
        let (map,tomogram,result)=real_scan_on_fork(&request,&motion);
        let (state,token)=local_token(&request);
        finish_with_observation(token,&request,&map,&tomogram,&request.reference,&motion,
            false,false,false,Some(ProgressPrefixPolicy {reference:&request.reference,minimum_distance_m:0.2}),
            &result,synthetic_remaining_observation());
        let captured={let mut state=state.lock().unwrap();
            assert_eq!(state.counts.eligible,1,"synthetic seam must reach the storage assignment");
            state.captured.take().expect("capture storage branch was not exercised")};
        assert!(Arc::ptr_eq(&captured.source,&request.source));
        assert!(captured.planning_source.same_body_query_view(&map),"retain the actual Scan fork");
        assert!(!captured.planning_source.same_body_query_view(&request.source));
        assert_eq!(captured.progress_reference.as_deref(),Some(request.reference.as_slice()));
        assert_eq!(captured.minimum_progress,Some(0.2));
        let (baseline,_)=captured.replay(false);
        let (guided,_)=captured.replay(true);
        assert!(baseline.error.is_none() && guided.error.is_none());
        assert_eq!(captured.original.partial,baseline.partial);
        assert_eq!(captured.original.error,baseline.error); // Existing LocalPlanError equality only.
        let bits=|poses:&[BodyPose]|poses.iter().map(|p|(p.0.map(f64::to_bits),p.1.map(f64::to_bits))).collect::<Vec<_>>();
        assert_eq!(bits(&captured.original.poses),bits(&baseline.poses));
        eprintln!("SYNTHETIC_CAPTURE_REPLAY_SEAM real_fork=true normal_scan=true replay_flags=false,true synthetic_admission=true real_global_h_qualified=false");
    }
    #[test]
    fn synthetic_seam_without_remaining_phase_does_not_capture() {
        let (request,motion)=fixture();
        let (map,tomogram,result)=real_scan_on_fork(&request,&motion);
        let (state,token)=local_token(&request);
        finish_with_observation(token,&request,&map,&tomogram,&request.reference,&motion,
            false,false,false,Some(ProgressPrefixPolicy {reference:&request.reference,minimum_distance_m:0.2}),
            &result,scan_observation::Observation::default());
        let state=state.lock().unwrap();
        assert_eq!(state.counts.global_curved,1);
        assert_eq!(state.counts.remaining_search,0);
        assert_eq!(state.counts.eligible,0);
        assert!(state.captured.is_none(),"the real qualification gate must not be weakened for this seam");
    }
}
