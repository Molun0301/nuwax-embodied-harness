//! Explicit Q1-R qualification support. Child of route_follower under the existing
//! surface-recovery-qualification feature. No production scheduling change.
#![allow(missing_docs)]
use super::*;
use super::route_change_scope_qualification::ActualCandidate;
use std::sync::mpsc;

#[derive(Clone)]
pub struct SubmittedSource {
    pub id: u64,
    pub source: Arc<SurfaceSnapshot>,
    pub pose: PoseStamped,
    pub detour: bool,
    pub generation: u64,
}
#[derive(Clone)]
pub struct KernelEntry {
    // This is a compute ordinal, deliberately NOT a request id.
    pub compute_ordinal: u64,
    pub source: Arc<SurfaceSnapshot>,
    pub pose: PoseStamped,
    pub detour: bool,
    pub generation: u64,
    pub epoch: u64,
}
impl KernelEntry {
    pub fn matches_submission(&self, submitted: &SubmittedSource) -> bool {
        Arc::ptr_eq(&self.source, &submitted.source)
            && self.pose.frame_id == submitted.pose.frame_id
            && pose_bits(body(&self.pose)) == pose_bits(body(&submitted.pose))
            && self.pose.stamp == submitted.pose.stamp
            && self.detour == submitted.detour && self.generation == submitted.generation
    }
}
pub struct Computed {
    pub entry: KernelEntry,
    pub actual: Result<ActualCandidate, Vec<String>>,
}
pub struct TicketState {
    pub id: u64,
    pub basis: Option<Arc<SurfaceSnapshot>>,
    pub columns: usize,
    pub batches: usize,
    pub unknown: bool,
}
pub struct PendingState {
    pub epoch: u64,
    pub submitted: Option<(u64,Point3,bool)>,
    pub tickets: Vec<TicketState>,
    pub map: Arc<SurfaceSnapshot>,
    pub pose: PoseStamped,
    pub retry_after: Instant,
    pub dirty: bool,
    pub calls: u64,
    pub plans: u64,
    pub generation: u64,
    pub admission: Option<u64>,
}
#[derive(Debug)]
pub struct EdgeAudit {
    pub old_all_valid: bool,
    pub latest_first_rejection: Option<(usize,String)>,
}
pub struct PendingSourceCase {
    follower: Option<RouteFollower>,
    release: Option<mpsc::Sender<()>>,
    entered: mpsc::Receiver<KernelEntry>,
    computed: mpsc::Receiver<Computed>,
    returned: mpsc::Receiver<(u64,bool,bool)>,
    submissions: Vec<SubmittedSource>,
    began: Instant,
}
impl PendingSourceCase {
    pub fn start(config: SurfaceFollowerConfig, source: Arc<SurfaceSnapshot>,
        pose: PoseStamped, route: Path3D, goal: PoseStamped) -> Result<Self,String> {
        let mut follower=RouteFollower::new(config.clone()).map_err(|e|e.to_string())?;
        follower.update_execution_map(ExecutionMap::Surface(source)).map_err(|e|e.to_string())?;
        follower.update_odometry(pose);
        follower.configure_goal(&goal,GoalOptions::default()).map_err(|e|e.to_string())?;
        follower.replace_execution_path(ExecutionPath::Surface(route),None,goal).map_err(|e|e.to_string())?;
        if let Some(reason)=follower.controller.current_body_error() {return Err(reason);}
        let (entry_tx,entered)=mpsc::channel();
        let (done_tx,computed)=mpsc::channel();
        let (release,released)=mpsc::channel();
        let (return_tx,returned)=mpsc::channel();
        let began=Instant::now();
        let mut kernel=PlanKernel::new(config);
        let mut compute_ordinal=0;
        // As in stage39, only an idle, never-submitted initial owner is replaced.
        follower.owner=Some(PlanningOwner::new(move |request:PlanRequest,cancel| {
            compute_ordinal+=1;
            let entry=KernelEntry {compute_ordinal,source:request.source.clone(),pose:request.pose.clone(),
                detour:request.detour,generation:request.generation,epoch:cancel.expected};
            eprintln!("Q1_KERNEL_START t_ms={} ordinal={} source={:p} pose_x={} detour={} epoch={}",
                began.elapsed().as_secs_f64()*1000.0,compute_ordinal,Arc::as_ptr(&entry.source),
                entry.pose.pose.position.x,entry.detour,entry.epoch);
            let _=entry_tx.send(entry.clone());
            let compute_began=Instant::now();
            let result=kernel.compute(request,cancel);
            let actual=match &result.outcome {
                Ok(candidate)=>Ok(ActualCandidate {source:result.request.source.clone(),path:candidate.path.clone(),
                    partial:candidate.partial,progress:result.request.progress,end:result.request.end,
                    horizon:result.request.scan.horizon_m,epoch:cancel.expected,generation:result.request.generation,
                    kernel_elapsed_ms:compute_began.elapsed().as_secs_f64()*1000.0}),
                Err(errors)=>Err(errors.iter().map(ToString::to_string).collect()),
            };
            eprintln!("Q1_KERNEL_DONE t_ms={} ordinal={} successful={}",
                began.elapsed().as_secs_f64()*1000.0,compute_ordinal,actual.is_ok());
            let _=done_tx.send(Computed {entry,actual});
            // Never hold before actual kernel compute. All branches return the
            // unmodified result through the original owner's publication gate.
            let explicit_release=released.recv_timeout(Duration::from_secs(30)).is_ok();
            let _=return_tx.send((compute_ordinal,cancel.cancelled(),explicit_release));
            result
        }).map_err(|e|e.to_string())?);
        let mut case=Self {follower:Some(follower),release:Some(release),entered,computed,returned,
            submissions:Vec::new(),began};
        case.follower.as_mut().unwrap().schedule(false);
        case.record_submission(0)?;
        if case.submissions.len()!=1 {return Err("initial real A request was not accepted".into());}
        Ok(case)
    }
    fn record_submission(&mut self, previous_next:u64)->Result<(),String> {
        let f=self.follower.as_ref().unwrap();
        if f.next_request==previous_next {return Ok(());}
        let (id,_,detour)=f.submitted.ok_or("accepted request lacks submitted receipt")?;
        if id!=f.next_request {return Err("next_request/submitted mismatch".into());}
        let observed=SubmittedSource {id,source:f.map.as_ref().ok_or("missing source")?.clone(),
            pose:f.pose.as_ref().ok_or("missing pose")?.clone(),detour,generation:f.route_generation};
        eprintln!("Q1_SUBMIT_SCHEDULE_RETURN t_ms={} id={} source={:p} pose_x={} detour={} tickets={:?}",
            self.began.elapsed().as_secs_f64()*1000.0,id,Arc::as_ptr(&observed.source),
            observed.pose.pose.position.x,detour,f.source_tickets.iter().flatten().map(|x|x.0).collect::<Vec<_>>());
        self.submissions.push(observed);Ok(())
    }
    pub fn submissions(&self)->&[SubmittedSource] {&self.submissions}
    pub fn state(&self)->PendingState {
        let f=self.follower.as_ref().unwrap();
        PendingState {epoch:f.owner.as_ref().unwrap().epoch(),submitted:f.submitted,
            tickets:f.source_tickets.iter().flatten().map(|(id,t)|TicketState {id:*id,basis:t.basis.clone(),
                columns:t.columns,batches:t.changes.len(),unknown:t.unknown}).collect(),
            map:f.map.as_ref().unwrap().clone(),pose:f.pose.as_ref().unwrap().clone(),retry_after:f.retry_after,
            dirty:f.dirty,calls:f.local_planning_calls,plans:f.local_plans,generation:f.route_generation,
            admission:f.admitted_generation}
    }
    pub fn current_body_ok(&mut self)->bool {self.follower.as_mut().unwrap().controller.current_body_error().is_none()}
    pub fn refresh_distance(&self)->f64 {self.follower.as_ref().unwrap().scan.refresh_distance_m}
    pub fn publish(&mut self,update:SurfaceMapUpdate)->Result<(),String> {
        eprintln!("Q1_PUBLISH t_ms={} source={:p}",self.began.elapsed().as_secs_f64()*1000.0,Arc::as_ptr(&update.snapshot));
        self.follower.as_mut().unwrap().update_execution_map(ExecutionMap::SurfaceUpdate(update)).map_err(|e|e.to_string())
    }
    pub fn update_pose_with_body_check(&mut self,pose:PoseStamped)->Result<(),String> {
        let f=self.follower.as_mut().unwrap();
        let source=f.map.as_ref().ok_or("missing current map")?;
        let old=f.pose.as_ref().ok_or("missing old pose")?;
        source.validate_body_pose(body(old).0,body(old).1).map_err(|e|e.to_string())?;
        source.validate_body_pose(body(&pose).0,body(&pose).1).map_err(|e|e.to_string())?;
        source.validate_body_motion(body(old),body(&pose),&f.boundary).map_err(|e|e.to_string())?;
        if old.stamp.clock!=pose.stamp.clock || old.stamp.nanos>pose.stamp.nanos {
            return Err("pose input replay did not retain monotonic timestamps".into());
        }
        eprintln!("Q1_POSE_INPUT t_ms={} from_x={} to_x={} validated_body_motion=true physical_closed_loop=false",
            self.began.elapsed().as_secs_f64()*1000.0,old.pose.position.x,pose.pose.position.x);
        f.update_odometry(pose);Ok(())
    }
    pub fn tick(&mut self)->Result<PlannerStep,String> {
        let f=self.follower.as_mut().unwrap();
        let previous_next=f.next_request;
        let before:Vec<_>=f.source_tickets.iter().flatten().map(|x|x.0).collect();
        let step=f.step(0.3);
        let after:Vec<_>=f.source_tickets.iter().flatten().map(|x|x.0).collect();
        if before!=after {
            eprintln!("Q1_TICK_RETURN t_ms={} old_tickets={before:?} new_tickets={after:?} submitted={:?} plans={}",
                self.began.elapsed().as_secs_f64()*1000.0,f.submitted.map(|s|s.0),f.local_plans);
        }
        self.record_submission(previous_next)?;Ok(step)
    }
    pub fn tick_after_original_retry(&mut self)->Result<PlannerStep,String> {
        let deadline=self.state().retry_after;
        if let Some(remaining)=deadline.checked_duration_since(Instant::now()) {std::thread::sleep(remaining);}
        if Instant::now()<deadline {return Err("original retry deadline has not elapsed".into());}
        self.tick()
    }
    pub fn entered(&self)->Result<KernelEntry,String> {
        self.entered.recv_timeout(Duration::from_secs(30)).map_err(|e|format!("kernel entry missing: {e}"))
    }
    pub fn computed(&self)->Result<Computed,String> {
        self.computed.recv_timeout(Duration::from_secs(30)).map_err(|e|format!("kernel completion missing: {e}"))
    }
    pub fn release_actual(&mut self,ordinal:u64)->Result<(),String> {
        eprintln!("Q1_RELEASE_SEND t_ms={} ordinal={ordinal}",self.began.elapsed().as_secs_f64()*1000.0);
        self.release.as_ref().ok_or("missing barrier")?.send(()).map_err(|e|e.to_string())?;
        let (got,cancelled,explicit)=self.returned.recv_timeout(Duration::from_secs(30)).map_err(|e|e.to_string())?;
        eprintln!("Q1_RELEASE_ACK_BEFORE_OWNER_RETURN t_ms={} ordinal={got} cancelled={cancelled} explicit={explicit} owner_publish_not_observed=true",
            self.began.elapsed().as_secs_f64()*1000.0);
        if got!=ordinal || cancelled || !explicit {return Err(format!("invalid real return ordinal={got} cancelled={cancelled} explicit={explicit}"));}
        Ok(())
    }
    pub fn pump_until_consumed(&mut self,id:u64)->Result<(),String> {
        let until=Instant::now()+Duration::from_secs(5);
        loop {
            self.tick()?;
            if self.state().tickets.iter().all(|t|t.id!=id) {return Ok(());}
            if Instant::now()>=until {return Err(format!("real request {id} was not consumed"));}
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    pub fn audit_edges(&self,actual:&ActualCandidate,latest:&SurfaceSnapshot)->EdgeAudit {
        let boundary=&self.follower.as_ref().unwrap().boundary;
        EdgeAudit {old_all_valid:actual.path.poses.windows(2).all(|e|
            actual.source.validate_body_motion(body(&e[0]),body(&e[1]),boundary).is_ok()),
            latest_first_rejection:actual.path.poses.windows(2).enumerate().find_map(|(i,e)|
                latest.validate_body_motion(body(&e[0]),body(&e[1]),boundary).err().map(|e|(i,e.to_string())))}
    }
    pub fn finish(mut self)->PendingState {
        let state=self.state();self.cleanup();state
    }
    fn cleanup(&mut self) {
        if let Some(mut follower)=self.follower.take() {
            follower.stop();
            if let Some(release)=self.release.take() {let _=release.send(());let _=release.send(());}
            drop(follower); // Original Drop returns; it does not expose join status.
        }
    }
}
impl Drop for PendingSourceCase {fn drop(&mut self){self.cleanup();}}
