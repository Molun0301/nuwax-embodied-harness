//! Uncompiled qualification-only baseline draft. No production policy change.
//! The real kernel returns normally; readiness is read from the original owner
//! completion slot. The caller delays consumption only for this semantic MRE.
#![allow(missing_docs)]
use super::*;
use super::route_change_scope_qualification::{ActualCandidate, GeometryAudit, Lifecycle};
use super::route_pending_source_qualification::SubmittedSource;
use nuwax_nav_types_messages::Quaternion;
use std::sync::mpsc;

#[derive(Clone, Debug)]
pub struct CompletedSlotObservation {
    pub current_epoch: u64,
    pub completion_id: Option<u64>,
    pub completion_epoch: Option<u64>,
    pub active: bool,
    pub pending: Option<(u64, u64)>,
    pub closed: bool,
    pub faulted: bool,
    pub full: bool,
    pub receipt_matches: bool,
    pub mirror_matches: bool,
}
impl CompletedSlotObservation {
    pub fn is_exact_ready_full(&self, id: u64, epoch: u64) -> bool {
        self.completion_id == Some(id) && self.completion_epoch == Some(epoch)
            && self.current_epoch == epoch && !self.active && self.pending.is_none()
            && !self.closed && !self.faulted && self.full
            && self.receipt_matches && self.mirror_matches
    }
}

fn exact_pose_payload(left: &PoseStamped, right: &PoseStamped) -> bool {
    left.frame_id == right.frame_id && left.stamp == right.stamp
        && pose_bits(body(left)) == pose_bits(body(right))
}
fn exact_path_payload(left: &Path3D, right: &Path3D) -> bool {
    left.poses.len() == right.poses.len()
        && left.poses.iter().zip(&right.poses).all(|(a,b)|exact_pose_payload(a,b))
}

pub struct CompletedSlotFinished {
    pub after_step: Lifecycle,
    pub step: PlannerStep,
    pub old_ticket_absent: bool,
    pub old_submitted_absent: bool,
    pub old_slot_absent: bool,
    pub cleanup_returned: bool,
}
pub struct PublishedKernelCase {
    follower: Option<RouteFollower>,
    actual: Option<ActualCandidate>,
    receipt: Option<SubmittedSource>,
    route: Option<Arc<Path3D>>,
    goal: PoseStamped,
}
impl PublishedKernelCase {
    pub fn capture(config: SurfaceFollowerConfig, source: Arc<SurfaceSnapshot>,
        pose: PoseStamped, route: Path3D, goal: PoseStamped) -> Result<Self, String> {
        let follower = RouteFollower::new(config.clone()).map_err(|e|e.to_string())?;
        // From this point every ? / assertion unwind goes through our original
        // stop + owner Drop cleanup, including setup before any actual submit.
        let mut case = Self {follower:Some(follower), actual:None, receipt:None,
            route:None, goal:goal.clone()};
        let (send, receive) = mpsc::channel::<Result<ActualCandidate,String>>();
        {
            let f = case.follower.as_mut().unwrap();
            f.update_execution_map(ExecutionMap::Surface(source)).map_err(|e|e.to_string())?;
            f.update_odometry(pose);
            f.configure_goal(&goal, GoalOptions::default()).map_err(|e|e.to_string())?;
            f.replace_execution_path(ExecutionPath::Surface(route),None,goal)
                .map_err(|e|e.to_string())?;
            if let Some(reason) = f.controller.current_body_error() {
                return Err(format!("completion-slot fixture body invalid: {reason}"));
            }
            case.route = f.route.clone();
            let mut kernel = PlanKernel::new(config);
            // Only the initial idle, never-submitted owner is replaced. There
            // is no recv/wait/barrier in this compute closure, before or after
            // kernel return. The mirror copy is semantic audit overhead only.
            f.owner = Some(PlanningOwner::new(move |request: PlanRequest, cancel| {
                let began = Instant::now();
                let result = kernel.compute(request,cancel);
                let mirror = match &result.outcome {
                    Ok(candidate) => Ok(ActualCandidate {
                        source:Arc::clone(&result.request.source), path:candidate.path.clone(),
                        partial:candidate.partial, progress:result.request.progress,
                        end:result.request.end, horizon:result.request.scan.horizon_m,
                        epoch:cancel.expected, generation:result.request.generation,
                        kernel_elapsed_ms:began.elapsed().as_secs_f64()*1000.0,
                    }),
                    Err(errors) => Err(format!("real PlanKernel prerequisite failed: {errors:?}")),
                };
                let _ = send.send(mirror);
                result // original PlanningOwner performs the only publication
            }).map_err(|e|e.to_string())?);
            f.schedule(false);
            let (id,_,detour) = f.submitted.ok_or("normal schedule did not submit")?;
            case.receipt = Some(SubmittedSource {id,
                source:Arc::clone(f.map.as_ref().ok_or("submitted source absent")?),
                pose:f.pose.as_ref().ok_or("submitted pose absent")?.clone(),
                detour,generation:f.route_generation});
        }
        case.actual = Some(receive.recv_timeout(Duration::from_secs(30))
            .map_err(|e|format!("kernel mirror not received: {e}"))??);
        // A mirror receipt is deliberately NOT a completion receipt. Only this
        // original exchange-locked observation establishes ready-before-take.
        let deadline = Instant::now()+Duration::from_secs(5);
        loop {
            let slot = case.slot()?;
            let epoch = case.actual().epoch;
            if slot.is_exact_ready_full(case.id(),epoch) {
                eprintln!("COMPLETED_SLOT_READY id={} epoch={} current_epoch={} active=false pending=None full=true receipt_match=true mirror_all_pose_bits_frame_full_stamp_match=true poses={}",
                    case.id(),epoch,slot.current_epoch,case.actual().path.poses.len());
                break;
            }
            if slot.closed || slot.faulted || slot.completion_id.is_some() {
                return Err(format!("actual original completion slot failed prerequisite: {slot:?}"));
            }
            if Instant::now()>=deadline {
                return Err(format!("owner did not publish exact full completion: {slot:?}"));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        Ok(case)
    }
    pub fn id(&self) -> u64 {self.receipt.as_ref().unwrap().id}
    pub fn actual(&self) -> &ActualCandidate {self.actual.as_ref().unwrap()}
    pub fn global_route(&self) -> &Path3D {self.route.as_ref().unwrap()}
    pub fn lifecycle(&self) -> Lifecycle {
        let f=self.follower.as_ref().unwrap();
        Lifecycle {epoch:f.owner.as_ref().unwrap().epoch(),submitted:f.submitted.map(|x|x.0),
            ticket_ids:f.source_tickets.iter().flatten().map(|x|x.0).collect(),
            retry_after:f.retry_after,calls:f.local_planning_calls,plans:f.local_plans,
            generation:f.route_generation,admission:f.admitted_generation,progress:f.progress}
    }
    pub fn slot(&self) -> Result<CompletedSlotObservation,String> {
        let f=self.follower.as_ref().ok_or("follower already cleaned up")?;
        let owner=f.owner.as_ref().ok_or("owner absent")?;
        let receipt=self.receipt.as_ref().ok_or("receipt absent")?;
        let actual=self.actual.as_ref().ok_or("mirror absent")?;
        owner.inspect_completion_slot_for_qualification(|completion,active,pending,closed,faulted,current_epoch| {
            let (full,receipt_matches,mirror_matches)=completion.map_or((false,false,false),|c| {
                let result=&c.value;
                let request=&result.request;
                let receipt_matches=c.id==receipt.id && c.epoch==actual.epoch
                    && Arc::ptr_eq(&request.source,&receipt.source)
                    && exact_pose_payload(&request.pose,&receipt.pose)
                    && request.detour==receipt.detour && request.generation==receipt.generation;
                match &result.outcome {
                    Ok(candidate)=>(!candidate.partial,receipt_matches,
                        Arc::ptr_eq(&request.source,&actual.source)
                        && request.generation==actual.generation
                        && request.progress.to_bits()==actual.progress.to_bits()
                        && request.end.to_bits()==actual.end.to_bits()
                        && request.scan.horizon_m.to_bits()==actual.horizon.to_bits()
                        && candidate.partial==actual.partial
                        && exact_path_payload(&candidate.path,&actual.path)),
                    Err(_)=>(false,receipt_matches,false),
                }
            });
            CompletedSlotObservation {current_epoch,
                completion_id:completion.map(|c|c.id),completion_epoch:completion.map(|c|c.epoch),
                active,pending,closed,faulted,full,receipt_matches,mirror_matches}
        }).ok_or_else(||"original exchange lock poisoned; slot observation unavailable".into())
    }
    pub fn proof_with_update(&self, update:&SurfaceMapUpdate) -> Result<SurfaceJobTicket,String> {
        let f=self.follower.as_ref().unwrap();
        let mut proof=f.source_tickets.iter().flatten().find(|(id,_)|*id==self.id())
            .map(|(_,ticket)|ticket.clone()).ok_or("original matching ticket absent")?;
        proof.observe(&update.snapshot,update.change_basis.as_deref(),&update.changes);
        Ok(proof)
    }
    pub fn retained_goal_and_route(&self) -> bool {
        let f=self.follower.as_ref().unwrap();
        f.goal.as_ref()==Some(&self.goal) && f.route.as_ref().zip(self.route.as_ref())
            .is_some_and(|(a,b)|Arc::ptr_eq(a,b))
    }
    pub fn publish(&mut self, update:SurfaceMapUpdate) -> Result<(),String> {
        self.follower.as_mut().unwrap().update_execution_map(ExecutionMap::SurfaceUpdate(update))
            .map_err(|e|e.to_string())
    }
    /// Same independent full-edge/current-connector/stopping oracle as stage39.
    /// It receives the exact checked mirror, never takes or mutates owner C.
    pub fn audit_on(&self, latest:Arc<SurfaceSnapshot>) -> Result<GeometryAudit,String> {
        let f=self.follower.as_ref().unwrap();let actual=self.actual();
        let current=f.pose.as_ref().ok_or("missing pose")?.clone();let from=body(&current);
        let old_edges_valid=actual.path.poses.windows(2).all(|e|
            actual.source.validate_body_motion(body(&e[0]),body(&e[1]),&f.boundary).is_ok());
        let first_new_edge_rejection=actual.path.poses.windows(2).enumerate().find_map(|(i,e)|
            latest.validate_body_motion(body(&e[0]),body(&e[1]),&f.boundary)
                .err().map(|error|(i,error.to_string())));
        let latest_body_valid=latest.validate_body_pose(from.0,from.1).is_ok();
        let mut oracle=SurfaceFollower::new(f.config.clone()).map_err(|e|e.to_string())?;
        oracle.update_execution_map(ExecutionMap::Surface(Arc::clone(&latest))).map_err(|e|e.to_string())?;
        oracle.configure_boundary(&f.boundary);oracle.update_odometry(current.clone());
        oracle.configure_goal(&self.goal,f.options).map_err(|e|e.to_string())?;
        oracle.observe_applied_command(Twist2::ZERO);
        let latest_connector_valid=oracle.replace_execution_path(
            ExecutionPath::Surface(actual.path.clone()),None,self.goal.clone()).is_ok();
        let current_command=if latest_connector_valid {
            match oracle.step(0.3) {PlannerStep::Command(command)=>Some(command),_=>None}
        } else {None};
        let limits=f.config.motion_limits.as_ref().ok_or("fixture lacks original motion limits")?;
        let pose=nuwax_nav_control::PlanarPose {x:current.pose.position.x,y:current.pose.position.y,
            yaw:current.pose.orientation.yaw()};
        let body_at=|p:nuwax_nav_control::PlanarPose| {
            let q=Quaternion::from_yaw(p.yaw);
            ([p.x,p.y,current.pose.position.z],[q.x,q.y,q.z,q.w])
        };
        let check=|| |a:nuwax_nav_control::PlanarPose,b:nuwax_nav_control::PlanarPose,_:f64| {
            latest.validate_body_motion(body_at(a),body_at(b),&f.boundary)
                .and_then(|_|latest.body_query_with_current_support(body_at(a).0,body_at(a).1)
                    .and_then(|view|view.validate_body_motion(body_at(a),body_at(b),&f.boundary))).is_ok()
        };
        let complete_stop_valid=current_command.is_some_and(|command|
            nuwax_nav_control::check_stopping_trajectory_with_period_check(pose,Twist2::ZERO,command,limits,
                1.0/f.config.controller.control_frequency,f64::from(latest.source_voxel_size_m())*0.5,
                latest.body_envelope().unwrap().radius(),check(),check()).is_some());
        Ok(GeometryAudit {old_edges_valid,latest_body_valid,
            latest_edges_valid:first_new_edge_rejection.is_none(),first_new_edge_rejection,
            latest_connector_valid,current_command,complete_stop_valid})
    }
    pub fn finish_with_original_step(mut self) -> Result<CompletedSlotFinished,String> {
        let id=self.id();
        let step=self.follower.as_mut().unwrap().step(0.3);
        let after_step=self.lifecycle();
        let old_ticket_absent=after_step.ticket_ids.iter().all(|got|*got!=id);
        let old_submitted_absent=after_step.submitted!=Some(id);
        let old_slot_absent=self.slot()?.completion_id!=Some(id);
        self.cleanup();
        Ok(CompletedSlotFinished {after_step,step,old_ticket_absent,
            old_submitted_absent,old_slot_absent,cleanup_returned:true})
    }
    fn cleanup(&mut self) {
        if let Some(mut follower)=self.follower.take() {
            follower.stop();
            drop(follower); // original owner Drop joins outside its exchange lock
            eprintln!("COMPLETED_SLOT_CLEANUP returned=true original_drop_join_result_not_exposed=true");
        }
    }
}
impl Drop for PublishedKernelCase {fn drop(&mut self) {self.cleanup();}}
