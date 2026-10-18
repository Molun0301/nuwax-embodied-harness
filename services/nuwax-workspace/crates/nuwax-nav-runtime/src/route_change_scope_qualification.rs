//! Explicit qualification support. No production policy change.
//! Intended as a child module of route_follower, enabled only by the existing
//! non-default surface-recovery-qualification feature. The stack test supplies
//! genuine Writer/cache snapshots; this driver invokes the real PlanKernel.
#![allow(missing_docs)]
use super::*;
use crate::route_planning::Candidate;
use nuwax_nav_types_messages::Quaternion;
use std::sync::mpsc;

pub struct ActualCandidate {
    pub source: Arc<SurfaceSnapshot>,
    pub path: Path3D,
    pub partial: bool,
    pub progress: f64,
    pub end: f64,
    pub horizon: f64,
    pub epoch: u64,
    pub generation: u64,
    pub kernel_elapsed_ms: f64,
}
#[derive(Clone, Debug)]
pub struct Lifecycle {
    pub epoch: u64,
    pub submitted: Option<u64>,
    pub ticket_ids: Vec<u64>,
    pub retry_after: Instant,
    pub calls: u64,
    pub plans: u64,
    pub generation: u64,
    pub admission: Option<u64>,
    pub progress: f64,
}
#[derive(Debug)]
pub struct GeometryAudit {
    pub old_edges_valid: bool,
    pub latest_body_valid: bool,
    pub latest_edges_valid: bool,
    pub first_new_edge_rejection: Option<(usize, String)>,
    pub latest_connector_valid: bool,
    pub current_command: Option<Twist2>,
    pub complete_stop_valid: bool,
}
#[derive(Debug)]
pub struct Finished {
    pub lifecycle_before_cleanup: Lifecycle,
    pub cancelled_before_owner_return: bool,
    pub completion_consumed: bool,
    pub command_after_admission: Option<PlannerStep>,
}

pub struct HeldKernelCase {
    follower: Option<RouteFollower>,
    release: Option<mpsc::Sender<()>>,
    returned: mpsc::Receiver<(bool, bool)>,
    actual: Option<ActualCandidate>,
    id: u64,
    route: Arc<Path3D>,
    goal: PoseStamped,
}
impl HeldKernelCase {
    pub fn capture(config: SurfaceFollowerConfig, source: Arc<SurfaceSnapshot>,
        pose: PoseStamped, route: Path3D, goal: PoseStamped) -> Result<Self, String> {
        let mut follower = RouteFollower::new(config.clone()).map_err(|e|e.to_string())?;
        follower.update_execution_map(ExecutionMap::Surface(source)).map_err(|e|e.to_string())?;
        follower.update_odometry(pose);
        follower.configure_goal(&goal, GoalOptions::default()).map_err(|e|e.to_string())?;
        follower.replace_execution_path(ExecutionPath::Surface(route), None, goal.clone())
            .map_err(|e|e.to_string())?;
        if let Some(reason) = follower.controller.current_body_error() {
            return Err(format!("fixture current body is invalid: {reason}"));
        }
        let retained_route = follower.route.as_ref().unwrap().clone();
        // Replace only an idle initial owner, before any request. The replacement
        // is the same original PlanningOwner driving a real, unmodified kernel.
        let (ready, receive) = mpsc::channel::<Result<ActualCandidate,String>>();
        let (release, released) = mpsc::channel();
        let (returned, returned_rx) = mpsc::channel();
        let mut kernel = PlanKernel::new(config);
        follower.owner = Some(PlanningOwner::new(move |request: PlanRequest, cancel| {
            let began = Instant::now();
            let result = kernel.compute(request, cancel);
            let observation = match &result.outcome {
                Ok(Candidate {partial, path, ..}) => Ok(ActualCandidate {
                    source: result.request.source.clone(), path: path.clone(), partial: *partial,
                    progress: result.request.progress, end: result.request.end,
                    horizon: result.request.scan.horizon_m, epoch: cancel.expected,
                    generation: result.request.generation,
                    kernel_elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
                }),
                Err(errors) => Err(format!("real PlanKernel prerequisite failed: {errors:?}")),
            };
            let _ = ready.send(observation);
            // The only artificial wait: actual compute has already returned.
            // Disconnection releases on test assertion unwind. Not performance data.
            let released_by_test = released.recv_timeout(Duration::from_secs(30)).is_ok();
            let _ = returned.send((cancel.cancelled(), released_by_test));
            result
        }).map_err(|e|e.to_string())?);
        let mut case = Self {follower:Some(follower), release:Some(release), returned:returned_rx,
            actual:None, id:0, route:retained_route, goal};
        case.follower.as_mut().unwrap().schedule(false);
        case.id = case.follower.as_ref().unwrap().submitted.ok_or("request was not submitted")?.0;
        case.actual = Some(receive.recv_timeout(Duration::from_secs(30))
            .map_err(|e|format!("real-kernel completion barrier not reached: {e}"))??);
        Ok(case)
    }
    pub fn actual(&self) -> &ActualCandidate { self.actual.as_ref().unwrap() }
    pub fn global_route(&self) -> &Path3D { &self.route }
    pub fn lifecycle(&self) -> Lifecycle {
        let f = self.follower.as_ref().unwrap();
        Lifecycle {epoch:f.owner.as_ref().unwrap().epoch(), submitted:f.submitted.map(|x|x.0),
            ticket_ids:f.source_tickets.iter().flatten().map(|x|x.0).collect(),
            retry_after:f.retry_after,calls:f.local_planning_calls,plans:f.local_plans,
            generation:f.route_generation,admission:f.admitted_generation,progress:f.progress}
    }
    pub fn retained_goal_and_route(&self) -> bool {
        let f = self.follower.as_ref().unwrap();
        f.goal.as_ref()==Some(&self.goal) && f.route.as_ref().is_some_and(|r|Arc::ptr_eq(r,&self.route))
    }
    pub fn publish(&mut self, update: SurfaceMapUpdate) -> Result<(), String> {
        self.follower.as_mut().unwrap().update_execution_map(ExecutionMap::SurfaceUpdate(update))
            .map_err(|e|e.to_string())
    }
    /// Independent test-only full-edge/connector/stopping audit. It neither
    /// mutates the held result nor grants admission to its original follower.
    pub fn audit_on(&self, latest: Arc<SurfaceSnapshot>) -> Result<GeometryAudit,String> {
        let f = self.follower.as_ref().unwrap();
        let actual = self.actual();
        let current = f.pose.as_ref().ok_or("missing current pose")?.clone();
        let from = body(&current);
        let old_edges_valid = actual.path.poses.windows(2).all(|e|
            actual.source.validate_body_motion(body(&e[0]),body(&e[1]),&f.boundary).is_ok());
        let first_new_edge_rejection = actual.path.poses.windows(2).enumerate().find_map(|(i,e)|
            latest.validate_body_motion(body(&e[0]),body(&e[1]),&f.boundary).err().map(|e|(i,e.to_string())));
        let latest_body_valid = latest.validate_body_pose(from.0,from.1).is_ok();
        let mut oracle = SurfaceFollower::new(f.config.clone()).map_err(|e|e.to_string())?;
        oracle.update_execution_map(ExecutionMap::Surface(latest.clone())).map_err(|e|e.to_string())?;
        oracle.configure_boundary(&f.boundary);
        oracle.update_odometry(current.clone());
        oracle.configure_goal(&self.goal,f.options).map_err(|e|e.to_string())?;
        oracle.observe_applied_command(Twist2::ZERO);
        let latest_connector_valid = oracle.replace_execution_path(
            ExecutionPath::Surface(actual.path.clone()),None,self.goal.clone()).is_ok();
        let current_command = if latest_connector_valid {
            match oracle.step(0.3) {PlannerStep::Command(c)=>Some(c),_=>None}
        } else {None};
        let limits = f.config.motion_limits.as_ref().ok_or("fixture lacks original motion limits")?;
        let pose = nuwax_nav_control::PlanarPose {x:current.pose.position.x,y:current.pose.position.y,
            yaw:current.pose.orientation.yaw()};
        let body_at = |p:nuwax_nav_control::PlanarPose| {
            let q=Quaternion::from_yaw(p.yaw);
            ([p.x,p.y,current.pose.position.z],[q.x,q.y,q.z,q.w])
        };
        let check = || |a:nuwax_nav_control::PlanarPose,b:nuwax_nav_control::PlanarPose,_:f64| {
            latest.validate_body_motion(body_at(a),body_at(b),&f.boundary)
                .and_then(|_|latest.body_query_with_current_support(body_at(a).0,body_at(a).1)
                    .and_then(|view|view.validate_body_motion(body_at(a),body_at(b),&f.boundary))).is_ok()
        };
        let complete_stop_valid=current_command.is_some_and(|command|
            nuwax_nav_control::check_stopping_trajectory_with_period_check(pose,Twist2::ZERO,command,limits,
                1.0/f.config.controller.control_frequency,f64::from(latest.source_voxel_size_m())*0.5,
                latest.body_envelope().unwrap().radius(),check(),check()).is_some());
        Ok(GeometryAudit {old_edges_valid,latest_body_valid,latest_edges_valid:first_new_edge_rejection.is_none(),
            first_new_edge_rejection,latest_connector_valid,current_command,complete_stop_valid})
    }
    pub fn finish(mut self) -> Result<Finished,String> {
        self.release.take().ok_or("barrier was already released")?.send(())
            .map_err(|e|e.to_string())?;
        let (cancelled, released_by_test) = self.returned.recv_timeout(Duration::from_secs(30)).map_err(|e|e.to_string())?;
        if !released_by_test { return Err("completion barrier expired/disconnected before explicit test release".into()); }
        let mut completion_consumed = false;
        if !cancelled {
            let until=Instant::now()+Duration::from_secs(5);
            loop {
                let f=self.follower.as_mut().unwrap();
                f.consume_plan();
                if f.source_tickets.iter().flatten().all(|(id,_)|*id!=self.id)
                    && f.submitted.is_none_or(|(id,_,_)|id!=self.id) {
                    completion_consumed=true;break;
                }
                if Instant::now()>=until {return Err("real owner result was not consumed".into());}
                std::thread::sleep(Duration::from_millis(1));
            }
        } else {
            // Original owner cannot publish once this token is cancelled. Do not
            // manually take/reinsert that result to make an apparent admission.
            self.follower.as_mut().unwrap().consume_plan();
        }
        let command_after_admission = {
            let f=self.follower.as_mut().unwrap();
            if f.admitted_generation==Some(f.route_generation) {Some(f.step(0.3))} else {None}
        };
        let report=Finished {lifecycle_before_cleanup:self.lifecycle(),cancelled_before_owner_return:cancelled,
            completion_consumed,command_after_admission};
        self.cleanup();
        Ok(report)
    }
    fn cleanup(&mut self) {
        if let Some(mut follower)=self.follower.take() {
            follower.stop();
            if let Some(release)=self.release.take() {let _=release.send(());let _=release.send(());}
            drop(follower); // original Drop joins outside the lock; it does not expose join status
        }
    }
}
impl Drop for HeldKernelCase {fn drop(&mut self) {self.cleanup();}}
