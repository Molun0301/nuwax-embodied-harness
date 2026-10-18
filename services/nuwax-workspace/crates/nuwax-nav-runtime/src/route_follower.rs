//! 3DMR式持续路线执行：全局参考留存，Scan只更新前方轨迹，原控制器继续唯一生成速度。
use crate::{
    ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RuntimeError, SurfaceFollower,
    SurfaceFollowerConfig, SurfaceSnapshot,
};
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_pct::PctPlanner;
use nuwax_nav_scan::{LocalPlanError, ScanConfig};
use nuwax_nav_types_messages::{
    GoalOptions, Path2D, Path3D, Point3, PoseStamped, Twist2,
};
use std::sync::Arc;
use crate::surface_changes::{SurfaceGeometryChanges, SurfaceJobTicket, SurfaceMapUpdate};

use crate::planning_owner::PlanningOwner;
use crate::route_planning::{body, pose_bits, exact_points, PlanKernel, PlanRequest, PlanResult, ReferenceSource};
use std::time::{Duration,Instant};

#[cfg(any(test, feature = "surface-recovery-qualification"))]
thread_local! { static PLANNING_REQUEST_BUILDS_FOR_QUALIFICATION: std::cell::Cell<u64> = const { std::cell::Cell::new(0) }; }


pub struct RouteFollower {
    controller: SurfaceFollower,
    config: SurfaceFollowerConfig,
    scan: ScanConfig,
    boundary: Arc<Vec<[f64; 2]>>,
    planner: Arc<PctPlanner>,
    map: Option<Arc<SurfaceSnapshot>>,
    pose: Option<PoseStamped>,
    route: Option<Arc<Path3D>>,
    goal: Option<PoseStamped>,
    options: GoalOptions,
    cumulative: Arc<Vec<f64>>,
    progress: f64,
    local_end: f64,
    local_progress_at_plan: f64,
    local_path: Option<Path3D>,
    local_start: Option<Point3>,
    dirty: bool,
    local_plans: u64,
    local_planning_calls: u64,
    owner: Option<PlanningOwner<PlanRequest,PlanResult>>,
    next_request: u64,
    submitted: Option<(u64,Point3,bool)>,
    // Exactly the owner's active/completed and pending slots; no second queue.
    source_tickets: [Option<(u64,SurfaceJobTicket)>;2],
    retry_after: Instant,
    preparation_fault: Option<String>,
    permanent_fault: Option<String>,
    qualified_failure: Option<String>,
    wait_reason: Option<String>,
    route_generation: u64,
    admitted_generation: Option<u64>,
    local_failures: Vec<LocalPlanError>,
}
fn error(message: impl Into<String>) -> RuntimeError {
    RuntimeError::InvalidInput {
        field: "route",
        reason: message.into(),
    }
}
impl RouteFollower {
    pub fn new(config: SurfaceFollowerConfig) -> Result<Self, RuntimeError> {
        let kernel=PlanKernel::new(config.clone());
        Self::with_kernel(config,kernel)
    }
    #[cfg(test)]
    pub(crate) fn new_with_planning_hook(config:SurfaceFollowerConfig, hook:Box<dyn FnMut(&PlanRequest)+Send>)->Result<Self,RuntimeError> {
        let mut kernel=PlanKernel::new(config.clone());kernel.hook=Some(hook);
        Self::with_kernel(config,kernel)
    }
    #[cfg(test)]
    pub(crate) fn planning_lifecycle_for_test(&self)->(Option<u64>,u64,Option<u64>,u64,u64) {
        (self.owner.as_ref().map(PlanningOwner::epoch),self.route_generation,self.submitted.map(|r|r.0),self.local_planning_calls,self.local_plans)
    }
    fn with_kernel(config:SurfaceFollowerConfig,mut kernel:PlanKernel)->Result<Self,RuntimeError> {
        let scan = config.scan.clone();
        scan.validate().map_err(error)?;
        let owner = if config.local_planning_enabled {
            Some(PlanningOwner::new(move |request,cancel|kernel.compute(request,cancel)).map_err(|e|error(e.to_string()))?)
        } else { None };
        Ok(Self {
            controller: SurfaceFollower::new(config.clone())?,
            planner: Arc::clone(&config.pct),
            config,
            scan,
            boundary: Arc::new(Vec::new()),
            map: None,
            pose: None,
            route: None,
            goal: None,
            options: GoalOptions::default(),
            cumulative: Arc::new(Vec::new()),
            progress: 0.0,
            local_end: 0.0,
            local_progress_at_plan: 0.0,
            local_path: None,
            local_start: None,
            dirty: true,
            local_plans: 0,
            local_planning_calls: 0,
            owner,
            next_request: 0,
            submitted: None,
            source_tickets: [None,None],
            retry_after: Instant::now(),
            preparation_fault: None,
            permanent_fault: None,
            qualified_failure: None,
            wait_reason: None,
            route_generation: 0,
            admitted_generation: None,
            local_failures: Vec::new(),
        })
    }
    /// 已执行的全局路线弧长，不受近场轨迹替换重置。
    pub fn progress_m(&self) -> f64 {
        self.progress
    }
    pub fn local_plans(&self) -> u64 {
        self.local_plans
    }
    /// Number of entries into local planning, including failed attempts.
    pub fn local_planning_calls(&self) -> u64 { self.local_planning_calls }
    #[cfg(any(test, feature = "surface-recovery-qualification"))]
    pub(crate) fn pending_source_work_for_qualification(&self) -> (u64, u64) {
        (crate::planning_owner::pending_replace_lock_attempts_for_qualification(),
            PLANNING_REQUEST_BUILDS_FOR_QUALIFICATION.get())
    }
    pub fn local_path(&self) -> Option<&Path3D> {
        self.local_path.as_ref()
    }
    /// Typed diagnostics from the latest unsuccessful local planning attempts.
    pub fn local_failures(&self) -> &[LocalPlanError] { &self.local_failures }
    fn revoke_local_admission(&mut self) {
        self.cancel_planning();
        self.preparation_fault=None;
        if let Some(next) = self.route_generation.checked_add(1) { self.route_generation=next; }
        else { self.permanent_fault=Some("route generation exhausted".into()); }
        self.admitted_generation = None;
        self.local_path = None;
        self.local_start = None;
        self.local_failures.clear();
        self.dirty = true;
    }
    fn advance_progress(&mut self) {
        let (Some(route), Some(pose)) = (&self.route, &self.pose) else {
            return;
        };
        let mut best = f64::INFINITY;
        let mut progress = self.progress;
        let window =
            (self.progress + self.scan.horizon_m).min(*self.cumulative.last().unwrap_or(&0.0));
        for (i, pair) in route.poses.windows(2).enumerate().skip(self.cumulative.partition_point(|&v|v<self.progress).saturating_sub(1)) {
            if self.cumulative[i + 1] + 1e-6 < self.progress {
                continue;
            }
            if self.cumulative[i] > window {
                break;
            }
            let a = pair[0].pose.position;
            let b = pair[1].pose.position;
            let v = [b.x - a.x, b.y - a.y, b.z - a.z];
            let l2 = v.iter().map(|x| x * x).sum::<f64>();
            if l2 < 1e-12 {
                continue;
            }
            let d = pose.pose.position;
            let t = (((d.x - a.x) * v[0] + (d.y - a.y) * v[1] + (d.z - a.z) * v[2]) / l2)
                .clamp(0.0, 1.0);
            let projected = Point3::new(a.x + v[0] * t, a.y + v[1] * t, a.z + v[2] * t);
            let distance = projected.distance(d);
            if distance < best {
                best = distance;
                progress = self.cumulative[i] + t * l2.sqrt();
            }
        }
        if best < self.config.max_path_deviation_m {
            self.progress = self.progress.max(progress);
        }
    }
    #[cfg(test)]
    fn reference(&self, horizon: f64) -> Result<(Vec<[f64; 3]>, f64, bool, bool), String> {
        let route = self.route.as_ref().ok_or("缺少全局路线")?;
        let current = self.pose.as_ref().ok_or("缺少实际位姿")?;
        let offset = self.config.body_to_support_z_m;
        let mut result = vec![[
            current.pose.position.x,
            current.pose.position.y,
            current.pose.position.z - offset,
        ]];
        let end = (self.progress + horizon).min(*self.cumulative.last().ok_or("全局路线为空")?);
        let mut direction:Option<[f64;2]>=None;
        let mut global_straight=true;
        for (i, pair) in route.poses.windows(2).enumerate().skip(self.cumulative.partition_point(|&v|v<self.progress).saturating_sub(1)) {
            if self.cumulative[i + 1] <= self.progress + 0.02 {
                continue;
            }
            let p = pair[1].pose.position;
            // 路段形状由原全局路线决定，实际机体的横向偏差不能把直线变成弯道。
            let delta=[p.x-pair[0].pose.position.x,p.y-pair[0].pose.position.y];
            let length=delta[0].hypot(delta[1]);
            if length>1e-9 {
                let next=delta.map(|v|v/length);
                if let Some(first)=direction {
                    global_straight &= first[0]*next[0]+first[1]*next[1]>0.0
                        && (first[0]*next[1]-first[1]*next[0]).abs()<1e-9;
                }else{direction=Some(next);}
            }
            if self.cumulative[i + 1] <= end {
                result.push([p.x, p.y, p.z - offset]);
            } else {
                let a = pair[0].pose.position;
                let length = self.cumulative[i + 1] - self.cumulative[i];
                let t = ((end - self.cumulative[i]) / length.max(1e-9)).clamp(0.0, 1.0);
                result.push([
                    a.x + (p.x - a.x) * t,
                    a.y + (p.y - a.y) * t,
                    a.z + (p.z - a.z) * t - offset,
                ]);
                break;
            }
        }
        let at_end = end >= *self.cumulative.last().unwrap() - 1e-6;
        if at_end {
            if let Some(goal) = &self.goal {
                if current.pose.position.distance(goal.pose.position)
                    >= self.options.position_tolerance_m
                {
                    // 层析末点是名义地面参考，实际站姿仍须进入用户三维到达球。
                    // 保留指向原目标的末段，允许投影在真正入球时结束。
                    let target = [
                        goal.pose.position.x,
                        goal.pose.position.y,
                        goal.pose.position.z - offset,
                    ];
                    if result.last() != Some(&target) {
                        result.push(target);
                    }
                }
            }
        }
        Ok((result, end, at_end, global_straight))
    }
    fn cancel_planning(&mut self) {
        if let Some(owner)=&self.owner {owner.cancel();}
        self.submitted=None;
        self.source_tickets=[None,None];
        self.qualified_failure=None;
        self.retry_after=Instant::now();
    }
    fn planning_request(
        &self, map: &Arc<SurfaceSnapshot>, pose: &PoseStamped,
        route: &Arc<Path3D>, detour: bool,
    ) -> PlanRequest {
        #[cfg(any(test, feature = "surface-recovery-qualification"))]
        PLANNING_REQUEST_BUILDS_FOR_QUALIFICATION.with(|count| count.set(count.get() + 1));
        let reference_source=ReferenceSource{route:Arc::clone(route),cumulative:Arc::clone(&self.cumulative),
            goal:self.goal.clone(),options:self.options,offset:self.config.body_to_support_z_m,progress:self.progress};
        PlanRequest{source:Arc::clone(map),pose:pose.clone(),boundary:Arc::clone(&self.boundary),reference:Vec::new(),
            reference_source:Some(reference_source),final_yaw:None,arrival:None,end:0.0,progress:self.progress,
            generation:self.route_generation,straight:false,detour,scan:self.scan.clone()}
    }
    // One receipt path for normal submission and pending-only replacement.
    // The caller alone decides whether a new retry deadline is appropriate.
    fn record_submission(&mut self,id:u64,start:Point3,detour:bool,retired:Option<u64>)->bool {
        self.next_request=id;self.submitted=Some((id,start,detour));
        for slot in &mut self.source_tickets {
            if slot.as_ref().is_some_and(|(old,_)|Some(*old)==retired) {*slot=None;}
        }
        if let Some(map)=self.map.as_ref().filter(|map|map.body_envelope().is_some()) {
            let Some(slot)=self.source_tickets.iter_mut().find(|slot|slot.is_none()) else {
                self.cancel_planning();self.permanent_fault=Some("planning proof slots exhausted".into());return false;
            };
            *slot=Some((id,SurfaceJobTicket::new(Some(Arc::clone(map)))));
        }
        self.local_planning_calls+=1;
        true
    }
    fn coalesce_pending_source(&mut self) {
        if self.preparation_fault.is_some() {return;}
        let Some((pending_id,_,previous_detour))=self.submitted else {return;};
        // Before take(), two proofs describe active/completed A and queued P.
        // A single changed active request must not add a lock/factory per tick.
        let [Some((first_id,first)),Some((second_id,second))]=&self.source_tickets else {return;};
        let proof=if *first_id==pending_id {first} else if *second_id==pending_id {second} else {return;};
        if proof.unknown || proof.changes.is_empty() {return;}
        let (Some(owner),Some(map),Some(pose),Some(route))=
            (&self.owner,&self.map,&self.pose,&self.route) else {return;};
        if self.config.kinematics().is_none() {self.preparation_fault=Some("近场执行缺少运动能力".into());return;}
        let Some(id)=self.next_request.checked_add(1) else {self.permanent_fault=Some("planning request IDs exhausted".into());return;};
        // Refresh source/reference inputs only; original scheduling owns mode changes.
        let detour=previous_detour;
        let start=pose.pose.position;
        // Borrow only; no request or Arc clone until the original lock matches P.
        let retired=owner.replace_pending_with_retired(pending_id,owner.epoch(),id,
            ||self.planning_request(map,pose,route,detour));
        if let Some(retired)=retired {
            self.record_submission(id,start,detour,Some(retired));
            // This updates the already queued slot, not the normal submit cadence.
            // Keep its original retry_after and request mode.
        }
    }
    fn schedule(&mut self,detour:bool) {
        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::PlanSchedule);
        if self.owner.is_none() || self.preparation_fault.is_some() || Instant::now()<self.retry_after {return;}
        let (Some(map),Some(pose))=(&self.map,&self.pose) else {return;};
        if let Some((_,start,previous_detour))=self.submitted {
            // Soft metadata alone cannot cancel or restart useful active work.
            if pose.pose.position.distance(start)<self.scan.refresh_distance_m.max(0.25) && (!detour || previous_detour) {return;}
        }
        if self.config.kinematics().is_none() {self.preparation_fault=Some("近场执行缺少运动能力".into());return;}
        let Some(route)=&self.route else {return;};
        let Some(id)=self.next_request.checked_add(1) else {self.permanent_fault=Some("planning request IDs exhausted".into());return;};
        let request=self.planning_request(map,pose,route,detour);
        let start=pose.pose.position;
        if let Some(retired)=self.owner.as_ref().unwrap().submit_with_retired(id,request) {
            if self.record_submission(id,start,detour,retired) {
                self.retry_after=Instant::now()+Duration::from_millis(250);
            }
        }
    }
    fn current_rejection(&mut self, reason: String) -> PlannerStep {
        self.dirty = true;
        self.schedule(true);
        // A rejected current command still means confirmed ZERO. With a local
        // owner, keep consuming current evidence while its replacement is being
        // computed (including the existing retry cooldown). Asking the scheduler
        // for a global path here would stop this owner and cancel that same job.
        let local_recovery = self.config.local_planning_enabled
            && self.owner.as_ref().is_some_and(|owner| !owner.faulted())
            && self.route.is_some() && self.goal.is_some() && self.map.is_some()
            && self.pose.as_ref().is_some_and(|pose| nuwax_nav_pct::inside_boundary(
                [pose.pose.position.x, pose.pose.position.y], &self.boundary))
            && self.preparation_fault.is_none() && self.permanent_fault.is_none()
            && self.qualified_failure.is_none();
        if local_recovery {
            // Retire only execution permission for the rejected local path. Keep
            // its owner/job and global route; consume_plan must re-admit before
            // any old-path Command (including ZERO) can reset the wait budget.
            self.admitted_generation = None;
            self.wait_reason = Some(format!("当前轨迹受阻，保持零速等待局部接续：{reason}"));
            PlannerStep::Pending(self.local_failures.clone())
        } else {
            PlannerStep::ReplanRequired(reason)
        }
    }
    fn consume_plan(&mut self) {
        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::PlanConsumeAdmission);
        let Some(owner)=&self.owner else {return;};
        let Some(completion)=owner.take() else {return;};
        if owner.faulted() {return;}
        if self.submitted.is_some_and(|(id,_,_)|id==completion.id) {self.submitted=None;}
        let proof = self.source_tickets.iter_mut().find_map(|slot| {
            if slot.as_ref().is_some_and(|(id,_)|*id==completion.id) {slot.take().map(|(_,proof)|proof)} else {None}
        });
        let result=completion.value;
        let request=result.request;
        if completion.epoch!=owner.epoch() || request.generation!=self.route_generation || self.route.is_none()
            || !exact_points(&request.boundary,&self.boundary)
            || self.map.as_ref().is_none_or(|map| match (&proof,&result.outcome) {
                (Some(proof),Ok(candidate)) => !proof.permits_path(map,&candidate.path),
                _ => !nuwax_nav_pct::same_source_geometry(map,&request.source),
            }) {self.dirty=true;return;}
        match result.outcome {
            Ok(candidate)=> {
                if request.end+1e-6<self.progress {self.dirty=true;return;}
                let covered_end=if candidate.partial {
                    if self.config.motion_limits.is_none() {self.dirty=true;return;}
                    let Some(endpoint)=candidate.path.poses.last().map(|p|p.pose.position) else {self.dirty=true;return;};
                    let Some(route)=self.route.as_ref() else {return;};
                    let mut nearest=f64::INFINITY;let mut covered=request.progress;
                    for (i,pair) in route.poses.windows(2).enumerate().skip(
                        self.cumulative.partition_point(|&value|value<request.progress).saturating_sub(1)) {
                        if self.cumulative[i]>request.end {break;}
                        let length=self.cumulative[i+1]-self.cumulative[i];
                        if length<=1e-12 || self.cumulative[i+1]<request.progress || self.cumulative[i]>request.end {continue;}
                        let a=pair[0].pose.position;let b=pair[1].pose.position;
                        let v=[b.x-a.x,b.y-a.y,b.z-a.z];let l2=v.iter().map(|x|x*x).sum::<f64>();
                        let lo=((request.progress-self.cumulative[i])/length).clamp(0.0,1.0);
                        let hi=((request.end-self.cumulative[i])/length).clamp(0.0,1.0);
                        let t=(((endpoint.x-a.x)*v[0]+(endpoint.y-a.y)*v[1]+(endpoint.z-a.z)*v[2])/l2).clamp(lo,hi);
                        let projected=Point3::new(a.x+v[0]*t,a.y+v[1]*t,a.z+v[2]*t);
                        let distance=projected.distance(endpoint);
                        if distance<nearest {nearest=distance;covered=self.cumulative[i]+length*t;}
                    }
                    let minimum=request.source.body_envelope().map_or(f64::INFINITY,|m|m.radius());
                    if nearest>self.config.max_path_deviation_m || covered-request.progress<minimum
                        || covered+1e-6<self.progress {self.dirty=true;return;}
                    covered
                }else{request.end};
                let Some(goal)=self.goal.clone() else {return;};
                // Candidate geometry is advice only. Current bounded connector admission
                // uses the actual latest pose/evidence; current step still owns motion.
                match self.controller.replace_execution_path(ExecutionPath::Surface(candidate.path.clone()),None,goal) {
                    Ok(())=> {
                        self.local_path=Some(candidate.path);
                        self.admitted_generation=Some(self.route_generation);
                        self.local_failures.clear();self.qualified_failure=None;self.local_progress_at_plan=request.progress;
                        self.local_end=covered_end;self.local_start=Some(request.pose.pose.position);
                        self.dirty=false;self.local_plans+=1;
                        tracing::info!(target:"navigation.scan",event="navigation.scan.candidate_admitted",
                            elapsed_ms=candidate.elapsed_ms,request_id=completion.id,partial=candidate.partial,covered_end,requested_end=request.end,"geometry candidate admitted; current guard remains authoritative");
                    }
                    Err(e)=> {
                        // Moved-start failure neither teleports nor destroys current admission.
                        let fresh=self.map.as_ref().is_some_and(|map|map.same_body_query_view(&request.source))
                            && self.pose.as_ref().is_some_and(|pose|pose.frame_id==request.pose.frame_id && pose_bits(body(pose))==pose_bits(body(&request.pose)));
                        if fresh {self.qualified_failure=Some(e.to_string());}
                        self.local_failures=vec![e.to_string().into()];self.dirty=true;
                    }
                }
            }
            Err(failures)=> {
                // Even an exact search negative is not a current obstacle measurement.
                // Keep current guarded execution and retry latest inputs at bounded cadence.
                let fresh=self.map.as_ref().is_some_and(|map|map.same_body_query_view(&request.source))
                    && self.pose.as_ref().is_some_and(|pose|pose.frame_id==request.pose.frame_id && pose_bits(body(pose))==pose_bits(body(&request.pose)));
                self.qualified_failure=if fresh { failures.iter().find(|f|f.kind==nuwax_nav_scan::LocalPlanFailureKind::Unclassified).map(ToString::to_string) } else {None};
                self.local_failures=failures;self.dirty=true;
            }
        }
    }

}
impl NavPlanner for RouteFollower {
    fn revalidate_selected_command(&mut self,command:Twist2)->Option<PlannerStep> {
        let result=self.controller.revalidate_selected_command(command);
        result.map(|step| match step {
            PlannerStep::ReplanRequired(reason) => self.current_rejection(reason),
            step => step,
        })
    }
    fn revalidate_selected_command_at_current_state(&mut self,command:Twist2)->Option<PlannerStep> {
        let result=self.controller.revalidate_selected_command_at_current_state(command);
        result.map(|step| match step {
            PlannerStep::ReplanRequired(reason) => self.current_rejection(reason),
            step => step,
        })
    }
    fn take_command_dispatch_bound(&mut self)->Option<crate::CommandDispatchBound> {self.controller.take_command_dispatch_bound()}
    fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
        self.preparation_fault=None;
        if !exact_points(&self.boundary,boundary) {self.cancel_planning();self.dirty=true;}
        self.boundary = Arc::new(boundary.to_vec());
        self.controller.configure_boundary(boundary);
    }
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: GoalOptions,
    ) -> Result<(), RuntimeError> {
        if self.config.local_planning_enabled { self.planner.clear_local_costs(); }
        self.controller.configure_goal(goal, options)?;
        self.revoke_local_admission();
        self.route = None;
        self.options = options;
        self.goal = Some(goal.clone());
        Ok(())
    }
    fn replace_path(&mut self, path: Path2D) {
        self.stop();
        self.controller.replace_path(path);
    }
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        let ExecutionPath::Surface(route) = path else {
            return Err(error("三维路线不能使用二维路径"));
        };
        if yaws.is_some() || route.poses.is_empty() {
            return Err(error("三维路线为空或带重复朝向源"));
        }
        for pose in &route.poses {
            crate::validate_map_pose(pose, "route.payload")?;
        }
        crate::validate_map_pose(&goal, "goal_pose")?;
        let direct = !self.config.local_planning_enabled
            || self.map.as_ref().is_some_and(|map| map.body_envelope().is_none());
        if direct {
            self.controller.replace_execution_path(
                ExecutionPath::Surface(route.clone()), None, goal.clone(),
            )?;
        }

        self.revoke_local_admission();
        if direct { self.admitted_generation = Some(self.route_generation); }
        let mut cumulative = vec![0.0];
        for pair in route.poses.windows(2) {
            cumulative.push(cumulative.last().unwrap()+pair[0].pose.position.distance(pair[1].pose.position));
        }
        self.cumulative=Arc::new(cumulative);
        self.route = Some(Arc::new(route));
        self.goal = Some(goal);
        self.progress = 0.0;
        self.local_end = 0.0;
        self.dirty = true;
        self.advance_progress();
        // Path acceptance never waits for the independent planning CPU owner.
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        if self.pose.as_ref().is_none_or(|old|old.frame_id!=pose.frame_id || pose_bits(body(old))!=pose_bits(body(&pose))) {self.qualified_failure=None;}
        self.controller.update_odometry(pose.clone());
        self.pose = Some(pose);
    }
    fn update_costmap(&mut self, map: CostMap2D) {
        self.stop();
        self.map = None;
        self.controller.update_costmap(map);
    }
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        let update = match map {
            ExecutionMap::SurfaceUpdate(update) => update,
            ExecutionMap::Surface(surface) => {
                let same=self.map.as_ref().is_some_and(|old|nuwax_nav_pct::same_source_geometry(old,&surface));
                SurfaceMapUpdate {planar_reference:None,snapshot:surface,change_basis:self.map.clone(),
                    changes:Arc::new(if same {SurfaceGeometryChanges::Empty} else {SurfaceGeometryChanges::Unknown})}
            }
            ExecutionMap::Planar(_) => return Err(error("三维路线执行缺少三维地图")),
        };
        if update.planar_reference.as_ref().is_some_and(|prepared|!prepared.matches_surface(&update.snapshot)) {
            return Err(error("后台平面查询地图与当前几何或配置不匹配"));
        }
        let surface=Arc::clone(&update.snapshot);
        if self.map.as_ref().is_none_or(|old|!old.same_body_query_view(&surface)) {self.qualified_failure=None;}
        for (_,ticket) in self.source_tickets.iter_mut().flatten() {
            ticket.observe(&surface,update.change_basis.as_deref(),&update.changes);
        }
        if let Some(old)=&self.map {
            let mut change=SurfaceJobTicket::new(Some(Arc::clone(old)));
            change.observe(&surface,update.change_basis.as_deref(),&update.changes);
            // Unknown cannot be softened by a newer Empty batch or an equal revision.
            // Include both the retained reference and the actual detour. Progress
            // and current guarded admission remain intact even for related changes.
            let affected=change.unknown || self.source_tickets.iter().flatten().any(|(_,ticket)|ticket.unknown)
                || (!matches!(update.changes.as_ref(),SurfaceGeometryChanges::Empty)
                    && (self.route.as_ref().is_some_and(|path|!change.permits_path(&surface,path))
                        || self.local_path.as_ref().is_some_and(|path|!change.permits_path(&surface,path))));
            if affected {self.cancel_planning();self.dirty=true;}
        }
        // Every publication reaches the current guard even when planning is unaffected.
        if self.local_path.is_none() {
            self.dirty = true;
        }
        self.controller
            .update_execution_map(ExecutionMap::SurfaceUpdate(update))?;
        self.map = Some(surface);
        Ok(())
    }
    fn invalidate_execution_map(&mut self) {
        self.stop();
        self.map = None;
        self.controller.invalidate_execution_map();
    }
    fn observe_applied_command(&mut self, command: Twist2) {
        self.controller.observe_applied_command(command);
    }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        if self.wait_reason.is_some() { return Vec::new(); }
        self.local_path.as_ref().map(|path| path.poses.iter().map(|p| {
            let p = p.pose.position; [p.x, p.y, p.z]
        }).collect()).unwrap_or_default()
    }
    fn step(&mut self, width: f64) -> PlannerStep {
        self.wait_reason = None;
        if let Some(reason)=&self.permanent_fault {return PlannerStep::Failed(reason.clone());}
        if self.owner.as_ref().is_some_and(|owner|owner.faulted()) {
            self.local_failures=vec!["local planning owner terminated unexpectedly".into()];
            return PlannerStep::Failed(self.local_failures[0].to_string());
        }
        if self.route.is_none() { return PlannerStep::Pending(Vec::new()); }
        if self
            .map
            .as_ref()
            .is_some_and(|map| map.body_envelope().is_none())
        {
            return if self.admitted_generation == Some(self.route_generation) {
                #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ControllerStep);
                self.controller.step(width)
            } else { PlannerStep::Pending(Vec::new()) };
        }
        let body_error={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::RouteCurrentBody);
            self.controller.current_body_error()
        };
        if let Some(reason) = body_error {
            // 当前实际机体碰撞或已知地形不相容时，重发同一全局路线只会造成规划忙循环。
            // 原实际速度回执、输入期限和取消仍由执行调度器处理。
            self.wait_reason = Some(format!(
                "当前机体几何检查未通过，保持零速等待地图或位姿恢复：{reason}"
            ));
            return PlannerStep::Pending(Vec::new());
        }
        {
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::RouteProgress);
            self.advance_progress();
        }
        if !self.config.local_planning_enabled {
            return {
                #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ControllerStep);
                self.controller.step(width)
            };
        }
        // Refresh a real queued P before take() can wake it behind completed A.
        self.coalesce_pending_source();
        self.consume_plan();
        if self.owner.as_ref().is_some_and(|owner|owner.faulted()) {
            return PlannerStep::Failed("local planning owner terminated unexpectedly".into());
        }
        let window=(self.local_end-self.local_progress_at_plan).max(0.1);
        let moved=self.pose.as_ref().zip(self.local_start).is_some_and(|(pose,start)|
            pose.pose.position.distance(start)>=self.scan.refresh_distance_m.min(window*0.6));
        let at_end=self.cumulative.last().is_some_and(|&end|self.local_end>=end-1e-6);
        let near_end=!at_end && self.local_end-self.progress<self.scan.refresh_distance_m.min(window*0.35);
        if self.dirty || moved && !at_end || near_end {self.schedule(self.local_path.is_some() && self.admitted_generation.is_none());}
        if let Some(reason)=&self.permanent_fault {return PlannerStep::Failed(reason.clone());}
        if let Some(reason)=&self.preparation_fault {
            self.local_failures=vec![reason.clone().into()];
            return PlannerStep::ReplanRequired(reason.clone());
        }
        if self.admitted_generation!=Some(self.route_generation) {
            if let Some(reason)=&self.qualified_failure {return PlannerStep::ReplanRequired(reason.clone());}
            self.wait_reason=Some("近场搜索尚未取得可执行轨迹，保持零速等待".into());
            return PlannerStep::Pending(self.local_failures.clone());
        }
        let step={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ControllerStep);
            self.controller.step(width)
        };
        if let PlannerStep::ReplanRequired(reason) = step {return self.current_rejection(reason);}
        if matches!(step,PlannerStep::Failed(_)) {return step;}
        if let Some(reason)=&self.qualified_failure {return PlannerStep::ReplanRequired(reason.clone());}
        step
    }

    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }
    fn stop(&mut self) {
        self.wait_reason = None;
        self.revoke_local_admission();
        self.controller.stop();
        self.route = None;
        self.goal = None;
        self.cumulative=Arc::new(Vec::new());
        self.progress = 0.0;
        self.local_end = 0.0;
        self.local_progress_at_plan = 0.0;
        self.local_path = None;
        self.local_start = None;
        self.dirty = true;
    }
}

#[cfg(feature = "surface-recovery-qualification")]
#[path = "route_change_scope_qualification.rs"]
pub mod route_change_scope_qualification;

#[cfg(feature = "surface-recovery-qualification")]
#[path = "route_pending_source_qualification.rs"]
pub mod route_pending_source_qualification;

#[cfg(feature = "surface-recovery-qualification")]
#[path = "route_r11_qualification.rs"]
pub mod route_r11_qualification;

#[cfg(feature = "surface-recovery-qualification")]
#[path = "route_pending_only_nohold_qualification.rs"]
pub mod route_pending_only_nohold_qualification;

#[cfg(feature = "surface-recovery-qualification")]
#[path = "route_completed_slot_qualification.rs"]
pub mod route_completed_slot_qualification;

#[cfg(test)]
mod tests {
    use super::*;
    include!("route_selective_refresh_tests.rs");
    include!("route_pending_only_v2_tests.rs");
    include!("route_progress_prefix_tests.rs");
    include!("route_partial_priority_tests.rs");

#[derive(Clone, Copy)]
enum RecoveryFixtureResult { Incomplete, Full, Partial }
struct RecoveryLifecycleProbe {
    entered: mpsc::Receiver<(bool, u64)>,
    completed: mpsc::Receiver<(bool, bool)>,
    release: Option<mpsc::Sender<()>>,
}
impl Drop for RecoveryLifecycleProbe {
    fn drop(&mut self) {
        if let Some(tx) = self.release.take() {
            // At most active + latest pending. Also release on assertion unwind.
            let _ = tx.send(()); let _ = tx.send(());
        }
    }
}
fn install_recovery_lifecycle_probe(
    follower: &mut RouteFollower, first_result: RecoveryFixtureResult,
) -> RecoveryLifecycleProbe {
    let (entered, receive) = mpsc::channel();
    let (completed, completions) = mpsc::channel();
    let (release, wait) = mpsc::channel();
    follower.cancel_planning();
    let offset = follower.config.body_to_support_z_m;
    let mut index = 0;
    follower.owner = Some(PlanningOwner::new(move |mut request: PlanRequest, cancel| {
        let detour = request.detour;
        let _ = entered.send((detour, cancel.expected));
        let _ = wait.recv_timeout(Duration::from_secs(5));
        let cancelled = cancel.cancelled();
        let _ = completed.send((detour, cancelled));
        let mode = if index == 0 { first_result } else { RecoveryFixtureResult::Incomplete };
        index += 1;
        let outcome = if cancelled || matches!(mode, RecoveryFixtureResult::Incomplete) {
            Err(typed_failures())
        } else {
            request.materialize_reference().unwrap();
            let partial = matches!(mode, RecoveryFixtureResult::Partial);
            let path = if partial {
                Path3D { poses: vec![comparison_pose(0.5, 0), comparison_pose(1.2, 0)] }
            } else {
                Path3D { poses: request.reference.iter().map(|p| {
                    let mut pose = request.pose.clone();
                    pose.pose.position = Point3::new(p[0], p[1], p[2] + offset);
                    pose
                }).collect() }
            };
            // A synthetic status fixture may not smuggle invalid geometry into
            // the current connector. All edges use the captured source too.
            for edge in path.poses.windows(2) {
                request.source.validate_body_motion(body(&edge[0]), body(&edge[1]),
                    &request.boundary).unwrap();
            }
            Ok(Candidate { partial, path, elapsed_ms: 0.0 })
        };
        PlanResult { request, outcome }
    }).unwrap());
    RecoveryLifecycleProbe { entered: receive, completed: completions, release: Some(release) }
}
fn recovery_publish_real_wall(follower: &mut RouteFollower) {
    let basis = follower.map.clone().unwrap();
    let wall = Arc::new(basis.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
        (0..5).map(|i| (i, [0.73, 0.8 + f64::from(i) * 0.1, 0.5]))).unwrap());
    wall.validate_body_pose([0.5, 1.0, 0.5], [0.0, 0.0, 0.0, 1.0]).unwrap();
    assert!(nuwax_nav_pct::same_source_geometry(&basis, &wall));
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
        planar_reference: None, snapshot: wall, change_basis: Some(basis),
        changes: Arc::new(SurfaceGeometryChanges::Empty),
    })).unwrap();
    follower.observe_applied_command(Twist2 {
        linear: nuwax_nav_types_messages::Point2::new(0.5, 0.0), angular_z: 0.0,
    });
    let deadline = follower.retry_after;
    let rejected = follower.step(0.3);
    assert!(Instant::now() < deadline, "fixture missed original 250ms cooldown");
    assert!(matches!(rejected, PlannerStep::Pending(_)), "{rejected:?}");
    assert!(follower.execution_wait_reason().is_some_and(|s| s.contains("当前轨迹受阻")));
    assert!(follower.local_path.is_some() && follower.admitted_generation.is_none());
    assert_eq!(follower.retry_after, deadline);
    follower.observe_applied_command(Twist2::ZERO);
}
fn recovery_clear_wall(follower: &mut RouteFollower) {
    let basis = follower.map.clone().unwrap();
    let cleared = Arc::new(basis.clear_recent_obstacles());
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
        planar_reference: None, snapshot: cleared, change_basis: Some(basis),
        changes: Arc::new(SurfaceGeometryChanges::Empty),
    })).unwrap();
}
fn recovery_cleanup(mut follower: RouteFollower, probe: &mut RecoveryLifecycleProbe) {
    follower.stop();
    if let Some(tx) = probe.release.take() { let _ = tx.send(()); let _ = tx.send(()); }
    drop(follower); // joins the original owner; observations are asserted afterward
    eprintln!("RECOVERY_CLEANUP original_owner_joined=true");
}

#[test]
fn rejected_active_nominal_gets_latest_detour_after_original_cooldown() {
    let mut follower = open_follower();
    assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
    let mut probe = install_recovery_lifecycle_probe(&mut follower, RecoveryFixtureResult::Incomplete);
    let epoch = follower.owner.as_ref().unwrap().epoch();
    let generation = follower.route_generation;
    let goal = follower.goal.clone(); let route = follower.route.clone();
    follower.dirty = true; // explicit background-refresh stimulus, no field resets after rejection
    follower.schedule(false); // actual admission starts the unchanged 250ms deadline
    let first = probe.entered.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(first, (false, epoch));
    let active_id = follower.submitted.unwrap().0;
    let original_deadline = follower.retry_after;
    recovery_publish_real_wall(&mut follower);
    assert_eq!(follower.submitted.unwrap().0, active_id);

    std::thread::sleep(original_deadline.saturating_duration_since(Instant::now()));
    let after_cooldown = follower.step(0.3);
    let pending = follower.submitted;
    let after_submit_calls = follower.local_planning_calls();
    let proof_ids: Vec<_> = follower.source_tickets.iter().flatten().map(|(id, _)| *id).collect();
    let no_second_active = matches!(probe.entered.try_recv(), Err(mpsc::TryRecvError::Empty));
    let same_epoch = follower.owner.as_ref().unwrap().epoch() == epoch;

    // Pass the new request's unchanged cooldown too, so duplicate suppression
    // is tested by the existing submitted-detour predicate, not just the timer.
    std::thread::sleep(follower.retry_after.saturating_duration_since(Instant::now()));
    for _ in 0..3 { assert!(matches!(follower.step(0.3), PlannerStep::Pending(_))); }
    let no_duplicates = follower.local_planning_calls() == after_submit_calls
        && follower.submitted == pending && follower.owner.as_ref().unwrap().epoch() == epoch;

    // Releasing the first active job must publish its non-cancelled result.
    probe.release.as_ref().unwrap().send(()).unwrap();
    let first_completion = probe.completed.recv_timeout(Duration::from_secs(5));
    wait_for(|| follower.owner.as_ref().unwrap().completion_ready_for_test());
    follower.consume_plan();
    let second = probe.entered.recv_timeout(Duration::from_secs(5));
    let retained = follower.goal == goal && follower.route == route
        && follower.route_generation == generation && follower.admitted_generation.is_none();
    recovery_cleanup(follower, &mut probe);

    assert!(matches!(after_cooldown, PlannerStep::Pending(_)));
    assert_eq!(pending.map(|(id, _, d)| (id, d)), Some((active_id + 1, true)));
    assert_eq!(proof_ids, vec![active_id, active_id + 1]);
    assert!(no_second_active && same_epoch && no_duplicates && retained);
    assert_eq!(first_completion.unwrap(), (false, false), "active nominal was cancelled");
    assert_eq!(second.unwrap(), (true, epoch), "latest pending did not become the next detour");
}

#[test]
fn readmitted_full_and_partial_restore_ordinary_refresh_mode() {
    for mode in [RecoveryFixtureResult::Full, RecoveryFixtureResult::Partial] {
        let mut follower = open_follower();
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
        let goal = follower.goal.clone(); let route = follower.route.clone();
        let generation = follower.route_generation;
        let plans = follower.local_plans();
        let mut probe = install_recovery_lifecycle_probe(&mut follower, mode);
        let epoch = follower.owner.as_ref().unwrap().epoch();
        follower.retry_after = Instant::now() + Duration::from_millis(250);
        recovery_publish_real_wall(&mut follower);
        recovery_clear_wall(&mut follower); // real latest evidence before the new request
        assert!(follower.local_path.is_some() && follower.admitted_generation.is_none());
        std::thread::sleep(follower.retry_after.saturating_duration_since(Instant::now()));
        assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
        let recovery_request = probe.entered.recv_timeout(Duration::from_secs(5));
        probe.release.as_ref().unwrap().send(()).unwrap();
        wait_for(|| follower.owner.as_ref().unwrap().completion_ready_for_test());
        follower.consume_plan(); // actual ticket, coverage, and latest connector admission
        let accepted = follower.local_plans() == plans + 1
            && follower.admitted_generation == Some(generation)
            && follower.goal == goal && follower.route == route;
        let coverage = follower.local_end;
        let total = *follower.cumulative.last().unwrap();

        follower.dirty = true; // trigger another refresh without replacing owner or clearing admission
        std::thread::sleep(follower.retry_after.saturating_duration_since(Instant::now()));
        let _ = follower.step(0.3);
        let normal_request = probe.entered.recv_timeout(Duration::from_secs(5));
        recovery_cleanup(follower, &mut probe);

        assert_eq!(recovery_request.unwrap(), (true, epoch));
        assert!(accepted, "synthetic valid candidate failed actual current admission");
        match mode {
            RecoveryFixtureResult::Partial => assert!((coverage - 0.7).abs() < 1e-9 && coverage < total),
            RecoveryFixtureResult::Full => assert!((coverage - total).abs() < 1e-9),
            RecoveryFixtureResult::Incomplete => unreachable!(),
        }
        assert_eq!(normal_request.unwrap(), (false, epoch), "successful re-admission retained a stale detour mode");
    }
}

#[test]
fn stop_and_new_route_clear_rejected_path_mode() {
    for stop_first in [false, true] {
        let mut follower = open_follower();
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
        let mut probe = install_recovery_lifecycle_probe(&mut follower, RecoveryFixtureResult::Incomplete);
        follower.retry_after = Instant::now() + Duration::from_millis(250);
        recovery_publish_real_wall(&mut follower);
        recovery_clear_wall(&mut follower);
        let old_generation = follower.route_generation;
        let old_epoch = follower.owner.as_ref().unwrap().epoch();
        if stop_first {
            follower.stop();
            assert!(follower.route.is_none() && follower.goal.is_none());
            assert!(follower.local_path.is_none() && follower.admitted_generation.is_none());
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            assert!(matches!(probe.entered.try_recv(), Err(mpsc::TryRecvError::Empty)));
            follower.configure_goal(&comparison_pose(3.0, 0), GoalOptions::default()).unwrap();
        }
        follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses: vec![
            comparison_pose(0.5, 0), comparison_pose(1.1, 0), comparison_pose(3.0, 0),
        ] }), None, comparison_pose(3.0, 0)).unwrap();
        let cleared = follower.local_path.is_none() && follower.admitted_generation.is_none()
            && follower.route_generation > old_generation
            && follower.owner.as_ref().unwrap().epoch() > old_epoch;
        let new_epoch = follower.owner.as_ref().unwrap().epoch();
        let next = follower.step(0.3);
        let request = probe.entered.recv_timeout(Duration::from_secs(5));
        recovery_cleanup(follower, &mut probe);
        assert!(cleared && matches!(next, PlannerStep::Pending(_)));
        assert_eq!(request.unwrap(), (false, new_epoch), "new route inherited old rejected-path detour intent");
    }
}


struct CooldownRequestProbe {
    entered: std::sync::mpsc::Receiver<bool>,
    release: Option<std::sync::mpsc::Sender<()>>,
}
impl Drop for CooldownRequestProbe {
    fn drop(&mut self) {
        // Unwind cannot strand the original bounded PlanningOwner thread.
        if let Some(release) = self.release.take() { let _ = release.send(()); }
    }
}
fn install_cooldown_request_probe(follower: &mut RouteFollower) -> CooldownRequestProbe {
    let (entered, receive) = std::sync::mpsc::channel();
    let (release, wait) = std::sync::mpsc::channel();
    follower.cancel_planning();
    let mut kernel = PlanKernel::new(follower.config.clone());
    follower.owner = Some(PlanningOwner::new(move |request: PlanRequest, cancel| {
        let _ = entered.send(request.detour);
        // Hold before PlanKernel. Cleanup cancels this same owner before release;
        // the production cancellation path then prevents unnecessary search.
        let _ = wait.recv_timeout(Duration::from_secs(5));
        kernel.compute(request, cancel)
    }).unwrap());
    CooldownRequestProbe { entered: receive, release: Some(release) }
}

#[test]
fn real_rejection_during_cooldown_keeps_detour_on_next_step() {
    let mut follower = open_follower();
    assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
    let old_route = follower.route.clone();
    let old_goal = follower.goal.clone();
    let generation = follower.route_generation;
    let old_path = follower.local_path.clone();
    assert!(old_path.is_some());

    // Geometry is identical to the existing selective recent-source regression.
    // The current footprint is clear; the commanded stopping tail meets the wall.
    let basis = follower.map.clone().unwrap();
    let wall = Arc::new(basis.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
        (0..5).map(|i| (i, [0.73, 0.8 + f64::from(i) * 0.1, 0.5]))).unwrap());
    assert!(nuwax_nav_pct::same_source_geometry(&basis, &wall));
    wall.validate_body_pose([0.5, 1.0, 0.5], [0.0, 0.0, 0.0, 1.0]).unwrap();

    let mut probe = install_cooldown_request_probe(&mut follower);
    let epoch = follower.owner.as_ref().unwrap().epoch();
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
        planar_reference: None, snapshot: wall, change_basis: Some(basis),
        changes: Arc::new(SurfaceGeometryChanges::Empty),
    })).unwrap();
    follower.observe_applied_command(Twist2 {
        linear: nuwax_nav_types_messages::Point2::new(0.5, 0.0), angular_z: 0.0,
    });
    // Synthetic scheduling phase, not a new production threshold or source time.
    let original_deadline = Instant::now() + Duration::from_millis(250);
    follower.retry_after = original_deadline;
    let rejected = follower.step(0.3); // real controller -> current_rejection
    assert!(Instant::now() < original_deadline, "fixture missed the original cooldown phase");
    assert!(matches!(rejected, PlannerStep::Pending(_)), "{rejected:?}");
    assert!(follower.execution_wait_reason().is_some_and(|s| s.contains("当前轨迹受阻")));
    assert!(follower.submitted.is_none(), "no job can enter during this cooldown");
    assert_eq!(probe.entered.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty));
    assert_eq!(follower.admitted_generation, None);
    assert_eq!(follower.retry_after, original_deadline);

    // Model the scheduler's acknowledged zero, preserving pose and obstacle.
    follower.observe_applied_command(Twist2::ZERO);
    std::thread::sleep(original_deadline.saturating_duration_since(Instant::now()));
    let next = follower.step(0.3); // never call current_rejection a second time
    let received = probe.entered.recv_timeout(Duration::from_secs(5));
    let retained = follower.route == old_route && follower.goal == old_goal
        && follower.route_generation == generation && follower.local_path == old_path
        && follower.admitted_generation.is_none()
        && follower.owner.as_ref().unwrap().epoch() == epoch;
    let submitted_detour = follower.submitted.map(|(_, _, detour)| detour);
    eprintln!("COOLDOWN_REQUEST observed={received:?} submitted={submitted_detour:?}");

    // Complete cleanup BEFORE the expected RED assertion.
    follower.stop();
    if let Some(release) = probe.release.take() { let _ = release.send(()); }
    drop(follower); // joins the same PlanningOwner, never a detached replacement
    eprintln!("COOLDOWN_CLEANUP original_owner_joined=true");

    assert!(matches!(next, PlannerStep::Pending(_)), "{next:?}");
    assert!(retained, "retry must preserve original goal/route and zero-only permission");
    assert_eq!(received.unwrap(), true, "actual PlanningOwner received nominal-first work");
    assert_eq!(submitted_detour, Some(true), "post-cooldown recovery lost detour intent");
}

#[test]
fn ordinary_incomplete_search_keeps_nominal_retry_without_guard_rejection() {
    let mut follower = open_follower();
    install_failures(&mut follower, typed_failures());
    complete_failure(&mut follower);
    assert!(follower.local_path.is_none());
    assert!(follower.admitted_generation.is_none());
    assert!(follower.qualified_failure.is_none());
    let mut probe = install_cooldown_request_probe(&mut follower);
    let next = follower.step(0.3);
    let received = probe.entered.recv_timeout(Duration::from_secs(5));
    let submitted_detour = follower.submitted.map(|(_, _, detour)| detour);
    eprintln!("COOLDOWN_REQUEST observed={received:?} submitted={submitted_detour:?}");
    follower.stop();
    if let Some(release) = probe.release.take() { let _ = release.send(()); }
    drop(follower);
    eprintln!("COOLDOWN_CLEANUP original_owner_joined=true");
    assert!(matches!(next, PlannerStep::Pending(_)));
    assert_eq!(submitted_detour, Some(false));
    assert_eq!(received.unwrap(), false, "ordinary incomplete work must retain nominal-first policy");
}

    use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
    use nuwax_nav_types_messages::{MotionKinematics, MotionModel, NavFrameId, Pose3, Quaternion, Timestamp};
    use crate::route_planning::Candidate;
    use std::sync::mpsc;
    fn comparison_config(enabled: Option<bool>) -> SurfaceFollowerConfig {
        let mut config = SurfaceFollowerConfig::new(0.4);
        if let Some(enabled) = enabled { config.local_planning_enabled = enabled; }
        config.controller.simulation = true;
        config.controller.speed = 0.5;
        config.motion_limits = Some(nuwax_nav_types_messages::MotionLimits {
            model: MotionModel::Differential, max_forward_mps: 0.5, max_reverse_mps: 0.0,
            max_lateral_mps: 0.0, max_angular_rps: 0.8, linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.05,
            minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
        });
        config
    }
    fn comparison_map(blocked: bool, observed_ns: Option<u64>) -> Arc<SurfaceSnapshot> {
        let map = crate::surface_follower::test_snapshot(0.0, blocked)
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
                max_step_up_m: 0.2, max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
            }).unwrap();
        if let Some(observed_ns) = observed_ns {
            use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
            let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
            for (x,y,z) in map.cell_keys() {
                let key = [x,y,z];
                blocks.entry(key.map(|v| v.div_euclid(32))).or_default().insert(key,
                    SurfaceObservation { observed_ns, hits: 2 });
            }
            let mut observations = SurfaceObservations::new(map.source_voxel_size_m(),32).unwrap();
            for (key,block) in blocks { observations.replace_block(key,block).unwrap(); }
            Arc::new(map.with_observations(Some(Arc::new(observations))))
        } else { Arc::new(map) }
    }
    fn comparison_pose(x: f64, tick: u32) -> PoseStamped {
        PoseStamped { frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1 + i64::from(tick / 20), tick % 20 * 50_000_000),
            pose: Pose3 { position: Point3::new(x,1.0,0.5), orientation: Quaternion::IDENTITY } }
    }
    fn comparison_route(follower: &mut RouteFollower, start: PoseStamped, goal: PoseStamped) {
        follower.update_odometry(start.clone());
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses: vec![start,goal.clone()] }), None,goal).unwrap();
    }
    // Waiting belongs only to this test module. Real deadlines keep a broken owner
    // from silently spinning while the control path itself remains nonblocking.
    fn wait_for(mut condition: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !condition() {
            assert!(Instant::now() < deadline, "local planner did not settle before test deadline");
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn ready_step(follower: &mut RouteFollower, width: f64) -> PlannerStep {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let step = follower.step(width);
            if !matches!(step, PlannerStep::Pending(_)) { return step; }
            assert!(Instant::now() < deadline, "local planner did not produce an executable result: {step:?}");
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn wait_for_plan(follower: &mut RouteFollower, previous: u64) {
        wait_for(|| {
            let step = follower.step(0.3);
            // The previous admitted controller may legitimately reject while its
            // replacement is still being computed. Only infrastructure failure is terminal here.
            assert!(!matches!(step, PlannerStep::Failed(_)), "local planning owner failed: {step:?}");
            follower.local_plans() > previous
        });
    }
    fn typed_failures() -> Vec<LocalPlanError> {
        [nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted,
         nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted]
            .into_iter().map(|kind| LocalPlanError { body_witness: None,
                kind, closest_pose: None, blocked: None, reason: "test incomplete search".into(),
            }).collect()
    }
    fn install_failures(follower: &mut RouteFollower, failures: Vec<LocalPlanError>) {
        follower.cancel_planning();
        follower.owner = Some(PlanningOwner::new(move |request, _| PlanResult {
            request, outcome: Err(failures.clone()),
        }).unwrap());
        follower.dirty = true;
    }
    fn complete_failure(follower: &mut RouteFollower) {
        follower.retry_after = Instant::now();
        follower.schedule(false);
        wait_for(|| follower.owner.as_ref().unwrap().completion_ready_for_test());
        follower.consume_plan();
    }
    struct PlannerGate {
        starts: mpsc::Receiver<()>,
        release: mpsc::Sender<()>,
    }
    impl PlannerGate {
        fn started(&self) {
            self.starts.recv_timeout(Duration::from_secs(5)).expect("planner did not start");
        }
        fn release(&self) { self.release.send(()).unwrap(); }
    }
    fn install_held_candidate(follower: &mut RouteFollower) -> PlannerGate {
        install_held_result(follower, None)
    }
    fn install_held_result(follower: &mut RouteFollower, failures: Option<Vec<LocalPlanError>>) -> PlannerGate {
        let (started, starts) = mpsc::channel();
        let (release, released) = mpsc::channel();
        let offset = follower.config.body_to_support_z_m;
        follower.cancel_planning();
        follower.owner = Some(PlanningOwner::new(move |mut request: PlanRequest, _| {
            started.send(()).unwrap();
            // Disconnection also releases the worker on an assertion's unwind.
            let _ = released.recv_timeout(Duration::from_secs(5));
            if let Some(failures) = &failures {
                return PlanResult { request, outcome: Err(failures.clone()) };
            }
            request.materialize_reference().unwrap();
            let path = Path3D { poses: request.reference.iter().map(|p| {
                let mut pose = request.pose.clone();
                pose.pose.position = Point3::new(p[0], p[1], p[2] + offset);
                pose
            }).collect() };
            PlanResult { request, outcome: Ok(Candidate { partial: false, path, elapsed_ms: 0.0 }) }
        }).unwrap());
        follower.dirty = true;
        PlannerGate { starts, release }
    }
    fn open_follower() -> RouteFollower {
        let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
        comparison_route(&mut follower, comparison_pose(0.5,0), comparison_pose(3.0,0));
        follower
    }
    #[test]
    fn incomplete_attempts_keep_goal_route_and_typed_evidence_without_penalty() {
        let mut follower = open_follower();
        install_failures(&mut follower, typed_failures());
        complete_failure(&mut follower);
        let goal = follower.goal.clone();
        let route = follower.route.clone();
        for _ in 0..3 {
            let PlannerStep::Pending(failures) = follower.step(0.3) else { panic!("no admitted trajectory must wait"); };
            assert_eq!(failures.len(), 2);
            assert_eq!(failures[0].kind, nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted);
            assert_eq!(failures[1].kind, nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted);
            assert!(failures.iter().all(|f| f.blocked.is_none()));
            assert_eq!(follower.goal, goal);
            assert_eq!(follower.route, route);
            assert!(follower.local_path_for_display().is_empty());
            assert!(follower.planner.local_costs().is_empty());
        }
    }
    #[test]
    fn unclassified_faults_do_not_become_incomplete_search_waits() {
        let mut follower = open_follower();
        let mut failures = typed_failures();
        failures[0] = "invalid local input".into();
        install_failures(&mut follower, failures);
        complete_failure(&mut follower);
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(reason)
            if reason.contains("invalid local input")));
        assert_eq!(follower.local_failures()[0].kind, nuwax_nav_scan::LocalPlanFailureKind::Unclassified);
        assert_eq!(follower.local_failures()[1].kind, nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted);
    }
    #[test]
    fn local_rejection_waits_through_existing_retry_cooldown_without_losing_route() {
        let mut follower = open_follower();
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
        follower.cancel_planning();
        let route = follower.route.clone();
        let goal = follower.goal.clone();
        let epoch = follower.owner.as_ref().unwrap().epoch();
        let generation = follower.route_generation;
        let plans = follower.local_plans();
        let calls = follower.local_planning_calls();
        follower.retry_after = Instant::now() + Duration::from_millis(250);
        assert!(matches!(follower.current_rejection("current stopping trajectory blocked".into()), PlannerStep::Pending(_)));
        assert!(follower.submitted.is_none());
        assert_eq!(follower.local_planning_calls(), calls);
        assert_eq!(follower.owner.as_ref().unwrap().epoch(), epoch);
        assert_eq!(follower.route, route);
        assert_eq!(follower.goal, goal);
        assert!(follower.execution_wait_reason().unwrap().contains("current stopping trajectory blocked"));
        assert_eq!(follower.admitted_generation, None);
        for _ in 0..3 {
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)),
                "rejected old admission must not emit a Command, even ZERO, during cooldown");
            assert!(follower.submitted.is_none());
            assert_eq!(follower.owner.as_ref().unwrap().epoch(), epoch);
            assert_eq!(follower.route_generation, generation);
        }
        std::thread::sleep(Duration::from_millis(260));
        assert!(matches!(follower.current_rejection("current stopping trajectory blocked".into()), PlannerStep::Pending(_)));
        assert!(follower.submitted.is_some());
        assert_eq!(follower.local_planning_calls(), calls + 1);
        assert_eq!(follower.owner.as_ref().unwrap().epoch(), epoch);
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
        assert_eq!(follower.admitted_generation, Some(generation));
        assert!(follower.local_plans() > plans);
        assert_eq!(follower.route, route);
        assert_eq!(follower.goal, goal);
    }
    #[test]
    fn local_rejection_does_not_mask_preparation_permanent_or_qualified_faults() {
        for fault in 0..3 {
            let mut follower = open_follower();
            assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
            match fault {
                0 => follower.preparation_fault=Some("preparation fault".into()),
                1 => follower.permanent_fault=Some("permanent fault".into()),
                _ => follower.qualified_failure=Some("qualified failure".into()),
            }
            assert_eq!(follower.current_rejection("current rejection".into()),
                PlannerStep::ReplanRequired("current rejection".into()));
        }
    }
    #[test]
    fn unavailable_kinematics_is_a_real_preparation_error_not_pending() {
        let mut follower = open_follower();
        follower.config.motion_limits = None;
        follower.config.basic_kinematics = None;
        assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(reason)
            if reason.contains("缺少运动能力")));
        assert_eq!(follower.local_failures().len(),1);
        assert!(follower.local_failures().iter().all(|f|
            f.kind == nuwax_nav_scan::LocalPlanFailureKind::Unclassified));
    }
    #[test]
    fn replacement_cannot_reuse_old_motion_arrival_or_display_on_budget_failure() {
        for at_old_goal in [false,true] {
            let mut follower = open_follower();
            assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
            assert!(!follower.local_path_for_display().is_empty());
            if at_old_goal { follower.update_odometry(comparison_pose(3.0,1)); }
            let new_goal = comparison_pose(4.0,1);
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses: vec![follower.pose.clone().unwrap(),new_goal.clone()]
            }), None, new_goal.clone()).unwrap();
            install_failures(&mut follower, typed_failures());
            complete_failure(&mut follower);
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            assert_eq!(follower.goal.as_ref(), Some(&new_goal));
            assert!(follower.local_path_for_display().is_empty());
            assert!(follower.local_path().is_none());
            follower.configure_goal(&comparison_pose(5.0,2), GoalOptions::default()).unwrap();
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            assert!(follower.local_path_for_display().is_empty());
        }
    }
    #[test]
    fn current_admission_keeps_fresh_motion_arrival_and_boundary_rejection() {
        for outcome in 0..3 {
            let mut follower = open_follower();
            assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
            if outcome == 1 { follower.update_odometry(comparison_pose(3.0,1)); }
            if outcome == 2 {
                follower.configure_boundary(&[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
            }
            install_failures(&mut follower, typed_failures());
            complete_failure(&mut follower);
            let step = follower.step(0.3);
            match outcome {
                0 => assert!(matches!(step,PlannerStep::Command(c) if c.linear.x > 0.0)),
                1 => assert_eq!(step,PlannerStep::Succeeded),
                _ => assert!(matches!(step,PlannerStep::ReplanRequired(_)), "fresh boundary rejection lost: {step:?}"),
            }
            assert_eq!(follower.local_failures().len(),2);
            assert!(follower.planner.local_costs().is_empty());
        }
    }
    #[test]
    fn failed_reconnection_preserves_the_existing_fresh_controller_rejection_verbatim() {
        let prepare = || {
            let mut follower = open_follower();
            assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
            follower.configure_boundary(&[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
            install_failures(&mut follower, typed_failures());
            complete_failure(&mut follower);
            follower
        };
        let mut oracle = prepare();
        let expected = oracle.controller.step(0.3);
        assert!(matches!(expected,PlannerStep::ReplanRequired(_)));
        let mut follower = prepare();
        assert_eq!(follower.step(0.3), expected);
        assert_eq!(follower.local_failures().len(),2);
    }

    fn refresh_observation_metadata(source: &SurfaceSnapshot, observed_ns: u64) -> Arc<SurfaceSnapshot> {
        use nuwax_nav_mls_planner::SurfaceObservation;
        let mut observations = source.observations().unwrap().as_ref().clone();
        let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
        for (x,y,z) in source.cell_keys() {
            let key = [x,y,z];
            blocks.entry(key.map(|v| v.div_euclid(32))).or_default()
                .insert(key, SurfaceObservation { observed_ns, hits: 3 });
        }
        for (key,block) in blocks { observations.replace_block(key,block).unwrap(); }
        Arc::new(source.with_observations(Some(Arc::new(observations))))
    }
    #[test]
    fn first_candidate_is_pending_while_owner_is_held_then_admitted_once() {
        let mut follower = open_follower();
        let gate = install_held_candidate(&mut follower);
        assert_eq!(follower.step(0.3), PlannerStep::Pending(Vec::new()));
        gate.started();
        let submitted = follower.local_planning_calls();
        for _ in 0..20 {
            assert_eq!(follower.step(0.3), PlannerStep::Pending(Vec::new()));
            assert!(follower.local_path_for_display().is_empty());
            assert_eq!(follower.local_plans(), 0);
        }
        assert_eq!(follower.local_planning_calls(), submitted);
        assert!(gate.starts.try_recv().is_err());
        gate.release();
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
        assert_eq!(follower.local_plans(), 1);
        assert!(!follower.local_path_for_display().is_empty());
    }
    #[test]
    fn committed_route_keeps_current_control_while_replacement_is_held() {
        let mut follower = open_follower();
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
        let previous_path = follower.local_path().cloned();
        let previous_plans = follower.local_plans();
        let gate = install_held_candidate(&mut follower);
        assert!(matches!(follower.step(0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
        gate.started();
        let submitted = follower.local_planning_calls();
        for _ in 0..20 {
            assert!(matches!(follower.step(0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
            assert_eq!(follower.local_path(), previous_path.as_ref());
            assert_eq!(follower.local_plans(), previous_plans);
        }
        assert_eq!(follower.local_planning_calls(), submitted);
        gate.release();
        wait_for_plan(&mut follower, previous_plans);
    }
    #[test]
    fn metadata_refresh_does_not_cancel_or_restart_a_held_candidate() {
        let source = comparison_map(false, Some(1_000_000_000));
        let refreshed = refresh_observation_metadata(&source, 2_000_000_000);
        assert!(nuwax_nav_pct::same_source_geometry(&source, &refreshed));
        assert!(!source.same_body_query_view(&refreshed));
        let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
        comparison_route(&mut follower, comparison_pose(0.5,0), comparison_pose(3.0,0));
        let gate = install_held_candidate(&mut follower);
        assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
        gate.started();
        let epoch = follower.owner.as_ref().unwrap().epoch();
        let submitted = follower.local_planning_calls();
        follower.update_execution_map(ExecutionMap::Surface(refreshed)).unwrap();
        let mut same_pose = comparison_pose(0.5,1);
        same_pose.stamp = Timestamp::from_unix(10,0);
        follower.update_odometry(same_pose);
        for _ in 0..20 { assert!(matches!(follower.step(0.3), PlannerStep::Pending(_))); }
        assert_eq!(follower.owner.as_ref().unwrap().epoch(), epoch);
        assert_eq!(follower.local_planning_calls(), submitted);
        assert!(gate.starts.try_recv().is_err());
        gate.release();
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
        assert_eq!(follower.local_plans(), 1);
    }

    #[test]
    fn exact_mask_reuse_preserves_held_local_work_but_mask_fine_and_owner_changes_cancel() {
        for change in 0..4 {
            let physical = comparison_map(false, Some(1_000_000_000));
            let blocked = vec![false; 60 * 30];
            let source = Arc::new(physical.with_virtual_obstacles([0.0,0.0],0.1,60,30,&blocked).unwrap());
            let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
            comparison_route(&mut follower, comparison_pose(0.5,0), comparison_pose(3.0,0));
            let gate = install_held_candidate(&mut follower);
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            gate.started();
            let epoch = follower.owner.as_ref().unwrap().epoch();
            let calls = follower.local_planning_calls();
            let next = match change {
                0 => {
                    assert!(source.matches_virtual_obstacles([0.0,0.0],0.1,60,30,blocked.iter().copied()));
                    refresh_observation_metadata(&source, 2_000_000_000)
                }
                1 => {
                    let mut changed = blocked.clone(); changed[2 * 60 + 40] = true;
                    Arc::new(physical.with_virtual_obstacles([0.0,0.0],0.1,60,30,&changed).unwrap())
                }
                2 => {
                    let mut observations = source.observations().unwrap().as_ref().clone();
                    observations.replace_block([2,0,0], std::collections::BTreeMap::from([
                        ([70,1,0], nuwax_nav_mls_planner::SurfaceObservation { observed_ns:2_000_000_000, hits:2 })
                    ])).unwrap();
                    let next = Arc::new(source.with_observations(Some(Arc::new(observations))));
                    assert!(source.same_geometry(&next));
                    assert!(!nuwax_nav_pct::same_source_geometry(&source,&next));
                    next
                }
                _ => Arc::new(comparison_map(false,Some(2_000_000_000))
                    .with_virtual_obstacles([0.0,0.0],0.1,60,30,&blocked).unwrap()),
            };
            follower.update_execution_map(ExecutionMap::Surface(next)).unwrap();
            if change == 0 {
                assert_eq!(follower.owner.as_ref().unwrap().epoch(), epoch);
                assert_eq!(follower.local_planning_calls(), calls);
                gate.release();
                assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0));
                assert_eq!(follower.local_plans(),1);
            } else {
                assert!(follower.owner.as_ref().unwrap().epoch()>epoch);
                gate.release();
                wait_for(||follower.owner.as_ref().unwrap().is_idle_for_test());
                follower.consume_plan();
                assert_eq!(follower.local_plans(),0);
                assert!(follower.local_path().is_none());
            }
        }
    }
    #[test]
    fn moved_start_candidate_is_rejected_without_teleporting_or_losing_admission() {
        for committed in [false,true] {
            let mut follower = open_follower();
            if committed { assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_))); }
            let previous_path = follower.local_path().cloned();
            let previous_plans = follower.local_plans();
            let previous_admission = follower.admitted_generation;
            let gate = install_held_candidate(&mut follower);
            follower.step(0.3);
            gate.started();
            let mut moved = comparison_pose(0.5,1);
            moved.pose.position.y = 2.5;
            follower.update_odometry(moved.clone());
            gate.release();
            wait_for(|| follower.owner.as_ref().unwrap().completion_ready_for_test());
            follower.consume_plan();
            assert_eq!(follower.pose.as_ref(), Some(&moved));
            assert_eq!(follower.local_path(), previous_path.as_ref());
            assert_eq!(follower.local_plans(), previous_plans);
            assert_eq!(follower.admitted_generation, previous_admission);
            assert!(follower.local_failures().iter().any(|e| e.reason.contains("接续")));
            assert!(follower.planner.local_costs().is_empty());
        }
    }
    #[test]
    fn lifecycle_invalidations_discard_held_late_completions() {
        for failure in [false,true] { for change in 0..6 {
            let mut follower = open_follower();
            let gate = install_held_result(&mut follower, failure.then(|| vec!["obsolete invalid input".into()]));
            assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
            gate.started();
            let epoch = follower.owner.as_ref().unwrap().epoch();
            match change {
                0 => follower.stop(),
                1 => follower.configure_goal(&comparison_pose(4.0,1), GoalOptions::default()).unwrap(),
                2 => follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                    poses: vec![comparison_pose(0.5,1), comparison_pose(4.0,1)],
                }), None, comparison_pose(4.0,1)).unwrap(),
                3 => follower.update_execution_map(ExecutionMap::Surface(comparison_map(true,None))).unwrap(),
                4 => follower.configure_boundary(&[[0.0,0.0],[6.0,0.0],[6.0,3.0],[0.0,3.0]]),
                _ => follower.invalidate_execution_map(),
            }
            assert!(follower.owner.as_ref().unwrap().epoch() > epoch, "change {change} did not cancel old work");
            gate.release();
            wait_for(|| follower.owner.as_ref().unwrap().is_idle_for_test());
            follower.consume_plan();
            assert_eq!(follower.local_plans(), 0, "change {change} accepted an obsolete completion");
            assert!(follower.local_path().is_none());
            assert!(follower.admitted_generation.is_none());
            assert!(follower.local_failures().is_empty());
            assert!(follower.planner.local_costs().is_empty());
        } }
    }
    #[test]
    fn owner_panic_is_visible_without_restart_or_motion() {
        let mut follower = open_follower();
        follower.owner = Some(PlanningOwner::new(|_: PlanRequest, _| -> PlanResult {
            panic!("injected route owner panic")
        }).unwrap());
        assert!(matches!(follower.step(0.3), PlannerStep::Pending(_)));
        wait_for(|| follower.owner.as_ref().unwrap().faulted());
        let submitted = follower.local_planning_calls();
        for _ in 0..3 {
            assert!(matches!(follower.step(0.3), PlannerStep::Failed(reason)
                if reason.contains("owner terminated")));
            assert_eq!(follower.local_planning_calls(), submitted);
            assert_eq!(follower.local_plans(), 0);
            assert!(follower.local_path_for_display().is_empty());
        }
    }
    #[test]
    fn fresh_controller_rejection_is_returned_verbatim_while_planner_is_held() {
        let prepare = || {
            let mut follower = open_follower();
            assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
            follower.configure_boundary(&[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
            follower
        };
        let mut oracle = prepare();
        let expected = oracle.controller.step(0.3);
        assert!(matches!(expected, PlannerStep::ReplanRequired(_)));
        let mut follower = prepare();
        let gate = install_held_candidate(&mut follower);
        assert_eq!(follower.step(0.3), expected);
        gate.started();
        assert!(!follower.owner.as_ref().unwrap().completion_ready_for_test());
        assert_eq!(follower.local_plans(), 1);
        assert!(follower.planner.local_costs().is_empty());
        gate.release();
    }
    #[test]
    fn stale_unclassified_failure_does_not_revoke_current_admission() {
        for consume_while_current in [false,true] { for metadata_change in [false,true] {
            let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
            let source = comparison_map(false, Some(1_000_000_000));
            follower.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
            comparison_route(&mut follower, comparison_pose(0.5,0), comparison_pose(3.0,0));
            assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(_)));
            install_failures(&mut follower, vec!["invalid old input".into()]);
            follower.schedule(false);
            wait_for(|| follower.owner.as_ref().unwrap().completion_ready_for_test());
            if consume_while_current {
                follower.consume_plan();
                assert!(follower.qualified_failure.is_some());
                assert!(matches!(follower.step(0.3), PlannerStep::ReplanRequired(reason)
                    if reason.contains("invalid old input")));
            }
            if metadata_change {
                follower.update_execution_map(ExecutionMap::Surface(refresh_observation_metadata(&source, 2_000_000_000))).unwrap();
            } else {
                follower.update_odometry(comparison_pose(0.51,1));
            }
            assert!(matches!(follower.step(0.3), PlannerStep::Command(c) if c.linear.x > 0.0));
            assert!(follower.planner.local_costs().is_empty());
            assert_eq!(follower.local_plans(), 1);
            assert!(follower.qualified_failure.is_none());
        } }
    }

    #[test]
    fn local_planning_default_and_explicit_on_match_and_off_progresses_without_planning() {
        let mut traces = Vec::new();
        for enabled in [None,Some(true),Some(false)] {
            let mut follower = RouteFollower::new(comparison_config(enabled)).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
            let mut actual = comparison_pose(0.5,0);
            comparison_route(&mut follower, actual.clone(),comparison_pose(3.0,0));
            let mut commands = Vec::new();
            let mut arrived = false;
            for tick in 1..=140 {
                let step = ready_step(&mut follower, 0.3);
                if step == PlannerStep::Succeeded { arrived = true; break; }
                let PlannerStep::Command(command) = step else { panic!("clear route must progress: mode={enabled:?} tick={tick} pose={actual:?} step={step:?}"); };
                assert!(command.linear.x >= 0.0);
                commands.push(command);
                actual.pose.position.x += command.linear.x * 0.05;
                actual.stamp = comparison_pose(0.0,tick).stamp;
                follower.observe_applied_command(command);
                follower.update_odometry(actual.clone());
            }
            assert!(arrived, "mode {enabled:?} did not reach its goal");
            assert!(actual.pose.position.x > 0.8, "mode {enabled:?} did not move");
            assert!(follower.progress_m() > 0.2);
            if enabled == Some(false) {
                assert_eq!(follower.local_planning_calls(),0);
                assert_eq!(follower.local_plans(),0);
                assert!(follower.owner.is_none() && follower.local_path.is_none());
                follower.schedule(false);
                assert_eq!(follower.local_planning_calls(),0);
            } else { assert!(follower.local_planning_calls() > 0); }
            traces.push(commands);
            follower.stop();
            assert!(!matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0));
        }
        assert_eq!(traces[0],traces[1],"default must preserve explicit ON behavior");
    }
    #[test]
    fn both_modes_keep_obstacle_braking_and_sparse_route_deviation_checks() {
        for enabled in [true,false] {
            let clear = comparison_map(false,None);
            let mut points = Vec::new();
            for ix in 0..65 { for iy in 0..35 {
                points.push(((ix as f32+0.5)*0.1,(iy as f32+0.5)*0.1,0.05));
            } }
            for iy in 0..35 { for iz in 1..8 { points.push((0.75,(iy as f32+0.5)*0.1,(iz as f32+0.5)*0.1)); } }
            let blocked = Arc::new(SurfaceSnapshot::from_points(&points,clear.config().clone(),0.1).unwrap()
                .with_body_envelope(clear.body_envelope().unwrap().clone()).unwrap());
            let mut follower = RouteFollower::new(comparison_config(Some(enabled))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();
            comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
            assert!(matches!(ready_step(&mut follower, 0.3),PlannerStep::Command(c) if c.linear.x>0.0));
            // The physical receipt remains authoritative even when planning is off.
            follower.observe_applied_command(Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0});
            follower.update_odometry(comparison_pose(0.5,1));
            follower.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
            let step = follower.step(0.3);
            assert!(!matches!(step,PlannerStep::Command(c) if c.linear.x>0.0),"new wall must stop the braking envelope: mode={enabled} {step:?}");
            if !enabled { assert_eq!(follower.local_planning_calls(),0); }
        }
        let mut follower = RouteFollower::new(comparison_config(Some(false))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
        comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
        let mut off_route = comparison_pose(1.5,1);
        off_route.pose.position.y = 2.5;
        follower.update_odometry(off_route);
        assert!(matches!(follower.step(0.3),PlannerStep::ReplanRequired(_)));
        assert_eq!(follower.local_planning_calls(),0);
    }

    #[test]
    fn off_sparse_route_rejects_a_valid_body_on_a_different_floor() {
        let clear = comparison_map(false,None);
        let mut points = Vec::new();
        for ix in 0..65 { for iy in 0..35 { for z in [0.05,1.55] {
            points.push(((ix as f32+0.5)*0.1,(iy as f32+0.5)*0.1,z));
        } } }
        let floors = Arc::new(SurfaceSnapshot::from_points(&points,clear.config().clone(),0.1).unwrap()
            .with_body_envelope(clear.body_envelope().unwrap().clone()).unwrap());
        let mut follower = RouteFollower::new(comparison_config(Some(false))).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(floors)).unwrap();
        comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
        let mut upper_floor = comparison_pose(1.5,1);
        upper_floor.pose.position.z = 2.0;
        follower.update_odometry(upper_floor);
        assert!(follower.controller.current_body_error().is_none(),"upper-floor pose must have valid support and clearance");
        assert!(matches!(follower.step(0.3),PlannerStep::ReplanRequired(reason) if reason.contains("偏离连续三维路线")),
            "a valid body on another floor is not on the lower route's 3D segment");
        assert_eq!(follower.local_planning_calls(),0);
    }

    #[test]
    fn both_modes_allow_missing_ground_without_inventing_obstacles() {
        for enabled in [true,false] {
            let mut follower = RouteFollower::new(comparison_config(Some(enabled))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
            comparison_route(&mut follower,comparison_pose(0.95,0),comparison_pose(3.0,0));
            assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0));
            // This fixture removes ground samples; it does not measure a cliff or obstacle.
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(true,None))).unwrap();
            follower.update_odometry(comparison_pose(0.95,1));
            assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0),
                "missing ground must remain an admissible candidate: mode={enabled}");
            assert!(follower.local_failures().iter().all(|failure|failure.blocked.is_none()));
            assert!(follower.planner.local_costs().is_empty());
            if !enabled {assert_eq!(follower.local_planning_calls(),0);}
        }
    }

    #[test]
    fn both_modes_reject_known_support_obstacle_but_accept_old_ground() {
        let clear = comparison_map(false,None);
        let points: Vec<_> = clear.cell_keys().map(|(x,y,z)| {
            // A measured 40 cm rise exceeds this body's 20 cm step capability
            // and intersects its current footprint. Keep occupancy evidence.
            let z = if (10..=13).contains(&x) {z+4} else {z};
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,(z as f32+0.5)*0.1)
        }).collect();
        let raised = Arc::new(SurfaceSnapshot::from_points(&points,clear.config().clone(),0.1).unwrap()
            .with_body_envelope(clear.body_envelope().unwrap().clone()).unwrap());
        assert!(raised.validate_body_pose([0.95,1.0,0.5],[0.0,0.0,0.0,1.0]).is_err(),
            "fixture must contain measured incompatible body geometry");
        for enabled in [true,false] {
            let mut follower = RouteFollower::new(comparison_config(Some(enabled))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(clear.clone())).unwrap();
            comparison_route(&mut follower,comparison_pose(0.95,0),comparison_pose(3.0,0));
            assert!(matches!(ready_step(&mut follower, 0.3),PlannerStep::Command(c) if c.linear.x > 0.0));
            follower.update_execution_map(ExecutionMap::Surface(raised.clone())).unwrap();
            follower.update_odometry(comparison_pose(0.95,1));
            assert!(!matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"known incompatible support allowed motion, mode={enabled}");
            let mut config = comparison_config(Some(enabled));
            config.max_observation_age_sec = Some(1.0);
            let mut follower = RouteFollower::new(config).unwrap();
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,Some(now)))).unwrap();
            comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
            assert!(matches!(ready_step(&mut follower, 0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"fresh evidence should move, mode={enabled}");
            // Old static ground remains support; live stream health is enforced by the scheduler.
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,Some(now.saturating_sub(2_000_000_000))))).unwrap();
            follower.update_odometry(comparison_pose(0.5,1));
            assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"old ground must remain usable, mode={enabled}");
            if !enabled { assert_eq!(follower.local_planning_calls(),0); }
        }
    }

    #[test]
    fn invalid_current_support_waits_without_replanning_and_recovers_on_pose_update() {
        let map = Arc::new(crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
                max_step_up_m: 0.2, max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
            }).unwrap());
        let make = |x,z| PoseStamped { frame_id: NavFrameId::Map, stamp: Timestamp::from_unix(1,0),
            pose: Pose3 { position: Point3::new(x,1.0,z), orientation: Quaternion::IDENTITY } };
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.simulation = true;
        config.basic_kinematics = Some(MotionKinematics { model: MotionModel::Differential,
            max_forward_mps: 1.0, max_reverse_mps: 0.0, max_lateral_mps: 0.0,
            max_angular_rps: 0.8, minimum_turning_radius_m: 0.0, can_rotate_in_place: true });
        let mut follower = RouteFollower::new(config).unwrap();
        let start = make(0.5,0.5);
        let goal = make(3.0,0.5);
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(make(0.5,2.0));
        follower.configure_goal(&goal, GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses: vec![start.clone(),goal.clone()] }), None, goal).unwrap();
        for _ in 0..3 {
            assert_eq!(follower.step(0.3), PlannerStep::Pending(Vec::new()));
            assert!(follower.execution_wait_reason().unwrap().contains("当前机体几何检查未通过"));
            assert_eq!(follower.local_plans(),0);
            assert!(follower.local_failures().is_empty());
        }
        follower.update_odometry(start.clone());
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(command) if command != Twist2::ZERO));
        assert!(follower.execution_wait_reason().is_none());
        assert_eq!(follower.local_plans(),1);
        // 中途输入再次失效时也保留同一参考，恢复后不重复生成局部路线。
        follower.update_odometry(make(0.5,2.0));
        assert_eq!(follower.step(0.3), PlannerStep::Pending(Vec::new()));
        follower.update_odometry(start);
        assert!(matches!(ready_step(&mut follower, 0.3), PlannerStep::Command(command) if command != Twist2::ZERO));
        assert_eq!(follower.local_plans(),1);
        follower.stop();
        assert!(follower.execution_wait_reason().is_none());
        assert!(follower.route.is_none());
    }

    #[test]
    fn straight_global_route_stays_straight_after_lateral_tracking_error() {
        let map=Arc::new(crate::surface_follower::test_snapshot(0.0,false)
            .with_body_envelope(SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
                max_cross_slope_deg:18.0,support_uncertainty_m:0.0}).unwrap());
        let make=|x,y|PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,0),
            pose:Pose3{position:Point3::new(x,y,0.5),orientation:Quaternion::IDENTITY}};
        let mut config=SurfaceFollowerConfig::new(0.4);config.controller.simulation=true;
        config.basic_kinematics=Some(MotionKinematics{model:MotionModel::Differential,max_forward_mps:1.0,
            max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:0.8,minimum_turning_radius_m:0.0,can_rotate_in_place:true});
        let mut follower=RouteFollower::new(config).unwrap();
        let actual=make(0.5,1.2);let goal=make(5.0,1.0);
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(actual.clone());follower.configure_goal(&goal,GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![make(0.5,1.0),make(2.0,1.0),goal.clone()]}),None,goal).unwrap();
        let previous = follower.local_plans();
        wait_for_plan(&mut follower, previous);
        let path=follower.local_path().unwrap();let end=path.poses.last().unwrap().pose.position;
        let start=actual.pose.position;let dx=end.x-start.x;let dy=end.y-start.y;
        assert!(path.poses.iter().all(|p|((p.pose.position.x-start.x)*dy-(p.pose.position.y-start.y)*dx).abs()<1e-8),
            "全局仍是直线时，小横差不能重新生成S形局部轨迹");
        let goal=make(2.0,2.0);
        follower.configure_goal(&goal,GoalOptions::default()).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![make(0.5,1.0),make(2.0,1.0),goal.clone()]}),None,goal).unwrap();
        let previous = follower.local_plans();
        wait_for_plan(&mut follower, previous);
        let path=follower.local_path().unwrap();let end=path.poses.last().unwrap().pose.position;
        let dx=end.x-start.x;let dy=end.y-start.y;
        assert!(path.poses.iter().any(|p|((p.pose.position.x-start.x)*dy-(p.pose.position.y-start.y)*dx).abs()>0.05),
            "真实全局转角仍应保留转弯，不能一律切成直线");
    }

    #[test]
    fn nominal_end_does_not_replace_the_actual_three_dimensional_goal_ball() {
        let map = Arc::new(
            crate::surface_follower::test_snapshot(0.0, false)
                .with_body_envelope(SurfaceBodyEnvelope {
                    footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                    z_bounds_m: [-0.05, 0.2],
                    body_to_support_z_m: 0.4,
                    max_step_up_m: 0.2,
                    max_step_down_m: 0.2,
                    max_cross_slope_deg: 18.0,
                    support_uncertainty_m: 0.0,
                })
                .unwrap(),
        );
        let make = |x, z| PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(x, 1.0, z),
                orientation: Quaternion::from_yaw(0.0),
            },
        };
        let goal = make(3.0, 0.5);
        let mut actual = make(1.0, 0.55);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.simulation = true;
        config.basic_kinematics = Some(MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        });
        let mut follower = RouteFollower::new(config).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map))
            .unwrap();
        follower.update_odometry(actual.clone());
        follower
            .configure_goal(
                &goal,
                GoalOptions {
                    position_tolerance_m: 0.35,
                    ..Default::default()
                },
            )
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![actual.clone(), make(2.652, 0.5)],
                }),
                None,
                goal.clone(),
            )
            .unwrap();
        let mut arrived = false;
        for i in 0..300 {
            match ready_step(&mut follower, 0.3) {
                PlannerStep::Command(command) => {
                    assert_eq!(command.linear.y, 0.0);
                    assert!(command.linear.x >= 0.0);
                    let yaw = actual.pose.orientation.yaw();
                    actual.pose.position.x += command.linear.x * yaw.cos() * 0.05;
                    actual.pose.position.y += command.linear.x * yaw.sin() * 0.05;
                    actual.pose.orientation = Quaternion::from_yaw(yaw + command.angular_z * 0.05);
                    actual.stamp = Timestamp::from_unix(2 + i / 20, (i % 20) as u32 * 50_000_000);
                    follower.observe_applied_command(command);
                    follower.update_odometry(actual.clone());
                }
                PlannerStep::Succeeded => {
                    arrived = true;
                    break;
                }
                other => panic!("名义末点不能使有效目标反复重规划：{other:?}"),
            }
        }
        assert!(arrived);
        assert!(actual.pose.position.distance(goal.pose.position) < 0.35);
    }
    fn rolling_window_follower() -> (RouteFollower,Arc<SurfaceSnapshot>) {
        let envelope = SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
            z_bounds_m: [-0.05, 0.2],
            body_to_support_z_m: 0.4,
            max_step_up_m: 0.2,
            max_step_down_m: 0.2,
            max_cross_slope_deg: 18.0,
            support_uncertainty_m: 0.0,
        };
        let make = |x| PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(x, 1.0, 0.5),
                orientation: Quaternion::from_yaw(0.0),
            },
        };
        let start = make(0.5);
        // 目标须在默认五米窗口之外，本例才会覆盖滚动更新而非末段跟随。
        let goal = make(6.0);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.simulation = true;
        config.scan.refresh_distance_m = 0.15;
        config.basic_kinematics = Some(MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        });
        let mut follower = RouteFollower::new(config).unwrap();
        let clear = Arc::new(
            crate::surface_follower::test_snapshot(0.0, false)
                .with_body_envelope(envelope.clone())
                .unwrap(),
        );
        follower
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&clear)))
            .unwrap();
        follower.update_odometry(start.clone());
        follower
            .configure_goal(&goal, GoalOptions::default())
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![start, goal.clone()],
                }),
                None,
                goal,
            )
            .unwrap();
        assert!(matches!(ready_step(&mut follower, 0.2), PlannerStep::Command(_)));
        assert!(follower.local_end < *follower.cumulative.last().unwrap());
        (follower,clear)
    }

    #[test]
    fn missing_forward_ground_refreshes_window_without_inventing_obstruction() {
        let (mut follower,clear) = rolling_window_follower();
        let previous_local = follower.local_plans();
        let missing = Arc::new(
            crate::surface_follower::test_snapshot(0.0, true)
                .with_body_envelope(clear.body_envelope().unwrap().clone())
                .unwrap(),
        );
        follower
            .update_execution_map(ExecutionMap::Surface(missing))
            .unwrap();
        let mut moved = comparison_pose(0.7,0);
        moved.stamp = Timestamp::from_unix(2, 0);
        follower.update_odometry(moved);
        let outcome = follower.step(0.2);
        assert!(matches!(outcome,PlannerStep::Command(c) if c.linear.x>0.0),
            "missing samples must not revoke fresh guarded control: {outcome:?}");
        // Exercise the real Scan algorithm across the missing strip: unknown
        // ground can produce a candidate, so waiting for search failure is wrong.
        wait_for_plan(&mut follower,previous_local);
        assert!(follower.local_failures().is_empty());
        assert!(follower.planner.local_costs().is_empty(), "missing samples must not penalize PCT");
        assert!(follower.local_path().is_some());
        let refreshed = follower.local_plans();
        follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();
        wait_for_plan(&mut follower,refreshed);
        assert!(matches!(follower.step(0.2),PlannerStep::Command(c) if c.linear.x>0.0));
        follower.stop();
        assert!(follower.local_failures().is_empty());
    }

    #[test]
    fn incomplete_forward_window_keeps_fresh_control_and_recovers() {
        let (mut follower,_) = rolling_window_follower();
        let previous_local = follower.local_plans();
        let previous_path = follower.local_path().cloned();
        let route = follower.route.clone();
        let goal = follower.goal.clone();
        // Typed failure injection tests owner/result handling, not geometry.
        // Preserve the real idle owner for an algorithm-backed recovery below.
        let real_owner = follower.owner.take().unwrap();
        assert!(real_owner.is_idle_for_test());
        install_failures(&mut follower,typed_failures());
        follower.update_odometry(comparison_pose(0.7,1));
        complete_failure(&mut follower);
        assert!(matches!(follower.step(0.2),PlannerStep::Command(c) if c.linear.x>0.0));
        assert_eq!(follower.local_failures(),typed_failures());
        assert!(follower.planner.local_costs().is_empty(), "incomplete search must not penalize PCT");
        assert_eq!(follower.local_plans(),previous_local);
        assert_eq!(follower.local_path(),previous_path.as_ref());
        assert_eq!(follower.route,route);
        assert_eq!(follower.goal,goal);
        follower.cancel_planning();
        follower.owner=Some(real_owner);
        follower.dirty=true;
        wait_for_plan(&mut follower,previous_local);
        assert!(follower.local_failures().is_empty());
        assert!(matches!(follower.step(0.2),PlannerStep::Command(c) if c.linear.x>0.0));
        follower.stop();
        assert!(follower.local_failures().is_empty());
    }
    #[test]
    fn owner_reference_materialization_matches_legacy_clipping_exactly() {
        for turn in [false,true] {
            let mut follower=open_follower();
            let start=comparison_pose(0.5,0);
            let mut middle=comparison_pose(1.5,0);
            if turn {middle.pose.position.y=1.3;}
            let end=comparison_pose(3.0,0);
            let goal=comparison_pose(3.2,0);
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![start.clone(),middle.clone(),middle,end]}),None,goal).unwrap();
            for progress in [0.0,0.1,0.98,1.01,2.1] {for horizon in [0.2,1.0,5.0] {
                follower.progress=progress;
                let source=ReferenceSource{route:follower.route.as_ref().unwrap().clone(),cumulative:follower.cumulative.clone(),
                    goal:follower.goal.clone(),options:follower.options,offset:follower.config.body_to_support_z_m,progress};
                assert!(Arc::ptr_eq(&source.route,follower.route.as_ref().unwrap()));
                assert!(Arc::ptr_eq(&source.cumulative,&follower.cumulative));
                let legacy=follower.reference(horizon).unwrap();
                let owner=source.materialize(follower.pose.as_ref().unwrap(),horizon).unwrap();
                assert!(exact_points(&legacy.0,&owner.0));
                assert_eq!((legacy.1.to_bits(),legacy.2,legacy.3),(owner.1.to_bits(),owner.2,owner.3));
            }}
        }
    }

}
