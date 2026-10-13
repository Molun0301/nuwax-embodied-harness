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
        let reference_source=ReferenceSource{route:route.clone(),cumulative:self.cumulative.clone(),
            goal:self.goal.clone(),options:self.options,offset:self.config.body_to_support_z_m,progress:self.progress};
        let request=PlanRequest{source:map.clone(),pose:pose.clone(),boundary:self.boundary.clone(),reference:Vec::new(),
            reference_source:Some(reference_source),final_yaw:None,arrival:None,end:0.0,progress:self.progress,
            generation:self.route_generation,straight:false,detour,scan:self.scan.clone()};
        if let Some(retired)=self.owner.as_ref().unwrap().submit_with_retired(id,request) {
            self.next_request=id;self.submitted=Some((id,pose.pose.position,detour));
            for slot in &mut self.source_tickets {
                if slot.as_ref().is_some_and(|(old,_)|Some(*old)==retired) {*slot=None;}
            }
            if map.body_envelope().is_some() {
                let Some(slot)=self.source_tickets.iter_mut().find(|slot|slot.is_none()) else {
                    self.cancel_planning();self.permanent_fault=Some("planning proof slots exhausted".into());return;
                };
                *slot=Some((id,SurfaceJobTicket::new(Some(Arc::clone(map)))));
            }
            self.local_planning_calls+=1;
            self.retry_after=Instant::now()+Duration::from_millis(250);
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
                let Some(goal)=self.goal.clone() else {return;};
                // Candidate geometry is advice only. Current bounded connector admission
                // uses the actual latest pose/evidence; current step still owns motion.
                match self.controller.replace_execution_path(ExecutionPath::Surface(candidate.path.clone()),None,goal) {
                    Ok(())=> {
                        self.local_path=Some(candidate.path);
                        self.admitted_generation=Some(self.route_generation);
                        self.local_failures.clear();self.qualified_failure=None;self.local_progress_at_plan=request.progress;
                        self.local_end=request.end;self.local_start=Some(request.pose.pose.position);
                        self.dirty=false;self.local_plans+=1;
                        tracing::info!(target:"navigation.scan",event="navigation.scan.candidate_admitted",
                            elapsed_ms=candidate.elapsed_ms,request_id=completion.id,"geometry candidate admitted; current guard remains authoritative");
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
        if matches!(result,Some(PlannerStep::ReplanRequired(_))) {self.dirty=true;self.schedule(true);}
        result
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
                SurfaceMapUpdate {snapshot:surface,change_basis:self.map.clone(),
                    changes:Arc::new(if same {SurfaceGeometryChanges::Empty} else {SurfaceGeometryChanges::Unknown})}
            }
            ExecutionMap::Planar(_) => return Err(error("三维路线执行缺少三维地图")),
        };
        let surface=update.snapshot;
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
            .update_execution_map(ExecutionMap::Surface(Arc::clone(&surface)))?;
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
            // 起点缺少合法支撑或净空时，重发同一全局路线只会造成规划忙循环。
            // 原实际速度回执、输入期限和取消仍由执行调度器处理。
            self.wait_reason = Some(format!(
                "当前机体支撑或净空尚不可用，保持零速等待地图或位姿恢复：{reason}"
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
        self.consume_plan();
        if self.owner.as_ref().is_some_and(|owner|owner.faulted()) {
            return PlannerStep::Failed("local planning owner terminated unexpectedly".into());
        }
        let window=(self.local_end-self.local_progress_at_plan).max(0.1);
        let moved=self.pose.as_ref().zip(self.local_start).is_some_and(|(pose,start)|
            pose.pose.position.distance(start)>=self.scan.refresh_distance_m.min(window*0.6));
        let at_end=self.cumulative.last().is_some_and(|&end|self.local_end>=end-1e-6);
        let near_end=!at_end && self.local_end-self.progress<self.scan.refresh_distance_m.min(window*0.35);
        if self.dirty || moved && !at_end || near_end {self.schedule(false);}
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
        if matches!(step,PlannerStep::ReplanRequired(_)) {self.dirty=true;self.schedule(true);}
        // Preserve a genuine current rejection verbatim while detour work is pending.
        if matches!(step,PlannerStep::ReplanRequired(_) | PlannerStep::Failed(_)) {return step;}
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

#[cfg(test)]
mod tests {
    use super::*;
    include!("route_selective_refresh_tests.rs");
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
            PlanResult { request, outcome: Ok(Candidate { path, elapsed_ms: 0.0 }) }
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
    fn both_modes_reject_changed_support_but_accept_old_ground() {
        for enabled in [true,false] {
            let mut follower = RouteFollower::new(comparison_config(Some(enabled))).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
            comparison_route(&mut follower,comparison_pose(0.95,0),comparison_pose(3.0,0));
            assert!(matches!(ready_step(&mut follower, 0.3),PlannerStep::Command(c) if c.linear.x > 0.0));
            follower.update_execution_map(ExecutionMap::Surface(comparison_map(true,None))).unwrap();
            follower.update_odometry(comparison_pose(0.95,1));
            assert!(!matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x > 0.0),"changed support allowed motion, mode={enabled}");
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
            assert!(follower.execution_wait_reason().unwrap().contains("支撑或净空"));
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
    #[test]
    fn disconnected_forward_window_defers_to_fresh_controller_without_inventing_obstruction() {
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
        let previous_local = follower.local_plans();
        let missing = Arc::new(
            crate::surface_follower::test_snapshot(0.0, true)
                .with_body_envelope(envelope)
                .unwrap(),
        );
        follower
            .update_execution_map(ExecutionMap::Surface(missing))
            .unwrap();
        let mut moved = make(0.7);
        moved.stamp = Timestamp::from_unix(2, 0);
        follower.update_odometry(moved);
        let outcome = follower.step(0.2);
        assert!(matches!(outcome, PlannerStep::Command(_) | PlannerStep::ReplanRequired(_)),
            "a current admitted route must use a fresh controller outcome: {outcome:?}");
        wait_for(|| {
            follower.step(0.2);
            !follower.local_failures().is_empty()
        });
        assert!(follower.local_failures().iter().all(|failure| failure.blocked.is_none()));
        assert!(follower.planner.local_costs().is_empty(), "search failure must not penalize PCT");
        assert_eq!(follower.local_plans(), previous_local);
        assert!(follower.local_path().is_some(), "search failure keeps the last display trajectory");
        follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();
        wait_for_plan(&mut follower, previous_local);
        assert_eq!(follower.local_plans(), previous_local + 1);
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
