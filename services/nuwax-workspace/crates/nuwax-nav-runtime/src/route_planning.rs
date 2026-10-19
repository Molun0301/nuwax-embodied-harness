//! Planning-only state. This module has no controller or motion port.
#[cfg(test)]
#[path="route_priority_probe.rs"]
pub(crate) mod priority_probe;
#[path="recent_guide_overlay.rs"]
mod recent_guide_overlay;
use crate::planning_owner::Cancellation;
use crate::{SurfaceFollowerConfig, SurfaceSnapshot};
use crate::surface_changes::{pct_surface_delta, SurfaceGeometryChanges};
use nuwax_nav_scan::{BodyPose, LocalPlanError, ScanConfig};
use nuwax_nav_mls_planner::surface::BodyGroundState;
use nuwax_nav_types_messages::{GoalOptions, Path3D, Point3, PoseStamped, Quaternion};
use std::borrow::Cow;
use std::sync::Arc;

/// Immutable global reference roots. Clipping/materialization runs only on the
/// planning owner; dense input memory scales with the supplied route, not a new cap.
pub(crate) struct ReferenceSource {
    pub route: Arc<Path3D>,
    pub cumulative: Arc<Vec<f64>>,
    pub goal: Option<PoseStamped>,
    pub options: GoalOptions,
    pub offset: f64,
    pub progress: f64,
}
impl ReferenceSource {
    /// 进度沿原全局窗口累计；实际当前位置的接头只供轨迹生成。
    fn nominal_reference(&self, materialized:&[[f64;3]]) -> Vec<[f64;3]> {
        let mut reference=materialized.to_vec();
        let i=self.cumulative.partition_point(|&value|value<self.progress).saturating_sub(1);
        if let (Some(first),Some(a))=(reference.first_mut(),self.route.poses.get(i).map(|p|p.pose.position)) {
            let p=if let Some(b)=self.route.poses.get(i+1).map(|p|p.pose.position) {
                let length=self.cumulative[i+1]-self.cumulative[i];
                let t=((self.progress-self.cumulative[i])/length.max(1e-12)).clamp(0.0,1.0);
                Point3::new(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t)
            }else{a};
            *first=[p.x,p.y,p.z-self.offset];
        }
        reference
    }
    pub fn materialize(
        &self,
        current: &PoseStamped,
        horizon: f64,
    ) -> Result<(Vec<[f64; 3]>, f64, bool, bool), String> {
        let route = &self.route;
        let offset = self.offset;
        let mut result = vec![[
            current.pose.position.x,
            current.pose.position.y,
            current.pose.position.z - offset,
        ]];
        let end = (self.progress + horizon).min(*self.cumulative.last().ok_or("全局路线为空")?);
        let mut direction: Option<[f64; 2]> = None;
        let mut global_straight = true;
        for (i, pair) in route.poses.windows(2).enumerate().skip(
            self.cumulative
                .partition_point(|&v| v < self.progress)
                .saturating_sub(1),
        ) {
            if nuwax_nav_pct::planning_cancelled() {return Err("reference planning superseded".into());}
            if self.cumulative[i + 1] <= self.progress + 0.02 {
                continue;
            }
            let p = pair[1].pose.position;
            // 路段形状由原全局路线决定，实际机体的横向偏差不能把直线变成弯道。
            let delta = [p.x - pair[0].pose.position.x, p.y - pair[0].pose.position.y];
            let length = delta[0].hypot(delta[1]);
            if length > 1e-9 {
                let next = delta.map(|v| v / length);
                if let Some(first) = direction {
                    global_straight &= first[0] * next[0] + first[1] * next[1] > 0.0
                        && (first[0] * next[1] - first[1] * next[0]).abs() < 1e-9;
                } else {
                    direction = Some(next);
                }
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
}
/// Frozen, explicit input for one internal Ground window. The incoming address
/// is actual consumed navigation; the intent and XY guide are planned inputs.
/// None on PlanRequest preserves the existing Body execution path.
#[derive(Clone)]
pub(crate) struct GroundPlanInput {
    pub incoming: nuwax_nav_pct::ground_route::OwnedGroundWalkState,
    pub intent: crate::replan_feedback::GroundIntent,
    pub xy_reference: Arc<Vec<[f64;2]>>,
    pub arrival_tolerance_m: Option<f64>,
}
impl GroundPlanInput {
    /// Context-only capture never pairs new odometry with an unconsumed basis.
    /// Missing proof means capture is incomplete, not that motion is forbidden.
    pub(crate) fn from_actual(actual:&crate::replan_feedback::ActualExecutionContext,
        intent:crate::replan_feedback::GroundIntent,xy_reference:Arc<Vec<[f64;2]>>,
        arrival_tolerance_m:Option<f64>)->Option<Self> {
        Some(Self {incoming:actual.navigation.as_ref()?.continuation().ok()?,intent,
            xy_reference,arrival_tolerance_m})
    }
    pub fn same_input(&self, other:&Self)->bool {
        self.incoming.semantic_key()==other.incoming.semantic_key()
            && self.intent.same_intent(&other.intent)
            && exact_points(&self.xy_reference,&other.xy_reference)
            && self.arrival_tolerance_m.map(f64::to_bits)==other.arrival_tolerance_m.map(f64::to_bits)
    }
    fn arrival(&self)->Option<([f64;2],f64)> {
        self.arrival_tolerance_m.filter(|_|self.intent.window().to().is_terminal())
            .map(|tolerance|(self.intent.goal().xy,tolerance))
    }

}

pub(crate) struct PlanRequest {
    pub ground: Option<GroundPlanInput>,
    pub start_ground: Result<BodyGroundState, String>,
    pub source: Arc<SurfaceSnapshot>,
    pub change_basis: Option<Arc<SurfaceSnapshot>>,
    pub changes: Arc<SurfaceGeometryChanges>,
    pub pose: PoseStamped,
    pub boundary: Arc<Vec<[f64; 2]>>,
    pub reference: Vec<[f64; 3]>,
    pub reference_source: Option<ReferenceSource>,
    pub final_yaw: Option<f64>,
    pub arrival: Option<([f64; 3], f64)>,
    pub end: f64,
    pub progress: f64,
    pub generation: u64,
    pub straight: bool,
    pub detour: bool,
    pub scan: ScanConfig,
}
impl PlanRequest {
    pub fn materialize_reference(&mut self) -> Result<(), LocalPlanError> {
        if self.ground.is_some() {return Ok(());}
        // 保留不可变原路线，供有限候选比较真实覆盖进度。
        if let Some(source) = self.reference_source.as_ref() {
            let (reference, end, at_end, straight) =
                source.materialize(&self.pose, self.scan.horizon_m)?;
            self.reference = reference;
            self.end = end;
            self.straight = straight;
            self.final_yaw = if at_end && source.options.requires_orientation() {
                source.goal.as_ref().map(|g| g.pose.orientation.yaw())
            } else {
                None
            };
            self.arrival = if at_end {
                source
                    .goal
                    .as_ref()
                    .map(|g| (body(g).0, source.options.position_tolerance_m))
            } else {
                None
            };
        }
        Ok(())
    }
}
pub(crate) struct Candidate {
    pub ground_binding: Option<Arc<nuwax_nav_scan::GroundCandidateBinding>>,
    pub partial: bool,
    pub path: Path3D,
    pub elapsed_ms: f64,
}
fn candidate_from_trajectory(pose: &PoseStamped, trajectory: nuwax_nav_scan::LocalTrajectory) -> Candidate {
    let path = Path3D {
        poses: trajectory
            .poses
            .into_iter()
            .map(|(p, q)| {
                let mut pose = pose.clone();
                pose.pose.position = Point3::new(p[0], p[1], p[2]);
                pose.pose.orientation = Quaternion {
                    x: q[0],
                    y: q[1],
                    z: q[2],
                    w: q[3],
                };
                pose
            })
            .collect(),
    };
    Candidate { ground_binding:None, partial: trajectory.partial, path, elapsed_ms: trajectory.elapsed_ms }
}
fn candidate_from_mapped_trajectory(pose:&PoseStamped,mapped:nuwax_nav_scan::MappedLocalTrajectory)->Candidate {
    let mut candidate=candidate_from_trajectory(pose,mapped.trajectory);
    candidate.ground_binding=Some(Arc::new(mapped.binding));
    candidate
}
fn ground_plan_error(error:nuwax_nav_scan::GroundCandidateError)->LocalPlanError {
    match error {
        nuwax_nav_scan::GroundCandidateError::Planning(error)
        | nuwax_nav_scan::GroundCandidateError::Cancelled {error,..}=>error,
        nuwax_nav_scan::GroundCandidateError::MappingInput(reason)=>reason.into(),
        nuwax_nav_scan::GroundCandidateError::Mapping {reason,..}=>
            format!("selected Ground candidate mapping failed: {reason}").into(),
    }
}
pub(crate) struct PlanResult {
    pub request: PlanRequest,
    pub outcome: Result<Candidate, Vec<LocalPlanError>>,
}
struct FailedLocalPlan {
    ground: Option<GroundPlanInput>,
    start_ground: BodyGroundState,
    coverage_context: [u64;2],
    scan_context: ([u64;2], [usize;2]),
    prefix_context: Option<(Option<[u64;3]>, u64)>,
    costs: Arc<Vec<nuwax_nav_pct::RoutePenalty>>,
    source: Arc<SurfaceSnapshot>,
    tomogram: Arc<nuwax_nav_pct::Tomogram>,
    boundary: Arc<Vec<[f64; 2]>>,
    start: BodyPose,
    // Input to this branch, before PCT/recent guidance. Only one reference copy.
    reference: Vec<[f64; 3]>,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    failure: LocalPlanError,
    detour: bool,
    straight: bool,
}
pub(crate) fn body(p: &PoseStamped) -> BodyPose {
    let q = p.pose.orientation;
    (
        [p.pose.position.x, p.pose.position.y, p.pose.position.z],
        [q.x, q.y, q.z, q.w],
    )
}
pub(crate) fn exact_points<const N: usize>(a: &[[f64; N]], b: &[[f64; N]]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(a, b)| a.map(f64::to_bits) == b.map(f64::to_bits))
}
pub(crate) fn pose_bits(p: BodyPose) -> ([u64; 3], [u64; 4]) {
    (p.0.map(f64::to_bits), p.1.map(f64::to_bits))
}
// These are exact values, not hashes. refresh_distance_m only schedules requests.
fn scan_context(scan: &ScanConfig) -> ([u64;2], [usize;2]) {
    ([scan.horizon_m.to_bits(), scan.reference_corridor_m.to_bits()],
        [scan.optimization_iterations, scan.search_expansion_limit])
}
pub(crate) fn exact_costs(a: &[nuwax_nav_pct::RoutePenalty], b: &[nuwax_nav_pct::RoutePenalty]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a,b)|
        a.center.map(f64::to_bits) == b.center.map(f64::to_bits)
        && a.radius.to_bits() == b.radius.to_bits() && a.weight.to_bits() == b.weight.to_bits())
}
const LOCAL_MAP_REUSE_DISTANCE_M: f64 = 0.25;
fn local_map_radius(horizon: f64, source: &SurfaceSnapshot) -> f64 {
    let (resolution, _) = source.navigation_source_columns(None);
    let halo = source.body_envelope().map_or(f64::from(source.config().wall_clearance_m), |body| body.radius())
        + resolution * 0.5;
    // Reuse may shift the search center; keep the entire search disk and physical
    // query halo inside the materialized source after that shift.
    (horizon + 1.5).max(horizon * 1.5 + LOCAL_MAP_REUSE_DISTANCE_M + halo)
}
pub(crate) struct PlanKernel {
    config: SurfaceFollowerConfig,
    local_map: Option<(Arc<SurfaceSnapshot>, Point3, Arc<nuwax_nav_pct::Tomogram>, f64)>,
    failed: Vec<FailedLocalPlan>,
    epoch: Option<u64>,
    #[cfg(test)]
    pub hook: Option<Box<dyn FnMut(&PlanRequest) + Send>>,
    // Synthetic branch outcomes for selection/cache/cancellation unit tests only.
    #[cfg(test)]
    scan_hook: Option<Box<dyn FnMut(bool, bool) -> Result<Option<nuwax_nav_scan::LocalTrajectory>, LocalPlanError> + Send>>,
}
impl PlanKernel {
    pub fn new(config: SurfaceFollowerConfig) -> Self {
        Self {
            config,
            local_map: None,
            failed: Vec::new(),
            epoch: None,
            #[cfg(test)]
            hook: None,
            #[cfg(test)]
            scan_hook: None,
        }
    }
    pub fn compute(&mut self, mut request: PlanRequest, cancel: &Cancellation) -> PlanResult {
        #[cfg(test)]
        if let Some(hook) = &mut self.hook {
            hook(&request);
        }
        if self.epoch != Some(cancel.expected) {
            self.failed.clear();
            self.epoch = Some(cancel.expected);
        }
        let token = nuwax_nav_pct::PlanningCancellation::new(cancel.epoch.clone(), cancel.expected);
        let outcome = nuwax_nav_pct::with_planning_cancellation(&token, || {
            if cancel.cancelled() {return Err(vec!["planning superseded".into()]);}
            #[cfg(test)] let reference_phase = priority_probe::kernel_phase("request_materialization");
            request.scan.reference_corridor_m = self.config.max_path_deviation_m;
            request.scan.horizon_m = request
                .scan
                .horizon_m
                .max(
                    request
                        .source
                        .body_envelope()
                        .map_or(0.0, |b| b.radius() * 2.0),
                )
                .max(
                    self.config
                        .kinematics()
                        .map_or(0.0, |k| k.minimum_turning_radius_m * 2.0),
                )
                .max(self.config.lookahead_m * 2.0);
            request.materialize_reference().map_err(|e| vec![e])?;
            #[cfg(test)] drop(reference_phase);
            self.compute_inner(&request, cancel)
        });
        PlanResult { request, outcome }
    }
    fn progress_minimum_distance(&self, request:&PlanRequest) -> f64 {
        let tolerance=request.reference_source.as_ref().map_or_else(
            ||request.arrival.map_or(self.config.controller.goal_tolerance,|(_,tolerance)|tolerance),
            |source|source.options.position_tolerance_m);
        tolerance.max(request.source.body_envelope().map_or(0.0,|model|model.radius()))
            +f64::from(request.source.source_voxel_size_m())*0.5
    }
    /// 保留现有近场规划域；完整 publication 仅决定旧图如何更新。
    /// A 当前对局部图执行原窗口冷建，不能把该接口调用报告为局部增量完成。
    fn prepare_local_map(&mut self,r:&PlanRequest,map:&SurfaceSnapshot)
        ->Result<Arc<nuwax_nav_pct::Tomogram>,Vec<LocalPlanError>> {
        if nuwax_nav_pct::planning_cancelled() {return Err(vec!["planning superseded".into()]);}
        let radius=local_map_radius(r.scan.horizon_m,map);
        let prepared=if let Some((source,center,local,_))=self.local_map.as_ref().filter(|(_,center,_,old_radius)|
            center.distance(r.pose.pose.position)<LOCAL_MAP_REUSE_DISTANCE_M && old_radius.to_bits()==radius.to_bits()) {
            if nuwax_nav_pct::same_source_geometry(source,&r.source) {
                Ok((Arc::clone(local),*center))
            } else {
                let delta=pct_surface_delta(r.change_basis.as_deref(),&r.changes);
                local.updated_from_surface(map,delta).map(|(next,_)|(Arc::new(next),*center))
            }
        } else {
            nuwax_nav_pct::Tomogram::local_from_surface(map,
                [r.pose.pose.position.x,r.pose.pose.position.y],radius)
                .map(|next|(Arc::new(next),r.pose.pose.position))
        };
        let (tomogram,center)=prepared.map_err(|error|vec![LocalPlanError::from(error)])?;
        if nuwax_nav_pct::planning_cancelled() {return Err(vec!["planning superseded".into()]);}
        self.local_map=Some((Arc::clone(&r.source),center,Arc::clone(&tomogram),radius));
        Ok(tomogram)
    }
    /// The same owner, cancellation and bounded failure slots accept an explicit
    /// finite Ground window. Its XY guide is already supplied by that window's
    /// caller; Body's support-Z PCT/recent-guide adapter does not synthesize one.
    /// The local Tomogram is only the existing Scan soft-scoring input.
    fn compute_ground(
        &mut self,r:&PlanRequest,input:&GroundPlanInput,cancel:&Cancellation,
    )->Result<Candidate,Vec<LocalPlanError>> {
        let start_ground=r.start_ground.as_ref().map_err(|error|vec![LocalPlanError::from(error.clone())])?;
        let fork=input.incoming.planning_fork(&r.source)
            .map_err(|error|vec![LocalPlanError::from(error.to_string())])?;
        let tomogram=self.prepare_local_map(r,fork.snapshot())?;
        let kinematics=self.config.kinematics().ok_or_else(||vec![LocalPlanError::from("近场执行缺少运动能力")])?;
        let costs=Arc::new(self.config.pct.local_costs());
        let nominal_reference=self.config.motion_limits.as_ref().map(|_| {
            let mut reference=input.xy_reference.as_ref().clone();
            if let Some(first)=reference.first_mut() {*first=input.intent.window().from().walk_state().xy();}
            reference
        });
        let progress_policy=nominal_reference.as_deref().map(|reference|nuwax_nav_scan::GroundProgressPrefixPolicy {
            reference,
            minimum_distance_m:input.arrival_tolerance_m.unwrap_or(self.config.controller.goal_tolerance)
                .max(r.source.body_envelope().map_or(0.0,|model|model.radius()))
                +f64::from(r.source.source_voxel_size_m())*0.5,
        });
        let start=body(&r.pose);
        let mut failures=Vec::new();
        let mut nominal_partial=None;
        let superseded=||LocalPlanError {body_witness:None,
            kind:nuwax_nav_scan::LocalPlanFailureKind::Cancelled,closest_pose:None,blocked:None,
            reason:"planning superseded".into()};
        for detour in if r.detour {&[true][..]}else{&[false,true][..]} {
            if cancel.cancelled() {return Err(vec![superseded()]);}
            let cached=self.failed.iter().find(|f| {
                f.ground.as_ref().is_some_and(|previous|previous.same_input(input))
                    && f.start_ground==*start_ground
                    && Arc::ptr_eq(&f.tomogram,&tomogram)
                    && f.coverage_context==[r.progress.to_bits(),r.end.to_bits()]
                    && f.scan_context==scan_context(&r.scan)
                    // Ground's original window, full XY guide and explicit
                    // tolerance are compared above, without a fake 3D prefix.
                    && f.prefix_context.is_none()
                    && exact_costs(&f.costs,&costs)
                    && f.source.same_body_query_view(&r.source)
                    && exact_points(&f.boundary,&r.boundary)
                    && pose_bits(f.start)==pose_bits(start)
                    && exact_points(&f.reference,&r.reference)
                    && f.final_yaw.map(f64::to_bits)==r.final_yaw.map(f64::to_bits)
                    && f.arrival.map(|(p,t)|(p.map(f64::to_bits),t.to_bits()))
                        ==r.arrival.map(|(p,t)|(p.map(f64::to_bits),t.to_bits()))
                    && f.detour==*detour && f.straight==r.straight
            });
            if let Some(f)=cached {failures.push(f.failure.clone());continue;}
            let follow_reference=*detour;
            let straight=r.straight && !follow_reference;
            let reference_only=*detour && nominal_partial.is_some();
            // Both extents enter the original plan_impl with the exact paired
            // fork/query. The old synthetic Body scan_hook is never consulted.
            let result=if reference_only {
                nuwax_nav_scan::probe_ground_reference_candidates_on_fork(&fork,&tomogram,start,start_ground,
                    input.incoming.walk_state(),&input.xy_reference,&kinematics,&r.scan,r.final_yaw,
                    input.arrival(),&r.boundary,follow_reference,straight,progress_policy,input.intent.window())
                    .map(|outcome|match outcome {
                        nuwax_nav_scan::GroundReferenceCandidateOutcome::Complete(path)=>Some(path),
                        nuwax_nav_scan::GroundReferenceCandidateOutcome::Miss=>None,
                    })
            } else {
                nuwax_nav_scan::plan_driven_ground_on_fork(&fork,&tomogram,start,start_ground,
                    input.incoming.walk_state(),&input.xy_reference,&kinematics,&r.scan,r.final_yaw,
                    input.arrival(),&r.boundary,follow_reference,straight,progress_policy,input.intent.window()).map(Some)
            };
            let result=match result {
                Ok(Some(path))=>Ok(path),
                Ok(None)=>continue,
                Err(nuwax_nav_scan::GroundCandidateError::Planning(error))=>Err(error),
                Err(error)=> {
                    // A selected physical winner whose original binding failed
                    // is not a search miss. Do not select a second winner or
                    // remember the mapping refusal as physical infeasibility.
                    failures.push(ground_plan_error(error));
                    return Err(failures);
                }
            };
            if let Err(failure)=&result {
                if !reference_only && failure.kind!=nuwax_nav_scan::LocalPlanFailureKind::Cancelled {
                    if self.failed.len()==2 {self.failed.remove(0);}
                    self.failed.push(FailedLocalPlan {
                        ground:Some(input.clone()),start_ground:start_ground.clone(),
                        coverage_context:[r.progress.to_bits(),r.end.to_bits()],scan_context:scan_context(&r.scan),
                        prefix_context:None,costs:Arc::clone(&costs),source:r.source.clone(),tomogram:tomogram.clone(),
                        boundary:r.boundary.clone(),start,reference:r.reference.clone(),final_yaw:r.final_yaw,
                        arrival:r.arrival,failure:failure.clone(),detour:*detour,straight:r.straight,
                    });
                }
            }
            match result {
                Ok(path) if !*detour && path.trajectory.partial=>nominal_partial=Some(path),
                Ok(path)=> {
                    if nominal_partial.is_some() && cancel.cancelled() {return Err(vec![superseded()]);}
                    let selected=if path.trajectory.partial {nominal_partial.take().unwrap_or(path)}else{path};
                    return Ok(candidate_from_mapped_trajectory(&r.pose,selected));
                }
                Err(failure)=> {
                    let cancelled=failure.kind==nuwax_nav_scan::LocalPlanFailureKind::Cancelled;
                    failures.push(failure);if cancelled {return Err(failures);}
                }
            }
        }
        if let Some(path)=nominal_partial {
            if cancel.cancelled() {return Err(vec![superseded()]);}
            Ok(candidate_from_mapped_trajectory(&r.pose,path))
        } else {Err(failures)}
    }
    fn compute_inner(
        &mut self,
        r: &PlanRequest,
        cancel: &Cancellation,
    ) -> Result<Candidate, Vec<LocalPlanError>> {
        if let Some(ground)=r.ground.as_ref() {return self.compute_ground(r,ground,cancel);}
        #[cfg(test)] let map_phase = priority_probe::kernel_phase("map_prepare");
        let start_ground=r.start_ground.as_ref().map_err(|error|vec![LocalPlanError::from(error.clone())])?;
        // Fork on this CPU owner. No cache permission travels back to control.
        let map = r.source.fork_for_planning();
        let tomogram=self.prepare_local_map(r,&map)?;
        #[cfg(test)] drop(map_phase);
        #[cfg(test)] let policy_phase = priority_probe::kernel_phase("overlay_and_policy");
        let kinematics = self
            .config
            .kinematics()
            .ok_or_else(|| vec!["近场执行缺少运动能力".into()])?;
        // One existing cost snapshot, shared by the two failure slots; no second lock/read.
        let costs = Arc::new(self.config.pct.local_costs());
        // 首次规划也读取本次近期源；静态层析缓存不包含这些障碍。
        // 范围覆盖完整绕行搜索圆及机体外包，避免只导出原直线附近的回波。
        let recent_guide = if let Some(model) = map.body_envelope() {
            let center = [r.pose.pose.position.x, r.pose.pose.position.y];
            let radius = r.scan.horizon_m * 1.5 + model.radius();
            let bounds = [center.map(|v|v-radius), center.map(|v|v+radius)];
            let hints = map.recent_obstacle_hints(bounds, body(&r.pose))
                .map_err(|e|vec![LocalPlanError::from(e.to_string())])?;
            Some(recent_guide_overlay::RecentGuideOverlay::new(hints, tomogram.resolution, f64::from(map.source_voxel_size_m()), model)
                .map_err(|e|vec![LocalPlanError::from(e)])?)
        } else { None };
        let nominal_reference=self.config.motion_limits.as_ref().map(|_|r.reference_source.as_ref()
            .map_or_else(||r.reference.clone(),|source|source.nominal_reference(&r.reference)));
        let progress_policy=nominal_reference.as_deref().map(|reference|nuwax_nav_scan::ProgressPrefixPolicy {
            reference,
            minimum_distance_m:self.progress_minimum_distance(r),
        });
        // nominal_reference changes only the first point of r.reference. The rest
        // is already compared below, so retaining a second full route is unnecessary.
        let prefix_context = progress_policy.map(|p|
            (p.reference.first().copied().map(|v|v.map(f64::to_bits)), p.minimum_distance_m.to_bits()));
        let start = body(&r.pose);
        #[cfg(test)] drop(policy_phase);
        let mut failures = Vec::new();
        // Keep the original partial by move while detour reference candidates
        // get one chance at a complete trajectory. Full nominal stays first.
        let mut nominal_partial = None;
        let superseded = || LocalPlanError {
            body_witness: None,
            kind: nuwax_nav_scan::LocalPlanFailureKind::Cancelled,
            closest_pose: None,
            blocked: None,
            reason: "planning superseded".into(),
        };
        for detour in if r.detour {
            &[true][..]
        } else {
            &[false, true][..]
        } {
            if cancel.cancelled() {
                return Err(vec![superseded()]);
            }
            let cached = self.failed.iter().find(|f| {
                f.ground.is_none() && f.start_ground == *start_ground
                    && Arc::ptr_eq(&f.tomogram, &tomogram)
                    && f.coverage_context==[r.progress.to_bits(),r.end.to_bits()]
                    && f.scan_context == scan_context(&r.scan)
                    && f.prefix_context == prefix_context
                    && exact_costs(&f.costs, &costs)
                    && f.source.same_body_query_view(&r.source)
                    && exact_points(&f.boundary, &r.boundary)
                    && pose_bits(f.start) == pose_bits(start)
                    && exact_points(&f.reference, &r.reference)
                    && f.final_yaw.map(f64::to_bits) == r.final_yaw.map(f64::to_bits)
                    && f.arrival.map(|(p, t)| (p.map(f64::to_bits), t.to_bits()))
                        == r.arrival.map(|(p, t)| (p.map(f64::to_bits), t.to_bits()))
                    && f.detour == *detour
                    && f.straight == r.straight
            });
            if let Some(f) = cached {
                #[cfg(test)] priority_probe::reused_failure(*detour, f.failure.kind);
                failures.push(f.failure.clone());
                continue;
            }
            let mut reference: Cow<'_, [[f64; 3]]> = Cow::Borrowed(r.reference.as_slice());
            if *detour && reference.len() > 1 {
                #[cfg(test)] let pct_at=priority_probe::clock();
                #[cfg(test)] let mut pct_stats=None;
                if let Ok(route) = tomogram.plan_bounded(
                    reference[0],
                    *reference.last().unwrap(),
                    tomogram.resolution,
                    &r.boundary,
                    None,
                    Some(r.scan.horizon_m * 1.5),
                    &costs,
                ) {
                    #[cfg(test)] {pct_stats=Some(route.stats.clone());}
                    reference = Cow::Owned(route.points);
                }
                #[cfg(test)] priority_probe::pct(pct_at,pct_stats);
            }
            let mut follow_reference = *detour;
            if let Some(overlay) = recent_guide.as_ref().filter(|overlay|
                !overlay.is_empty() && reference.windows(2).any(|p|!overlay.accepts(p[0],p[1]))) {
                #[cfg(test)] let _overlay_phase = priority_probe::kernel_phase("filtered_recent_pct");
                match tomogram.plan_bounded_with_segment_filter(
                    reference[0], *reference.last().unwrap(), tomogram.resolution,
                    &r.boundary, None, Some(r.scan.horizon_m*1.5), &costs,
                    &|a,b|overlay.accepts(a,b),
                ) {
                    Ok(route) => {reference = Cow::Owned(route.points); follow_reference = true;}
                    Err(reason) => {
                        // 外接圆比完整姿态保守，窄门仍交给原机体扫掠判断。
                        // 引导失败不等于真实不可通行；非法提示输入则在上方明确返回。
                        tracing::debug!(target:"navigation.scan",%reason,
                            "recent guide unavailable; retaining exact Scan fallback");
                    }
                }
            }
            let straight = r.straight && !follow_reference;
            let reference_only = *detour && nominal_partial.is_some();
            #[cfg(test)] let branch_at=priority_probe::branch_begin(*detour,follow_reference,straight,
                &r.reference,&reference,false);
            #[cfg(feature="local-guidance-qualification")]
            let qualification=crate::local_guidance_qualification::begin(r);
            let result = {
                let scan = || {
                    if reference_only {
                        nuwax_nav_scan::probe_reference_candidates_from_ground(&map,&tomogram,start,start_ground,&reference,
                            &kinematics,&r.scan,r.final_yaw,r.arrival,&r.boundary,
                            follow_reference,straight,progress_policy).map(|outcome| match outcome {
                                nuwax_nav_scan::ReferenceCandidateOutcome::Complete(path) => Some(path),
                                nuwax_nav_scan::ReferenceCandidateOutcome::Miss => None,
                            })
                    } else {
                        nuwax_nav_scan::plan_with_progress_from_ground(&map,&tomogram,start,start_ground,&reference,
                            &kinematics,&r.scan,r.final_yaw,r.arrival,&r.boundary,
                            follow_reference,straight,progress_policy).map(Some)
                    }
                };
                #[cfg(not(test))]
                let result = scan();
                #[cfg(test)]
                let result = self.scan_hook.as_mut().map_or_else(scan, |hook| hook(*detour, reference_only));
                let result = match result {
                    Ok(Some(path)) => Ok(path),
                    Ok(None) => {
                        // Miss is an untried-search outcome, not a complete failure.
                        #[cfg(feature="local-guidance-qualification")]
                        crate::local_guidance_qualification::finish_reference_miss(qualification);
                        #[cfg(test)] priority_probe::branch_reference_miss(branch_at);
                        continue;
                    }
                    Err(failure) => Err(failure),
                };
                if let Err(failure) = &result {
                    if !reference_only && failure.kind != nuwax_nav_scan::LocalPlanFailureKind::Cancelled {
                        if self.failed.len() == 2 {
                            self.failed.remove(0);
                        }
                        self.failed.push(FailedLocalPlan {
                            ground:None,
                            start_ground:start_ground.clone(),
                            coverage_context:[r.progress.to_bits(),r.end.to_bits()],
                            scan_context: scan_context(&r.scan),
                            prefix_context,
                            costs: Arc::clone(&costs),
                            source: r.source.clone(),
                            tomogram: tomogram.clone(),
                            boundary: r.boundary.clone(),
                            start,
                            reference: r.reference.clone(),
                            final_yaw: r.final_yaw,
                            arrival: r.arrival,
                            failure: failure.clone(),
                            detour: *detour,
                            straight: r.straight,
                        });
                    }
                }
                result
            };
            #[cfg(feature="local-guidance-qualification")]
            crate::local_guidance_qualification::finish(qualification,r,&map,&tomogram,
                &reference,&kinematics,*detour,follow_reference,straight,progress_policy,&result);
            #[cfg(test)] priority_probe::branch_end(branch_at,&result);
            match result {
                Ok(trajectory) if !*detour && trajectory.partial => {
                    nominal_partial = Some(trajectory);
                }
                Ok(trajectory) => {
                    if nominal_partial.is_some() && cancel.cancelled() {
                        return Err(vec![superseded()]);
                    }
                    let selected = if trajectory.partial {
                        nominal_partial.take().unwrap_or(trajectory)
                    } else {
                        trajectory
                    };
                    return Ok(candidate_from_trajectory(&r.pose, selected));
                }
                Err(failure) => {
                    let cancelled = failure.kind == nuwax_nav_scan::LocalPlanFailureKind::Cancelled;
                    failures.push(failure);
                    if cancelled {
                        return Err(failures);
                    }
                }
            }
        }
        if let Some(trajectory) = nominal_partial {
            // The detour can finish with an ordinary error or a cache hit.
            // Neither may publish the saved partial after cancellation.
            if cancel.cancelled() {
                return Err(vec![superseded()]);
            }
            Ok(candidate_from_trajectory(&r.pose, trajectory))
        } else {
            Err(failures)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    include!("route_progress_policy_tests.rs");
    include!("route_repeated_detour_key_tests.rs");
    include!("route_partial_choice_tests.rs");
    include!("route_incremental_tests.rs");
    use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::{MotionKinematics, MotionModel, NavFrameId, Pose3, Timestamp};
    use std::sync::atomic::AtomicU64;

    fn fixture() -> (PlanKernel, PlanRequest, Cancellation) {
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.basic_kinematics = Some(MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        });
        let source = crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(SurfaceBodyEnvelope {
                fixed_collision: None,
                footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap();
        let mut observations = SurfaceObservations::new(source.source_voxel_size_m(), 32).unwrap();
        let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
        for (x, y, z) in source.cell_keys() {
            let key = [x, y, z];
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 1_000_000_000,
                        hits: 2,
                    },
                );
        }
        for (key, block) in blocks {
            observations.replace_block(key, block).unwrap();
        }
        let source = Arc::new(source.with_observations(Some(Arc::new(observations))));
        let pose = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(0.5, 1.0, 0.5),
                orientation: Quaternion::IDENTITY,
            },
        };
        let request = PlanRequest {
            ground:None,
            start_ground:source.body_query().start_ground_state(body(&pose)).map_err(|error|error.to_string()),
            source,
            change_basis: None,
            changes: Arc::new(SurfaceGeometryChanges::Unknown),
            pose,
            boundary: Arc::new(Vec::new()),
            reference: vec![[0.5, 1.0, 0.1], [3.0, 1.0, 0.1]],
            reference_source: None,
            final_yaw: None,
            arrival: Some(([3.0, 1.0, 0.5], 0.35)),
            end: 2.5,
            progress: 0.0,
            generation: 1,
            straight: true,
            detour: false,
            scan: config.scan.clone(),
        };
        let cancel = Cancellation {
            epoch: Arc::new(AtomicU64::new(1)),
            expected: 1,
        };
        (PlanKernel::new(config), request, cancel)
    }
    #[test]
    fn local_materialization_covers_detours_after_cached_center_drift() {
        let (_, request, _) = fixture();
        let horizon = 5.0;
        let halo = request.source.body_envelope().unwrap().radius();
        let radius = local_map_radius(horizon, &request.source);
        assert!(radius > horizon + 1.5, "the old crop was smaller than the detour disk");
        for drift in [0.0, 0.1, LOCAL_MAP_REUSE_DISTANCE_M - 1e-9] {
            assert!(drift + horizon * 1.5 + halo <= radius);
        }
    }
    fn seed_exact_failures(kernel: &mut PlanKernel, request: &PlanRequest) {
        let tomogram = Arc::new(
            nuwax_nav_pct::Tomogram::local_from_surface(
                &request.source,
                [request.pose.pose.position.x, request.pose.pose.position.y],
                local_map_radius(request.scan.horizon_m,&request.source),
            )
            .unwrap(),
        );
        kernel.local_map = Some((
            request.source.clone(),
            request.pose.pose.position,
            tomogram.clone(),
            local_map_radius(request.scan.horizon_m,&request.source),
        ));
        kernel.failed.clear();
        for detour in [false, true] {
            kernel.failed.push(FailedLocalPlan {
                ground:None,
                start_ground:request.start_ground.as_ref().unwrap().clone(),
                coverage_context:[request.progress.to_bits(),request.end.to_bits()],
                scan_context: scan_context(&request.scan),
                prefix_context: kernel.config.motion_limits.as_ref().map(|_| {
                    let nominal=request.reference_source.as_ref().map_or_else(
                        ||request.reference.clone(),|source|source.nominal_reference(&request.reference));
                    (nominal.first().copied().map(|v|v.map(f64::to_bits)), kernel.progress_minimum_distance(request).to_bits())
                }),
                costs: Arc::new(kernel.config.pct.local_costs()),
                source: request.source.clone(),
                tomogram: tomogram.clone(),
                boundary: request.boundary.clone(),
                start: body(&request.pose),
                reference: request.reference.clone(),
                final_yaw: request.final_yaw,
                arrival: request.arrival,
                failure: LocalPlanError { body_witness: None,
                    kind: nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted,
                    closest_pose: Some(body(&request.pose)),
                    blocked: None,
                    reason: "exact cached failure".into(),
                },
                detour,
                straight: request.straight,
            });
        }
    }
    #[test]
    fn first_plan_routes_around_recent_wall_before_exhausting_scan() {
        let (mut kernel, mut request, cancel) = fixture();
        request.source = Arc::new(request.source.with_recent_obstacle_points(
            1, 1_000_000_000, 0, &[],
            (0..5).map(|i| (i, [0.73, 0.8 + f64::from(i) * 0.1, 0.5])),
        ).unwrap());
        request.final_yaw = Some(0.0);
        let began = std::time::Instant::now();
        let result = kernel.compute_inner(&request, &cancel).unwrap();
        let elapsed_ms = began.elapsed().as_secs_f64() * 1000.0;
        let length_m: f64 = result.path.poses.windows(2)
            .map(|p|p[0].pose.position.distance(p[1].pose.position)).sum();
        let turns: Vec<_> = result.path.poses.windows(2).map(|p|
            nuwax_nav_astar::controllers::angle_diff(p[1].pose.orientation.yaw(),p[0].pose.orientation.yaw()).abs()).collect();
        eprintln!("FIRST_RECENT_PLAN elapsed_ms={elapsed_ms} scan_ms={} failed_attempts={} poses={} length_m={length_m} turn_rad={} max_turn_rad={}",
            result.elapsed_ms, kernel.failed.len(), result.path.poses.len(),
            turns.iter().sum::<f64>(), turns.iter().copied().fold(0.0,f64::max));
        assert!(kernel.failed.is_empty(), "首次规划须先避开近期墙，不能消耗一次失败搜索后才绕行");
        assert!(result.path.poses.iter().any(|p| (p.pose.position.y - 1.0).abs() > 0.1));
        let end = result.path.poses.last().unwrap();
        assert!(end.pose.position.distance(Point3::new(3.0, 1.0, 0.5)) < 0.35);
        for pair in result.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]), body(&pair[1]), &[]).unwrap();
        }
    }
    #[test]
    fn recent_detour_overlay_reuses_static_tomogram_and_clears_without_rebuild() {
        let (mut kernel,mut request,cancel)=fixture();
        kernel.compute_inner(&request,&cancel).unwrap_or_else(|e|panic!("baseline: {e:?}"));
        let cached=kernel.local_map.as_ref().unwrap().2.clone();
        let original=request.source.clone();
        request.source=Arc::new(original.with_recent_obstacle_points(1,1_000_000_000,0,&[],
            (0..5).map(|i|(i,[0.73,0.8+f64::from(i)*0.1,0.5]))).unwrap());
        request.detour=true;request.final_yaw=Some(0.0);
        let original_reference=request.reference.clone();
        let original_boundary=request.boundary.clone();
        let original_horizon=request.scan.horizon_m.to_bits();
        let original_costs=kernel.config.pct.local_costs();
        priority_probe::begin();
        let detour_result=kernel.compute_inner(&request,&cancel);
        let detour_work=priority_probe::end();
        let detour=detour_result.unwrap_or_else(|e|panic!("detour: {e:?}"));
        assert!(Arc::ptr_eq(&cached,&kernel.local_map.as_ref().unwrap().2));
        assert!(nuwax_nav_pct::same_source_geometry(&original,&request.source));
        assert!(detour.path.poses.iter().any(|p|(p.pose.position.y-1.0).abs()>0.1));
        for pair in detour.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]),body(&pair[1]),&[]).unwrap();
        }
        request.source=Arc::new(request.source.with_recent_obstacle_points(2,2_000_000_000,0,&[],std::iter::empty()).unwrap());
        assert!(exact_points(&request.reference,&original_reference));
        assert!(exact_points(&request.boundary,&original_boundary));
        assert_eq!(request.scan.horizon_m.to_bits(),original_horizon);
        assert!(exact_costs(&kernel.config.pct.local_costs(),&original_costs));
        priority_probe::begin();
        let clear_result=kernel.compute_inner(&request,&cancel);
        let clear_work=priority_probe::end();
        let clear=clear_result.unwrap_or_else(|e|panic!("clear: {e:?}"));
        assert!(Arc::ptr_eq(&cached,&kernel.local_map.as_ref().unwrap().2));
        assert!(clear.path.poses.iter().all(|p|(p.pose.position.y-1.0).abs()<1e-6));
        assert!(exact_points(&request.reference,&original_reference));
        for work in [&detour_work,&clear_work] {
            assert_eq!(work.branches.len(),1);
            assert_eq!(work.pct.len(),1);
            assert!(work.pct[0].stats.is_some(),"actual unfiltered PCT must succeed");
            assert!(work.branches[0].detour && work.branches[0].follow_reference && !work.branches[0].straight);
        }
        assert_eq!(detour_work.kernel_phases.iter().filter(|p|p.phase=="filtered_recent_pct").count(),1);
        assert!(!clear_work.kernel_phases.iter().any(|p|p.phase=="filtered_recent_pct"));
        assert!(!exact_points(&detour_work.branches[0].reference,&clear_work.branches[0].reference),
            "same static PCT inputs require the recent filtered success to replace its reference");
        eprintln!("COW_REFERENCE_REPLACEMENTS detour_bits={:?} clear_bits={:?}",
            detour_work.branches[0].reference.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>(),
            clear_work.branches[0].reference.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>());
    }
    #[test]
    fn recent_radius_guide_failure_preserves_pose_feasible_narrow_door() {
        let (mut kernel,mut request,cancel)=fixture();
        let mut model=request.source.body_envelope().unwrap().clone();
        model.footprint_xy_m=vec![[-0.3,-0.1],[0.3,-0.1],[0.3,0.1],[-0.3,0.1]];
        request.source=Arc::new(request.source.with_body_envelope(model).unwrap()
            .with_recent_obstacle_points(1,1_000_000_000,0,&[],
                [(0,[1.25,0.75,0.5]),(1,[1.25,1.25,0.5])]).unwrap());
        request.boundary=Arc::new(vec![[0.0,0.82],[4.0,0.82],[4.0,1.18],[0.0,1.18]]);
        request.detour=true;request.final_yaw=Some(0.0);
        let map=&request.source;
        let start=body(&request.pose);let end=([3.0,1.0,0.5],start.1);
        map.validate_body_motion(start,end,&request.boundary).expect("real long body fits the narrow door");
        let tomo=nuwax_nav_pct::Tomogram::local_from_surface(map,[0.5,1.0],6.5).unwrap();
        let hints=map.recent_obstacle_hints([[-6.5,-6.5],[6.5,6.5]],start).unwrap();
        let overlay=recent_guide_overlay::RecentGuideOverlay::new(hints,tomo.resolution,f64::from(map.source_voxel_size_m()),map.body_envelope().unwrap()).unwrap();
        assert!(!overlay.is_empty());
        assert!(tomo.plan_bounded_with_segment_filter(request.reference[0],*request.reference.last().unwrap(),
            tomo.resolution,&request.boundary,None,Some(7.5),&[],&|a,b|overlay.accepts(a,b)).is_err(),
            "radius-only guide must be demonstrably more conservative than this pose-feasible door");
        let result=kernel.compute_inner(&request,&cancel).unwrap_or_else(|e|panic!("exact fallback: {e:?}"));
        assert!(result.path.poses.iter().all(|p|(p.pose.position.y-1.0).abs()<1e-6));
        for pair in result.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]),body(&pair[1]),&request.boundary).unwrap();
        }
    }
    #[test]
    fn cache_key_preserves_signed_zero_and_quaternion_antipodes() {
        let pose = ([0.0, 1.0, 2.0], [0.0, 0.0, 0.0, 1.0]);
        let mut changed = pose;
        changed.0[0] = -0.0;
        assert_ne!(pose_bits(pose), pose_bits(changed));
        changed = pose;
        changed.1 = changed.1.map(|x| -x);
        assert_ne!(pose_bits(pose), pose_bits(changed));
        assert!(!exact_points(&[[0.0, 1.0]], &[[-0.0, 1.0]]));
    }
    #[test]
    fn exact_failed_attempt_cache_requires_owner_pose_boundary_source_and_semantics() {
        for change in 0..12 {
            let (mut kernel, mut request, cancel) = fixture();
            seed_exact_failures(&mut kernel, &request);
            let failures = match kernel.compute_inner(&request, &cancel) {
                Err(failures) => failures,
                Ok(_) => panic!("identical requests must reuse exact failures"),
            };
            assert_eq!(failures.len(), 2);
            assert!(failures
                .iter()
                .all(|failure| failure.reason == "exact cached failure"));
            match change {
                0 => {
                    let (source, center, _, radius) = kernel.local_map.take().unwrap();
                    let owner = Arc::new(
                        nuwax_nav_pct::Tomogram::local_from_surface(
                            &source,
                            [center.x, center.y],
                            radius,
                        )
                        .unwrap(),
                    );
                    kernel.local_map = Some((source, center, owner, radius));
                }
                1 => request.pose.pose.position.x += 0.01,
                2 => {
                    request.boundary =
                        Arc::new(vec![[0.0, 0.0], [6.0, 0.0], [6.0, 3.0], [0.0, 3.0]])
                }
                3 => {
                    // A metadata-only source has the same Tomogram geometry, but cannot
                    // inherit a failure qualified against a different body query view.
                    let observations =
                        Arc::new(request.source.observations().unwrap().as_ref().clone());
                    let changed = Arc::new(request.source.with_observations(Some(observations)));
                    assert!(nuwax_nav_pct::same_source_geometry(
                        &request.source,
                        &changed
                    ));
                    assert!(!request.source.same_body_query_view(&changed));
                    request.source = changed;
                }
                4 => request.arrival.as_mut().unwrap().1 *= 0.9,
                5 => request.arrival.as_mut().unwrap().0[0] += 0.1,
                6 => request.pose.pose.orientation.x = -0.0,
                7 => request.final_yaw = Some(0.0),
                8 => {
                    // Each failure qualifies the original input, before the optional PCT pass.
                    for cached in &mut kernel.failed {
                        cached.reference[0][0] += 0.01;
                    }
                }
                9 => {
                    // Reversing both flags only permutes the two valid input keys.
                    // A detour-only request cannot inherit the nominal-only failure.
                    kernel.failed.retain(|cached| !cached.detour);
                    request.detour = true;
                }
                10 => {
                    for cached in &mut kernel.failed {
                        cached.straight = !cached.straight;
                    }
                }
                _ => {
                    let source = crate::surface_follower::test_snapshot(0.0, false)
                        .with_body_envelope(request.source.body_envelope().unwrap().clone())
                        .unwrap();
                    request.source = Arc::new(source);
                }
            }
            if let Err(failures) = kernel.compute_inner(&request, &cancel) {
                assert!(
                    failures
                        .iter()
                        .all(|failure| failure.reason != "exact cached failure"),
                    "dependency change {change} reused a stale negative"
                );
            }
            assert!(kernel.failed.len() <= 2);
        }
    }
    #[test]
    fn exact_failed_attempt_cache_isolates_nominal_and_detour_inputs_both_ways() {
        use nuwax_nav_scan::LocalPlanFailureKind;
        for requested_detour in [false, true] {
            let (mut kernel, mut request, cancel) = fixture();
            request.detour = requested_detour;
            seed_exact_failures(&mut kernel, &request);
            priority_probe::begin();
            let same = kernel.compute_inner(&request, &cancel);
            let same_work = priority_probe::end();
            let same_failures = same.err().expect("identical branches must hit their exact failures");
            let expected_branches = if requested_detour { vec![true] } else { vec![false, true] };
            assert_eq!(same_failures.len(), expected_branches.len());
            assert!(same_failures.iter().all(|f|f.reason == "exact cached failure"));
            assert_eq!(same_work.reused_failures, expected_branches.into_iter()
                .map(|d|(d, LocalPlanFailureKind::BudgetExhausted)).collect::<Vec<_>>());
            assert!(same_work.pct.is_empty() && same_work.branches.is_empty());

            // Keep exactly one WRONG branch entry. Flipping both seeded flags
            // would only permute the same valid {nominal, detour} input keys.
            kernel.failed.retain(|f|f.detour != requested_detour);
            assert_eq!(kernel.failed.len(), 1);
            priority_probe::begin();
            let changed = kernel.compute_inner(&request, &cancel);
            let changed_work = priority_probe::end();
            let candidate = changed.unwrap_or_else(|errors|
                panic!("wrong-branch failure replaced actual clear-fixture planning: {errors:?}"));
            assert!(!candidate.partial && !candidate.path.poses.is_empty());
            assert!(changed_work.reused_failures.is_empty(),
                "the opposite branch's input-qualified failure must not be reused");
            assert_eq!(changed_work.branches.iter().map(|b|b.detour).collect::<Vec<_>>(),
                vec![requested_detour], "the requested branch must actually enter Scan");
            assert_eq!(changed_work.pct.len(), usize::from(requested_detour));
        }
    }
    #[test]
    fn lifecycle_epoch_retires_exact_failures() {
        let (mut kernel, request, mut cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        kernel.epoch = Some(cancel.expected);
        cancel.expected += 1;
        cancel
            .epoch
            .store(cancel.expected, std::sync::atomic::Ordering::Release);
        let result = kernel.compute(request, &cancel);
        if let Err(failures) = result.outcome {
            assert!(failures
                .iter()
                .all(|failure| failure.reason != "exact cached failure"));
        }
        assert_eq!(kernel.epoch, Some(cancel.expected));
        assert!(kernel
            .failed
            .iter()
            .all(|failure| failure.failure.reason != "exact cached failure"));
    }
    #[test]
    fn local_tomogram_reuse_keeps_its_original_construction_center() {
        for z_only in [false, true] {
            let (mut kernel, mut request, cancel) = fixture();
            let _ = kernel.compute_inner(&request, &cancel);
            let (_, original_center, original_tomogram, _) =
                kernel.local_map.as_ref().unwrap().clone();
            for distance in [0.1, 0.2] {
                request.pose.pose.position = original_center;
                if z_only {
                    request.pose.pose.position.z += distance;
                } else {
                    request.pose.pose.position.x += distance;
                }
                let _ = kernel.compute_inner(&request, &cancel);
                let (_, center, tomogram, _) = kernel.local_map.as_ref().unwrap();
                assert_eq!(
                    *center, original_center,
                    "cache origin drifted after a sub-threshold move"
                );
                assert!(Arc::ptr_eq(tomogram, &original_tomogram));
            }
            request.pose.pose.position = original_center;
            if z_only {
                request.pose.pose.position.z += 0.3;
            } else {
                request.pose.pose.position.x += 0.3;
            }
            let _ = kernel.compute_inner(&request, &cancel);
            let (_, center, tomogram, _) = kernel.local_map.as_ref().unwrap();
            assert_eq!(*center, request.pose.pose.position);
            assert!(
                !Arc::ptr_eq(tomogram, &original_tomogram),
                "cumulative displacement must reconstruct the local Tomogram"
            );
        }
    }
// Insert inside route_planning.rs's existing #[cfg(test)] mod tests.
// Cache-key unit only: the separate route-follower test proves the production
// request chain. This unit varies only the ground input after common setup.
// Source-only draft; not built or run. No successful Scan result is simulated.
#[test]
fn exact_failed_attempt_cache_isolates_unknown_ground_history() {
    use nuwax_nav_mls_planner::surface::GroundQueryState;
    use std::sync::atomic::{AtomicUsize, Ordering};
    let (mut kernel, mut request, cancel) = fixture();
    request.pose.pose.position.x = 6.65;
    let start = body(&request.pose);
    let mut known_pose = start;
    known_pose.0[0] = 6.45;
    request.reference = vec![
        [start.0[0], start.0[1], start.0[2] - kernel.config.body_to_support_z_m],
        [known_pose.0[0], known_pose.0[1], known_pose.0[2] - kernel.config.body_to_support_z_m],
    ];
    request.arrival = Some((known_pose.0, 0.01));
    request.end = (start.0[0] - known_pose.0[0]).abs();
    let query = request.source.body_query();
    let known = query.start_ground_state(known_pose).unwrap();
    let fresh = query.start_ground_state(start).unwrap();
    let carried = query.observe_actual_ground(known_pose, &known, start).unwrap();
    assert_ne!(known.query_state(), GroundQueryState::Unknown);
    assert_eq!(fresh.query_state(), GroundQueryState::Unknown);
    assert_eq!(carried.query_state(), GroundQueryState::Unknown);
    assert_eq!(carried.height().to_bits(), known.height().to_bits());
    assert_ne!(fresh, carried, "same actual pose has two legal source-local histories");
    request.start_ground = Ok(fresh.clone());

    let calls = Arc::new(AtomicUsize::new(0));
    let hook_calls = calls.clone();
    kernel.scan_hook = Some(Box::new(move |detour, reference_only| {
        assert!(!reference_only);
        let n = hook_calls.fetch_add(1, Ordering::SeqCst) + 1;
        assert_eq!(detour, n % 2 == 0, "misses run nominal then detour");
        Err(LocalPlanError {
            kind: nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted,
            closest_pose: None, blocked: None, body_witness: None,
            reason: format!("ground-cache-unit-{n}"),
        })
    }));
    let first = kernel.compute_inner(&request, &cancel).err().expect("failure-only unit hook");
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    assert_eq!(first.iter().map(|f| f.reason.as_str()).collect::<Vec<_>>(),
        ["ground-cache-unit-1", "ground-cache-unit-2"]);
    assert_eq!(kernel.failed.len(), 2);
    assert!(kernel.failed.iter().all(|f| f.start_ground == fresh));
    let tomogram = kernel.local_map.as_ref().unwrap().2.clone();
    let repeated = kernel.compute_inner(&request, &cancel).err().expect("exact negative reuse");
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    assert_eq!(repeated.iter().map(|f| f.reason.as_str()).collect::<Vec<_>>(),
        ["ground-cache-unit-1", "ground-cache-unit-2"]);

    // Deliberate unit input change, not a substitute production request builder.
    // Source, complete pose, reference, arrival, policy, epoch and every other
    // cache input remain untouched. Preserve the production Result field type.
    request.start_ground = Ok(carried.clone());
    let changed = kernel.compute_inner(&request, &cancel).err().expect("failure-only unit hook");
    assert_eq!(calls.load(Ordering::SeqCst), 4,
        "different ground must not inherit either nominal or detour negative");
    assert_eq!(changed.iter().map(|f| f.reason.as_str()).collect::<Vec<_>>(),
        ["ground-cache-unit-3", "ground-cache-unit-4"]);
    assert!(Arc::ptr_eq(&tomogram, &kernel.local_map.as_ref().unwrap().2),
        "a tomogram rebuild must not mask a missing ground cache key");
    assert_eq!(kernel.failed.len(), 2);
    assert!(kernel.failed.iter().all(|f| f.start_ground == carried));
    let repeated = kernel.compute_inner(&request, &cancel).err().expect("new exact negative reuse");
    assert_eq!(calls.load(Ordering::SeqCst), 4);
    assert_eq!(repeated.iter().map(|f| f.reason.as_str()).collect::<Vec<_>>(),
        ["ground-cache-unit-3", "ground-cache-unit-4"]);
    assert!(Arc::ptr_eq(&tomogram, &kernel.local_map.as_ref().unwrap().2));
}

}
