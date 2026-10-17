//! 局部几何计算拥有独立线程；续租、取消和输入接收只交换有界快照。
use super::*;
use std::sync::{Condvar, Mutex};

type Route = (ExecutionPath, Option<Vec<f64>>, PoseStamped);
#[derive(Default)]
struct Inputs {
    goal_id:String,
    epoch: u64,
    goal_version: u64,
    boundary: Vec<[f64;2]>,
    boundary_version: u64,
    path_version: u64,
    goal: Option<(PoseStamped, nuwax_nav_types_messages::GoalOptions)>,
    path: Option<Arc<Route>>,
    map: Option<ExecutionMap>,
    map_version: u64,
    map_pending: bool,
    terrain_generation: u64,
    result_terrain_generation: u64,
    pose: Option<PoseStamped>,
    pose_version: u64,
    requested: Option<f64>,
    control_width: Option<f64>,
    applied: Twist2,
    applied_version: u64,
    result: Option<(std::time::Instant, PlannerStep)>,
    result_dispatch_bound: Option<CommandDispatchBound>,
    #[cfg(test)]
    result_proof: Option<crate::command_proof_trace::PublishedCommandContext>,
    result_pose_version: u64,
    result_applied_version: u64,
    result_map_version: u64,
    result_boundary_version: u64,
    result_wait_reason: Option<String>,
    display_path: Vec<[f64; 3]>,
    closed: bool,
    lifecycle_stop: bool,
    lifecycle_update: bool,
}

// A complete, continuous surface update can recheck the selected stopping
// trajectory against its latest geometry instead of ranking every candidate
// again. This never reuses an old collision answer or crosses an Unknown gap.
fn recheckable_surface_update(previous:Option<&ExecutionMap>,next:&ExecutionMap)->bool {
    let Some(previous)=previous else {return false;};
    let previous=match previous {ExecutionMap::Surface(map)=>map,
        ExecutionMap::SurfaceUpdate(update)=>&update.snapshot,_=>return false};
    let ExecutionMap::SurfaceUpdate(update)=next else {return false;};
    update.change_basis.as_ref().is_some_and(|basis|nuwax_nav_pct::same_source_geometry(previous,basis))
        && surface_changes::same_query_semantics(previous,&update.snapshot)
        && match update.changes.as_ref() {
            surface_changes::SurfaceGeometryChanges::Empty =>
                nuwax_nav_pct::same_source_geometry(previous,&update.snapshot),
            surface_changes::SurfaceGeometryChanges::Columns {..} => true,
            surface_changes::SurfaceGeometryChanges::Unknown => false,
        }
}

// Only timestamps may advance without changing pose-derived authority. Preserve
// signed-zero distinctions too: geometric projections can branch on their bits.
fn same_pose(a: &PoseStamped, b: &PoseStamped) -> bool {
    fn bits(pose: &PoseStamped) -> [u64; 7] {
        let p = pose.pose.position;
        let q = pose.pose.orientation;
        [p.x, p.y, p.z, q.x, q.y, q.z, q.w].map(f64::to_bits)
    }
    a.frame_id == b.frame_id && bits(a) == bits(b)
}

// An already computed success records a guard-valid arrival event. Requalifying
// that event within the original strict goal region does NOT validate the latest
// body footprint and never grants motion. Source/path/goal/applied fences still
// apply; Pending and negative outcomes cannot become success through this check.
fn arrival_event_still_in_goal(input: &Inputs) -> bool {
    let (Some(pose), Some((goal, options)), Some(path)) =
        (&input.pose, &input.goal, &input.path) else { return false; };
    if input.map.is_none() || validate_map_pose(pose, "odometry").is_err()
        || validate_map_pose(goal, "goal_pose").is_err()
        || !same_pose(goal, &path.2) {
        return false;
    }
    let yaw = nuwax_nav_astar::controllers::angle_diff(
        goal.pose.orientation.yaw(), pose.pose.orientation.yaw()).abs();
    pose.pose.position.distance(goal.pose.position) < options.position_tolerance_m
        && (!options.requires_orientation() || yaw < options.yaw_tolerance_rad)
}

/// 计算线程没有运动端，结果须经原调度器核对目标身份再执行。
pub(crate) struct PlannerWorker {
    shared: Arc<(Mutex<Inputs>, Condvar)>,
    thread: Option<std::thread::JoinHandle<()>>,
    wait_reason: Option<String>,
    delivered_dispatch_bound: Option<CommandDispatchBound>,
    #[cfg(test)]
    delivered_command_deadline: Option<std::time::Instant>,
    #[cfg(test)]
    delivered_command_context: Option<crate::command_proof_trace::DeliveredCommandContext>,
}
impl PlannerWorker {
    #[cfg(test)]
    pub(crate) fn delivered_command_deadline_for_test(&self)->Option<std::time::Instant> {self.delivered_command_deadline}
    #[cfg(test)]
    pub(crate) fn delivered_command_context_for_test(&self)->Option<&crate::command_proof_trace::DeliveredCommandContext> {
        self.delivered_command_context.as_ref()
    }
    pub(crate) fn new(mut planner: impl NavPlanner) -> Result<Self, RuntimeError> {
        let shared = Arc::new((Mutex::new(Inputs::default()), Condvar::new()));
        let worker = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("nav-local-compute".into())
            .spawn(move || {
                let (lock, wake) = &*worker;
                let mut goal_version = u64::MAX;
                let mut path_version = u64::MAX;
                let mut map_version = 0;
                let mut boundary_version = u64::MAX;
                let mut slow_log_at: Option<std::time::Instant> = None;
                #[cfg(test)] let mut result_id=0_u64;
                loop {
                    let (
                        version,
                        goal_id,
                        goal_seq,
                        path_seq,
                        goal,
                        boundary,
                        boundary_seq,
                        path,
                        map,
                        mut map_seq,
                        terrain_seq,
                        replay,
                        pose,
                        mut pose_seq,
                        width,
                        applied,
                        mut applied_seq,
                    ) = {
                        let mut input = lock.lock().unwrap();
                        while input.requested.is_none() && !input.lifecycle_stop && !input.lifecycle_update && !input.closed {
                            input = wake.wait(input).unwrap();
                        }
                        if input.closed {
                            break;
                        }
                        if input.lifecycle_stop {
                            input.lifecycle_stop=false;
                            // A newer goal/path may already have coalesced after
                            // stop. Never mark that uninstalled state consumed.
                            goal_version=if input.goal.is_some(){u64::MAX}else{input.goal_version};
                            path_version=if input.path.is_some(){u64::MAX}else{input.path_version};
                            drop(input);
                            // Revoke inner planning even without another control tick.
                            planner.stop();
                            continue;
                        }
                        input.lifecycle_update=false;
                        input.map_pending=false;
                        let replay=input.result.as_ref().and_then(|(started,step)| {
                            let PlannerStep::Command(command)=step else {return None;};
                            (*command!=Twist2::ZERO && started.elapsed()<=DEFAULT_CONTROL_HORIZON
                                && (input.result_map_version!=input.map_version
                                    || input.result_pose_version!=input.pose_version
                                    || input.result_applied_version!=input.applied_version)
                                && input.result_terrain_generation==input.terrain_generation
                                && input.result_boundary_version==input.boundary_version)
                                .then_some((*started,*command,
                                    input.result_pose_version!=input.pose_version
                                        || input.result_applied_version!=input.applied_version,
                                    input.result_dispatch_bound))
                        });
                        (
                            input.epoch,
                            input.goal_id.clone(),
                            input.goal_version,
                            input.path_version,
                            input.goal.clone(),
                            if input.boundary_version != boundary_version || input.goal_version != goal_version {
                                input.boundary.clone()
                            } else { Vec::new() },
                            input.boundary_version,
                            input.path.clone(),
                            // Planar maps own their cells; unchanged inputs need no deep copy.
                            if input.map_version != map_version { input.map.clone() } else { None },
                            input.map_version,
                            input.terrain_generation,
                            replay,
                            input.pose.clone(),
                            input.pose_version,
                            input.requested.take(),
                            input.applied,
                            input.applied_version,
                        )
                    };
                    let mut started = replay.map_or_else(std::time::Instant::now,|(started,_,_,_)|started);
                    let mut rechecked=false;
                    #[cfg(test)] let compute_trace=width.map(|_| {
                        result_id=result_id.checked_add(1).expect("test worker result ID exhausted");
                        crate::command_proof_trace::ComputeScope::begin(crate::command_proof_trace::ComputeInputTrace {
                            result_id,
                            pose:pose.as_ref().map(crate::command_proof_trace::PoseTrace::capture),
                            applied_bits:crate::command_proof_trace::command_bits(applied),
                            epoch:version,goal_version:goal_seq,path_version:path_seq,map_version:map_seq,
                            boundary_version:boundary_seq,pose_version:pose_seq,applied_version:applied_seq,
                            compute_started:started,compute_start_unix_ns:crate::command_proof_trace::unix_ns(),
                        })
                    });
                    let span=tracing::info_span!("navigation.local",goal_id=%goal_id);
                    let _entered=span.enter();
                    let mut computed = (|| -> Result<PlannerStep, RuntimeError> {
                        if goal_seq != goal_version {
                            planner.stop();
                            if let Some((goal, options)) = &goal {
                                planner.configure_goal(goal, *options)?;
                            }
                            goal_version = goal_seq;
                            path_version = u64::MAX;
                            boundary_version = u64::MAX;
                        }
                        if boundary_seq != boundary_version {
                            planner.configure_boundary(&boundary);
                            boundary_version = boundary_seq;
                        }
                        if map_seq != map_version {
                            if let Some(map) = map {
                                planner.update_execution_map(map)?;
                            } else {
                                planner.invalidate_execution_map();
                            }
                            map_version = map_seq;
                        }
                        if let Some(pose) = pose {
                            planner.update_odometry(pose);
                        }
                        // 同一目标的新路径只替换路线，保留速度、制动及跟随状态。
                        if path_seq != path_version {
                            if let Some(path) = &path {
                                planner.replace_execution_path(
                                    path.0.clone(),
                                    path.1.clone(),
                                    path.2.clone(),
                                )?;
                            }
                            path_version = path_seq;
                        }
                        planner.observe_applied_command(applied);
                        if let Some((_,command,current_state,_))=replay {
                            let checked=if current_state {planner.revalidate_selected_command_at_current_state(command)}
                                else {planner.revalidate_selected_command(command)};
                            if let Some(result)=checked {
                                rechecked=true;return Ok(result);
                            }
                            // Unsupported planners retain their ordinary full-compute semantics.
                            started=std::time::Instant::now();
                        }
                        Ok(width.map_or(PlannerStep::Pending(Vec::new()),|width|planner.step(width)))
                    })()
                    .unwrap_or_else(|error| PlannerStep::ReplanRequired(error.to_string()));
                    // Keep the original source bound even when later rechecks use
                    // fresher evidence. A selected trajectory never renews its lease.
                    let candidate_bound=planner.take_command_dispatch_bound();
                    let mut dispatch_bound=match (replay.and_then(|r|r.3),candidate_bound) {
                        (Some(a),Some(b))=>Some(a.earliest(b)),(a,b)=>a.or(b),
                    };
                    // Catch up only this selected trajectory, not candidate ranking.
                    // Input reception remains independent of this geometry work.
                    while matches!(computed,PlannerStep::Command(v) if v!=Twist2::ZERO)
                        && started.elapsed()<=DEFAULT_CONTROL_HORIZON {
                        let latest={
                            let mut input=lock.lock().unwrap();
                            if input.epoch==version && input.boundary_version==boundary_seq && !input.closed
                                && input.terrain_generation==terrain_seq
                                && (input.map_version!=map_seq || input.pose_version!=pose_seq
                                    || input.applied_version!=applied_seq) {
                                input.map_pending=false;
                                Some((input.map_version,
                                    if input.map_version != map_seq { input.map.clone() } else { None },
                                    input.pose_version,input.pose.clone(),
                                    input.applied_version,input.applied))
                            } else {None}
                        };
                        let Some((sequence,map,latest_pose_seq,pose,latest_applied_seq,applied))=latest else {break;};
                        if sequence!=map_seq {
                            if let Some(map)=map {if let Err(error)=planner.update_execution_map(map) {
                                computed=PlannerStep::ReplanRequired(error.to_string());break;
                            }}
                            map_version=sequence;
                        }
                        let current_state=latest_pose_seq!=pose_seq || latest_applied_seq!=applied_seq;
                        if current_state {
                            if let Some(pose)=pose {planner.update_odometry(pose);}
                            planner.observe_applied_command(applied);
                        }
                        let PlannerStep::Command(command)=computed else {unreachable!()};
                        let checked=if current_state {planner.revalidate_selected_command_at_current_state(command)}
                            else {planner.revalidate_selected_command(command)};
                        let Some(checked)=checked else {break;};
                        if let Some(bound)=planner.take_command_dispatch_bound() {
                            dispatch_bound=Some(dispatch_bound.map_or(bound,|old|old.earliest(bound)));
                        }
                        computed=checked;map_seq=sequence;pose_seq=latest_pose_seq;applied_seq=latest_applied_seq;rechecked=true;
                    }
                    #[cfg(test)] let computed_proof=compute_trace.and_then(|trace|trace.finish(&computed));
                    if !matches!(&computed,PlannerStep::Command(v) if *v!=Twist2::ZERO) {dispatch_bound=None;}
                    if width.is_none() {continue;}
                    let mut wait_reason = if matches!(computed, PlannerStep::Pending(_))
                        || matches!(computed, PlannerStep::Command(command) if command == Twist2::ZERO) {
                        planner.execution_wait_reason()
                    } else { None };
                    let display_path = if matches!(computed, PlannerStep::Command(_)) {
                        planner.local_path_for_display()
                    } else { Vec::new() };
                    let elapsed = started.elapsed();
                    // 快速重规划也保留原因，避免被紧接着的路径接纳状态覆盖；不改变计算结果。
                    let diagnostic_result = ((elapsed > DEFAULT_CONTROL_HORIZON
                        || matches!(computed, PlannerStep::ReplanRequired(_) | PlannerStep::Failed(_)))
                        && slow_log_at.is_none_or(|last| last.elapsed() >= Duration::from_secs(5)))
                        .then(|| format!("{computed:?}"));
                    let mut input = lock.lock().unwrap();
                    let current = input.epoch == version
                        && input.applied_version == applied_seq
                        && input.map_version == map_seq
                        && input.boundary_version == boundary_seq
                        && (!(rechecked || matches!(computed,PlannerStep::Command(v) if v!=Twist2::ZERO))
                            || input.pose_version == pose_seq)
                        && (!matches!(computed, PlannerStep::ReplanRequired(_) | PlannerStep::Failed(_))
                            || input.pose_version == pose_seq)
                        && !input.closed;
                    // 换目标、换路径、取消或来源撤销后的迟到结果一律退休。
                    if current {
                        let computed=if dispatch_bound.is_some_and(CommandDispatchBound::expired_now) {
                            dispatch_bound=None;
                            wait_reason=Some("观察扫掠凭据在发布前已过期，保持零速并重新计算".into());
                            PlannerStep::Pending(Vec::new())
                        }else{computed};
                        input.result_dispatch_bound=dispatch_bound;
                        input.result = Some((started, computed));
                        #[cfg(test)] {input.result_proof=if matches!(input.result.as_ref(),Some((_,PlannerStep::Command(_)))) {computed_proof.map(|proof|proof.published())}else{None};}
                        input.result_pose_version = pose_seq;
                        input.result_applied_version = applied_seq;
                        input.result_map_version = map_seq;
                        input.result_terrain_generation=terrain_seq;
                        if rechecked && input.pose_version==pose_seq {
                            input.requested=None;input.lifecycle_update=false;
                        }
                        input.result_boundary_version = boundary_seq;
                        input.result_wait_reason = wait_reason;
                        input.display_path = display_path;
                    }
                    drop(input);
                    // 日志不持输入锁，慢磁盘不能阻塞调度器的取消与续租。
                    if let Some(result) = diagnostic_result {
                        tracing::warn!(target: "navigation.local", event = "navigation.local.result",
                            elapsed_ms = elapsed.as_secs_f64() * 1000.0, current, result,
                            "局部计算诊断：慢结果或重规划/失败原因；原有速度许可窗口保持不变");
                        slow_log_at = Some(std::time::Instant::now());
                    }
                }
                planner.stop();
            })
            .map_err(|error| RuntimeError::Planner(error.to_string()))?;
        Ok(Self {
            shared,
            thread: Some(thread),
            wait_reason: None,
            delivered_dispatch_bound: None,
            #[cfg(test)]
            delivered_command_deadline: None,
            #[cfg(test)]
            delivered_command_context: None,
        })
    }
}
impl NavPlanner for PlannerWorker {
    fn set_goal_log_context(&mut self,goal_id:&str) {
        self.shared.0.lock().unwrap().goal_id=goal_id.to_owned();
    }
    fn configure_boundary(&mut self, boundary: &[[f64;2]]) {
        self.delivered_dispatch_bound=None;
        let mut input=self.shared.0.lock().unwrap();
        input.boundary=boundary.to_vec();
        input.boundary_version += 1;
        input.display_path.clear();
        input.result_wait_reason = None;
        input.requested = input.control_width;
        input.lifecycle_update = true;
        self.shared.1.notify_one();
    }
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: nuwax_nav_types_messages::GoalOptions,
    ) -> Result<(), RuntimeError> {
        self.delivered_dispatch_bound=None;
        options.validate().map_err(RuntimeError::Planner)?;
        // 自主导航授权按前向策略收窄；旧目标允许倒退不能导致误拒或恢复倒退。
        let options=nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options};
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
        input.result_dispatch_bound=None;
        #[cfg(test)] {input.result_proof=None;}
        input.display_path.clear();
        input.path = None;
        input.goal_version += 1;
        input.path_version += 1;
        input.goal = Some((goal.clone(), options));
        input.lifecycle_update = true;
        self.shared.1.notify_one();
        Ok(())
    }
    fn replace_path(&mut self, _path: Path2D) {
        self.stop();
    }
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        self.delivered_dispatch_bound=None;
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
        input.result_dispatch_bound=None;
        #[cfg(test)] {input.result_proof=None;}
        input.display_path.clear();
        input.path_version += 1;
        input.path = Some(Arc::new((path, yaws, goal)));
        input.lifecycle_update = true;
        self.shared.1.notify_one();
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        self.delivered_dispatch_bound=None;
        let mut input = self.shared.0.lock().unwrap();
        if input.pose.as_ref().is_none_or(|previous|
            !same_pose(previous, &pose))
        {
            input.pose_version += 1;
            // Refresh an unissued selection ahead of the next control tick.
            // This is one trajectory check; unchanged timestamp-only receipts
            // neither invalidate nor re-enumerate the selected candidate.
            if input.result.as_ref().is_some_and(|(_,step)|matches!(step,PlannerStep::Command(c) if *c!=Twist2::ZERO)) {
                input.requested=input.control_width;
                self.shared.1.notify_one();
            }
        }
        input.pose = Some(pose);
    }
    fn update_costmap(&mut self, map: CostMap2D) {
        let _ = self.update_execution_map(ExecutionMap::Planar(map));
    }
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        self.delivered_dispatch_bound=None;
        let mut input = self.shared.0.lock().unwrap();
        let recheckable=recheckable_surface_update(input.map.as_ref(),&map);
        let map = match (input.map_pending, input.map.as_ref(), map) {
            (true, Some(ExecutionMap::SurfaceUpdate(previous)), ExecutionMap::SurfaceUpdate(next)) => {
                let mut merged=previous.clone();merged.append(next);ExecutionMap::SurfaceUpdate(merged)
            }
            // A skipped legacy snapshot provides no complete delta across the gap.
            (true, Some(ExecutionMap::Surface(_)), ExecutionMap::SurfaceUpdate(mut next)) => {
                next.change_basis=None;
                next.changes=Arc::new(surface_changes::SurfaceGeometryChanges::Unknown);
                ExecutionMap::SurfaceUpdate(next)
            }
            (_, _, map) => map,
        };
        // Individually complete batches may coalesce beyond the existing delta
        // budget, or span an unconsumed legacy snapshot. Fence that resulting
        // Unknown too; the current check cannot inherit a lost source chain.
        if !recheckable || matches!(&map,ExecutionMap::SurfaceUpdate(update)
            if matches!(update.changes.as_ref(),surface_changes::SurfaceGeometryChanges::Unknown)) {
            input.terrain_generation+=1;
        }
        input.map = Some(map);
        input.map_pending = true;
        input.map_version += 1;
        input.display_path.clear();
        input.result_wait_reason = None;
        input.requested = input.control_width;
        input.lifecycle_update = true;
        self.shared.1.notify_one();
        Ok(())
    }
    fn invalidate_execution_map(&mut self) {
        self.stop();
        let mut input = self.shared.0.lock().unwrap();
        input.map = None;
        input.map_version += 1;
    }
    fn observe_applied_command(&mut self, command: Twist2) {
        self.delivered_dispatch_bound=None;
        let mut input = self.shared.0.lock().unwrap();
        if input.applied != command {
            input.applied_version += 1;
            input.applied = command;
        }
        // 执行回执到达后计算下一拍，使用实际已下发速度；不在本拍
        // 下发前抢算下一拍，避免候选被误作真实执行状态。
        input.requested = input.control_width;
        self.shared.1.notify_one();
    }
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {self.delivered_dispatch_bound.take()}
    fn step(&mut self, width: f64) -> PlannerStep {
        self.delivered_dispatch_bound=None;
        #[cfg(test)] {self.delivered_command_context=None;}
        let mut input = self.shared.0.lock().unwrap();
        if self
            .thread
            .as_ref()
            .is_none_or(|thread| thread.is_finished())
        {
            return PlannerStep::Failed("局部计算线程已退出".into());
        }
        input.control_width = Some(width);
        let expired = input.result.as_ref().is_some_and(|(started, step)|
            matches!(step, PlannerStep::Command(_)) && started.elapsed() > DEFAULT_CONTROL_HORIZON);
        let dispatch_expired=input.result_dispatch_bound.is_some_and(CommandDispatchBound::expired_now);
        let command_state_changed=input.result.as_ref().is_some_and(|(_,step)|
            matches!(step,PlannerStep::Command(c) if *c!=Twist2::ZERO)
                && (input.result_pose_version!=input.pose_version
                    || input.result_applied_version!=input.applied_version));
        let arrival_changed = input.result.as_ref().is_some_and(|(_, step)|
            matches!(step, PlannerStep::Succeeded) && input.result_pose_version != input.pose_version
                && !arrival_event_still_in_goal(&input));
        let terminal_changed = input.result.as_ref().is_some_and(|(_, step)|
            (matches!(step, PlannerStep::ReplanRequired(_) | PlannerStep::Failed(_))
                && input.result_pose_version != input.pose_version)
            || (matches!(step, PlannerStep::Succeeded | PlannerStep::ReplanRequired(_) | PlannerStep::Failed(_))
                && input.result_applied_version != input.applied_version));
        // A stored result authorizes only the exact map and boundary it consumed.
        let source_changed = input.result.is_some()
            && (input.result_map_version != input.map_version
                || input.result_boundary_version != input.boundary_version);
        if input.result.is_none() || input.result.as_ref().is_some_and(|(_,s)|matches!(s,PlannerStep::Pending(_))) || expired || dispatch_expired || arrival_changed || terminal_changed || source_changed || command_state_changed {
            input.requested = Some(width);
            self.shared.1.notify_one();
        }
        // A nonzero command result is consumed once. Its existing deadline still
        // bounds first delivery; it is not proof for another full hold from a
        // newer pose while the next computation is pending.
        let ready = input
            .result
            .as_ref()
            .filter(|_| !source_changed && !terminal_changed && !dispatch_expired && !command_state_changed)
            .filter(|(started, step)| {
                // 时效窗约束速度许可；失败、重规划及到达只触发原调度器归零，
                // 不能因计算慢而把这些结论吞成永远运行中的零速度。
                match step {
                    PlannerStep::Command(_) => started.elapsed() <= DEFAULT_CONTROL_HORIZON,
                    PlannerStep::Succeeded => !arrival_changed,
                    _ => true,
                }
            })
            .map(|(_, step)| step.clone());
        #[cfg(test)]
        { self.delivered_command_deadline=if matches!(&ready,Some(PlannerStep::Command(_))) {
            input.result.as_ref().map(|(started,_)|*started+DEFAULT_CONTROL_HORIZON)
        } else {None}; }
        // Select diagnostic context under the same lock as the exact result,
        // before its existing one-shot consumption. Never query TLS here or
        // infer provenance from a nearby result/time/guard invocation.
        #[cfg(test)] {
            self.delivered_command_context=match (&ready,&input.result,&input.result_proof) {
                (Some(PlannerStep::Command(command)),Some((started,_)),Some(proof)) =>
                    proof.retrieved(*started,*command),
                _ => None,
            };
        }
        self.delivered_dispatch_bound=if matches!(&ready,Some(PlannerStep::Command(v)) if *v!=Twist2::ZERO) {input.result_dispatch_bound}else{None};
        self.wait_reason = if ready.is_some() { input.result_wait_reason.clone() } else if arrival_changed {
            Some("位姿已更新，保持零速并重新核对到达状态".into())
        } else if terminal_changed {
            Some("位姿或执行回执已更新，保持零速并重新计算".into())
        } else if command_state_changed {
            Some("位姿或执行速度已更新，等待已选停车轨迹复核".into())
        } else if source_changed {
            Some("地图或边界已更新，保持零速并重新计算".into())
        } else if dispatch_expired {
            Some("观察扫掠凭据在交付前已过期，保持零速并重新计算".into())
        } else if expired {
            Some("局部速度计算结果已过期，保持零速并重新计算".into())
        } else {
            Some("局部轨迹正在计算，保持零速等待".into())
        };
        if matches!(&ready, Some(PlannerStep::Command(command)) if *command != Twist2::ZERO) {
            // Consume only a selection rechecked for the latest received state.
            // A same-valued future command needs a new selection. Applied
            // feedback (or a later step seeing no result) requests that compute;
            // selecting this candidate is not an actuator receipt.
            input.result = None;
            input.result_dispatch_bound=None;
            #[cfg(test)] {input.result_proof=None;}
        }
        ready.unwrap_or(PlannerStep::Pending(Vec::new()))
    }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        self.shared.0.lock().unwrap().display_path.clone()
    }
    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }
    fn stop(&mut self) {
        self.delivered_dispatch_bound=None;
        self.wait_reason = None;
        #[cfg(test)]
        {self.delivered_command_deadline=None;self.delivered_command_context=None;}
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
        input.result_dispatch_bound=None;
        #[cfg(test)] {input.result_proof=None;}
        input.display_path.clear();
        input.result_wait_reason = None;
        input.path = None;
        input.goal = None;
        input.applied = Twist2::ZERO;
        input.applied_version += 1;
        input.goal_version += 1;
        input.path_version += 1;
        input.requested = None;
        input.control_width = None;
        input.lifecycle_stop = true;
        self.shared.1.notify_one();
    }
}
impl Drop for PlannerWorker {
    fn drop(&mut self) {
        {
            let mut input = self.shared.0.lock().unwrap();
            input.closed = true;
            input.result = None;
            input.result_dispatch_bound=None;
            #[cfg(test)] {input.result_proof=None;}
            input.display_path.clear();
            self.shared.1.notify_one();
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    include!("worker_first_delivery_tests.rs");
    include!("worker_geometry_recheck_tests.rs");
    mod snapshot_tests { use super::*; include!("worker_snapshot_tests.rs"); }
    include!("worker_recent_recheck_tests.rs");
    use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
    use nuwax_nav_types_messages::{
        GoalOptions, MotionKinematics, MotionModel, NavFrameId, Path3D, Point3, Pose3,
        Quaternion, Timestamp,
    };

    #[test]
    fn selective_worker_coalesces_undelivered_related_and_far_columns() {
        use crate::surface_changes::{SurfaceGeometryChanges,SurfaceMapUpdate};
        let mut harness=RouteWorkerHarness::new(true);
        let _=harness.worker.step(0.3);
        let _=harness.next_computation(); // Owner is now held after a real guarded step.
        let source=harness.clear.clone();
        for columns in [vec![[10,10]],vec![[100,100]],vec![[110,100]]] {
            harness.worker.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {planar_reference:None,
                snapshot:source.clone(),change_basis:Some(source.clone()),
                changes:Arc::new(SurfaceGeometryChanges::Columns {fine_size:source.source_voxel_size_m(),
                    fine:columns,planner_size:source.config().voxel_size,coarse:vec![]}),
            })).unwrap();
        }
        let input=harness.worker.shared.0.lock().unwrap();
        let Some(ExecutionMap::SurfaceUpdate(update))=&input.map else {panic!("missing delta publication");};
        let SurfaceGeometryChanges::Columns {fine,..}=update.changes.as_ref() else {panic!("lost complete differences");};
        assert_eq!(fine,&vec![[10,10],[100,100],[110,100]]);
        assert!(input.map_pending);
        drop(input);
        harness.release.send(()).unwrap();
    }

    struct RouteComputation {
        outcome: PlannerStep,
        map: Arc<SurfaceSnapshot>,
        boundary: Vec<[f64; 2]>,
        odometry: PoseStamped,
        goal_updates: u64,
        path_updates: u64,
        local_planning_calls: u64,
        display_path: Vec<[f64; 3]>,
    }

    // Only publication is gated. Every input, actual geometry query, route update
    // and command below goes through the production RouteFollower/SurfaceFollower.
    struct GatedRouteFollower {
        inner: crate::RouteFollower,
        computed: std::sync::mpsc::Sender<RouteComputation>,
        release: std::sync::mpsc::Receiver<()>,
        map: Option<Arc<SurfaceSnapshot>>,
        boundary: Vec<[f64; 2]>,
        odometry: Option<PoseStamped>,
        goal_updates: u64,
        path_updates: u64,
    }
    impl NavPlanner for GatedRouteFollower {
        fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {self.inner.take_command_dispatch_bound()}
        fn configure_goal(&mut self, goal: &PoseStamped, options: GoalOptions) -> Result<(), RuntimeError> {
            self.inner.configure_goal(goal, options)?;
            self.goal_updates += 1;
            Ok(())
        }
        fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
            self.inner.configure_boundary(boundary);
            self.boundary = boundary.to_vec();
        }
        fn replace_path(&mut self, path: Path2D) { self.inner.replace_path(path); }
        fn replace_execution_path(&mut self, path: ExecutionPath, yaws: Option<Vec<f64>>, goal: PoseStamped)
            -> Result<(), RuntimeError>
        {
            self.inner.replace_execution_path(path, yaws, goal)?;
            self.path_updates += 1;
            Ok(())
        }
        fn update_odometry(&mut self, pose: PoseStamped) {
            self.inner.update_odometry(pose.clone());
            self.odometry = Some(pose);
        }
        fn update_costmap(&mut self, map: CostMap2D) { self.inner.update_costmap(map); }
        fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
            self.inner.update_execution_map(map.clone())?;
            self.map = match map { ExecutionMap::Surface(map) | ExecutionMap::SurfaceUpdate(surface_changes::SurfaceMapUpdate {snapshot:map,..}) => Some(map), _ => None };
            Ok(())
        }
        fn invalidate_execution_map(&mut self) {
            self.inner.invalidate_execution_map();
            self.map = None;
        }
        fn observe_applied_command(&mut self, command: Twist2) {
            self.inner.observe_applied_command(command);
        }
        fn revalidate_selected_command_at_current_state(&mut self,c:Twist2)->Option<PlannerStep> {
            self.inner.revalidate_selected_command_at_current_state(c)
        }
        fn execution_wait_reason(&self) -> Option<String> { self.inner.execution_wait_reason() }
        fn local_path_for_display(&self) -> Vec<[f64; 3]> { self.inner.local_path_for_display() }
        fn step(&mut self, width: f64) -> PlannerStep {
            let mut outcome = self.inner.step(width);
            // This fixture gates publication, not planning latency. Await initial
            // asynchronous admission before applying its publication barrier.
            let deadline=std::time::Instant::now()+Duration::from_secs(5);
            while matches!(&outcome,PlannerStep::Pending(failures) if failures.is_empty())
                && self.inner.local_path().is_none() && std::time::Instant::now()<deadline {
                std::thread::sleep(Duration::from_millis(1));outcome=self.inner.step(width);
            }
            let _ = self.computed.send(RouteComputation {
                outcome: outcome.clone(),
                map: self.map.as_ref().unwrap().clone(),
                boundary: self.boundary.clone(),
                odometry: self.odometry.as_ref().unwrap().clone(),
                goal_updates: self.goal_updates,
                path_updates: self.path_updates,
                local_planning_calls: self.inner.local_planning_calls(),
                display_path: self.inner.local_path_for_display(),
            });
            // A timeout bounds cleanup after an assertion failure, not ordering.
            let _ = self.release.recv_timeout(Duration::from_secs(15));
            outcome
        }
        fn stop(&mut self) { self.inner.stop(); }
    }

    struct RouteWorkerHarness {
        worker: PlannerWorker,
        computed: std::sync::mpsc::Receiver<RouteComputation>,
        release: std::sync::mpsc::Sender<()>,
        clear: Arc<SurfaceSnapshot>,
    }
    impl RouteWorkerHarness {
        fn new(local_planning_enabled: bool) -> Self {
            let clear = Arc::new(crate::surface_follower::test_snapshot(0.0, false)
                .with_body_envelope(SurfaceBodyEnvelope {
                    footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
                    z_bounds_m: [-0.05, 0.2], body_to_support_z_m: 0.4,
                    max_step_up_m: 0.2, max_step_down_m: 0.2,
                    max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
                }).unwrap());
            let mut config = crate::SurfaceFollowerConfig::new(0.4);
            config.local_planning_enabled = local_planning_enabled;
            config.controller.simulation = true;
            config.basic_kinematics = Some(MotionKinematics {
                model: MotionModel::Differential, max_forward_mps: 1.0,
                max_reverse_mps: 0.0, max_lateral_mps: 0.0, max_angular_rps: 0.8,
                minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
            });
            let (computed, receive) = std::sync::mpsc::channel();
            let (release, wait) = std::sync::mpsc::channel();
            let mut worker = PlannerWorker::new(GatedRouteFollower {
                inner: crate::RouteFollower::new(config).unwrap(),
                computed, release: wait, map: None, boundary: Vec::new(), odometry: None,
                goal_updates: 0, path_updates: 0,
            }).unwrap();
            let start = route_pose(0.5, 0);
            let goal = route_pose(3.0, 0);
            worker.configure_goal(&goal, GoalOptions::default()).unwrap();
            worker.update_execution_map(ExecutionMap::Surface(clear.clone())).unwrap();
            worker.update_odometry(start.clone());
            worker.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses: vec![start, goal.clone()],
            }), None, goal).unwrap();
            Self { worker, computed: receive, release, clear }
        }
        fn next_computation(&self) -> RouteComputation {
            self.computed.recv_timeout(Duration::from_secs(15))
                .expect("real RouteFollower did not finish its gated step")
        }
        fn change_source(&mut self, map_changed: bool) {
            // No movement was applied: stationary odometry still advances by one
            // real simulated control period, rather than replaying a frozen stamp.
            self.worker.update_odometry(route_pose(0.5, 1));
            if map_changed {
                // A current-footprint virtual obstacle cannot be cleared by the
                // controller's self-support view or bypassed by local replanning.
                let blocked = Arc::new(self.clear.with_virtual_obstacles(
                    [0.0, 0.0], 0.1, 65, 35, &vec![true; 65 * 35],
                ).unwrap());
                self.worker.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
            } else {
                self.worker.configure_boundary(&[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]]);
            }
        }
    }
    impl Drop for RouteWorkerHarness {
        fn drop(&mut self) {
            self.worker.stop();
            // Release a held computation even if an assertion has unwound.
            let _ = self.release.send(());
        }
    }
    fn route_pose(x: f64, tick: i64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp { nanos: 1_000_000_000 + tick * 50_000_000, ..Timestamp::default() },
            pose: Pose3 { position: Point3::new(x, 1.0, 0.5), orientation: Quaternion::IDENTITY },
        }
    }
    fn assert_positive_route_computation(computed: &RouteComputation, local_planning_enabled: bool) {
        assert!(matches!(computed.outcome, PlannerStep::Command(command) if command.linear.x > 0.0),
            "clear terrain must actually produce forward motion in mode {local_planning_enabled}: {:?}", computed.outcome);
        assert_eq!(computed.local_planning_calls > 0, local_planning_enabled);
        assert_eq!(!computed.display_path.is_empty(), local_planning_enabled);
    }
    fn assert_blocked_route_computation(computed: &RouteComputation, harness: &RouteWorkerHarness, map_changed: bool, local_planning_enabled: bool) {
        assert!(matches!(computed.outcome, PlannerStep::Command(command) if command == Twist2::ZERO)
            || matches!(computed.outcome, PlannerStep::Pending(_) | PlannerStep::ReplanRequired(_)),
            "the real follower must not move through the replacement source: {:?}", computed.outcome);
        assert_eq!(computed.odometry.stamp.nanos, route_pose(0.5, 1).stamp.nanos);
        assert_eq!(Arc::ptr_eq(&computed.map, &harness.clear), !map_changed);
        assert_eq!(computed.boundary.is_empty(), map_changed);
        if !local_planning_enabled { assert_eq!(computed.local_planning_calls, 0); }
    }

    #[test]
    fn both_route_modes_reject_stored_positive_commands_after_map_or_boundary_changes() {
        for local_planning_enabled in [true, false] {
            for map_changed in [true, false] {
                let mut harness = RouteWorkerHarness::new(local_planning_enabled);
                harness.worker.step(0.3);
                let clear = harness.next_computation();
                assert_positive_route_computation(&clear, local_planning_enabled);
                harness.release.send(()).unwrap();
                wait_for_source_result(&harness.worker);
                // Keep this positive publication unconsumed: this test isolates
                // the source fence before first delivery, not single-use retirement.
                assert_eq!(harness.worker.shared.0.lock().unwrap().result.as_ref().unwrap().1, clear.outcome);
                let goal_version = harness.worker.shared.0.lock().unwrap().goal_version;
                harness.change_source(map_changed);
                let blocked = harness.next_computation();
                assert_blocked_route_computation(&blocked, &harness, map_changed, local_planning_enabled);
                {
                    let mut input = harness.worker.shared.0.lock().unwrap();
                    assert_eq!(input.goal_version, goal_version, "a source update must preserve the goal");
                    assert_eq!(input.result.as_ref().unwrap().1, clear.outcome);
                    // Keep the real computed candidate unexpired so only the
                    // source fence, not wall-clock expiry, can make this pass.
                    input.result.as_mut().unwrap().0 = std::time::Instant::now();
                }
                assert_eq!(harness.worker.step(0.3), PlannerStep::Pending(Vec::new()));
                assert!(harness.worker.execution_wait_reason().unwrap().contains("地图或边界"));
                assert!(harness.worker.local_path_for_display().is_empty());
            }
        }
    }

    #[test]
    fn both_route_modes_retire_inflight_results_after_map_or_boundary_changes() {
        for local_planning_enabled in [true, false] {
            for map_changed in [true, false] {
                let mut harness = RouteWorkerHarness::new(local_planning_enabled);
                harness.worker.step(0.3);
                let clear = harness.next_computation();
                assert_positive_route_computation(&clear, local_planning_enabled);
                harness.change_source(map_changed);
                harness.release.send(()).unwrap();
                // No new step/applied-command call: source replacement itself
                // must wake the worker and retire the already-computed command.
                let blocked = harness.next_computation();
                assert_blocked_route_computation(&blocked, &harness, map_changed, local_planning_enabled);
                assert!(harness.worker.shared.0.lock().unwrap().result.is_none(),
                    "late command from the old source was published in mode {local_planning_enabled}");
                assert!(harness.worker.local_path_for_display().is_empty());
                harness.release.send(()).unwrap();
                wait_for_source_result(&harness.worker);
                assert_eq!(harness.worker.shared.0.lock().unwrap().result.as_ref().unwrap().1, blocked.outcome);
                assert!(!matches!(harness.worker.step(0.3), PlannerStep::Command(command) if command != Twist2::ZERO));
            }
        }
    }

    #[test]
    fn both_route_modes_retire_inflight_commands_on_cancel_without_replacement() {
        for local_planning_enabled in [true, false] {
            let mut harness = RouteWorkerHarness::new(local_planning_enabled);
            harness.worker.step(0.3);
            let old = harness.next_computation();
            assert_positive_route_computation(&old, local_planning_enabled);
            harness.worker.stop();
            {
                let input = harness.worker.shared.0.lock().unwrap();
                assert!(input.result.is_none() && input.path.is_none() && input.goal.is_none());
                assert!(input.requested.is_none() && input.control_width.is_none());
                assert!(input.display_path.is_empty());
            }
            harness.worker.update_odometry(route_pose(0.5, 1));
            // A pathless tick is solely a completion barrier for the old step.
            // It cannot change the goal/path epoch and therefore cannot mask a
            // broken cancellation fence with replacement-goal invalidation.
            let cancelled_epoch = harness.worker.shared.0.lock().unwrap().epoch;
            assert_eq!(harness.worker.step(0.3), PlannerStep::Pending(Vec::new()));
            harness.release.send(()).unwrap();
            let cancelled = harness.next_computation();
            assert!(matches!(cancelled.outcome, PlannerStep::Pending(_) | PlannerStep::ReplanRequired(_))
                || matches!(cancelled.outcome, PlannerStep::Command(command) if command == Twist2::ZERO));
            assert_eq!(cancelled.goal_updates, 1);
            assert_eq!(cancelled.path_updates, 1);
            assert!(cancelled.odometry.stamp.nanos > old.odometry.stamp.nanos);
            if !local_planning_enabled { assert_eq!(cancelled.local_planning_calls, 0); }
            let input = harness.worker.shared.0.lock().unwrap();
            assert_eq!(input.epoch, cancelled_epoch);
            assert!(input.result.is_none(), "the cancelled route's late command was published");
            assert!(input.display_path.is_empty());
        }
    }

    #[test]
    fn both_route_modes_retire_inflight_commands_on_same_goal_path_replacement() {
        for local_planning_enabled in [true, false] {
            let mut harness = RouteWorkerHarness::new(local_planning_enabled);
            harness.worker.step(0.3);
            let old = harness.next_computation();
            assert_positive_route_computation(&old, local_planning_enabled);
            let old_epoch = harness.worker.shared.0.lock().unwrap().epoch;
            let goal = route_pose(3.0, 1);
            let start = route_pose(0.5, 1);
            harness.worker.update_odometry(start.clone());
            harness.worker.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses: vec![start, route_pose(1.5, 1), goal.clone()],
            }), None, goal).unwrap();
            assert!(harness.worker.shared.0.lock().unwrap().epoch > old_epoch);
            assert_eq!(harness.worker.step(0.3), PlannerStep::Pending(Vec::new()));
            harness.release.send(()).unwrap();
            let replacement = harness.next_computation();
            assert_positive_route_computation(&replacement, local_planning_enabled);
            assert_eq!(replacement.goal_updates, 1);
            assert_eq!(replacement.path_updates, 2);
            assert!(replacement.odometry.stamp.nanos > old.odometry.stamp.nanos);
            assert!(harness.worker.shared.0.lock().unwrap().result.is_none(),
                "path replacement allowed the old route's late command to return");
            harness.release.send(()).unwrap();
            wait_for_source_result(&harness.worker);
            assert_eq!(harness.worker.shared.0.lock().unwrap().result.as_ref().unwrap().1, replacement.outcome);
            assert_eq!(harness.worker.local_path_for_display(), replacement.display_path);
        }
    }

    // Each step announces the sources it consumed, then waits for an explicit
    // release. The timeout only bounds a failed test's Drop; ordering uses channels.
    #[test]
    fn stop_then_new_goal_and_path_before_old_compute_returns_installs_latest_inputs() {
        for enabled in [false,true] {
            let mut harness=RouteWorkerHarness::new(enabled);
            harness.worker.step(0.3);
            let old=harness.next_computation();assert_positive_route_computation(&old,enabled);
            harness.worker.stop();
            let goal=route_pose(4.0,1);
            harness.worker.configure_goal(&goal,GoalOptions::default()).unwrap();
            harness.worker.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses:vec![route_pose(0.5,1),goal.clone()]
            }),None,goal).unwrap();
            harness.worker.step(0.3);
            harness.release.send(()).unwrap();
            let latest=harness.next_computation();
            assert_positive_route_computation(&latest,enabled);
            assert_eq!(latest.goal_updates,old.goal_updates+1);
            assert_eq!(latest.path_updates,old.path_updates+1);
            harness.release.send(()).unwrap();
        }
    }

    struct SourceProbe {
        started: std::sync::mpsc::Sender<(bool, Vec<[f64; 2]>)>,
        release: std::sync::mpsc::Receiver<()>,
        blocked: bool,
        boundary: Vec<[f64; 2]>,
    }
    impl NavPlanner for SourceProbe {
        fn replace_path(&mut self, _: Path2D) {}
        fn update_odometry(&mut self, _: PoseStamped) {}
        fn update_costmap(&mut self, map: CostMap2D) {
            self.blocked = map.get(0, 0) != 0;
        }
        fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
            self.boundary = boundary.to_vec();
        }
        fn step(&mut self, _: f64) -> PlannerStep {
            let _ = self.started.send((self.blocked, self.boundary.clone()));
            let _ = self.release.recv_timeout(Duration::from_secs(2));
            PlannerStep::Command(if self.blocked || !self.boundary.is_empty() {
                Twist2::ZERO
            } else {
                Twist2 { linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0), angular_z: 0.0 }
            })
        }
        fn stop(&mut self) { self.boundary.clear(); }
    }
    fn source_worker() -> (
        PlannerWorker,
        std::sync::mpsc::Receiver<(bool, Vec<[f64; 2]>)>,
        std::sync::mpsc::Sender<()>,
    ) {
        let (started, start) = std::sync::mpsc::channel();
        let (release, receive) = std::sync::mpsc::channel();
        let worker = PlannerWorker::new(SourceProbe {
            started, release: receive, blocked: false, boundary: Vec::new(),
        }).unwrap();
        (worker, start, release)
    }
    fn wait_for_source_result(worker: &PlannerWorker) {
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        loop {
            let input = worker.shared.0.lock().unwrap();
            if input.result.is_some()
                && input.result_map_version == input.map_version
                && input.result_boundary_version == input.boundary_version {
                return;
            }
            drop(input);
            assert!(std::time::Instant::now() < deadline, "current-source result was not published");
            std::thread::yield_now();
        }
    }

    // Synthetic selections isolate the diagnostic handoff; they assert no
    // geometry permission. Real guard selection is traced only at its callsite.
    struct TraceProbe {
        started: std::sync::mpsc::Sender<()>,
        finish: std::sync::mpsc::Receiver<(PlannerStep, Option<u64>)>,
        pose: Option<PoseStamped>,
        applied: Twist2,
    }
    impl NavPlanner for TraceProbe {
        fn replace_path(&mut self, _: Path2D) {}
        fn update_odometry(&mut self, pose: PoseStamped) { self.pose=Some(pose); }
        fn update_costmap(&mut self, _: CostMap2D) {}
        fn observe_applied_command(&mut self, applied: Twist2) { self.applied=applied; }
        fn step(&mut self, _: f64) -> PlannerStep {
            let _=self.started.send(());
            let (outcome,anchor)=self.finish.recv_timeout(Duration::from_secs(5))
                .unwrap_or((PlannerStep::Pending(Vec::new()),None));
            if let (PlannerStep::Command(command),Some(anchor))=(&outcome,anchor) {
                let guard=crate::command_proof_trace::GuardScope::begin();
                guard.record_basis(self.pose.as_ref().unwrap(),self.applied,Some((anchor,1_000_000_000)));
                crate::command_proof_trace::record_selected(*command);
            }
            outcome
        }
        fn stop(&mut self) {}
    }
    fn traced_worker()->(PlannerWorker,std::sync::mpsc::Receiver<()>,std::sync::mpsc::Sender<(PlannerStep,Option<u64>)>) {
        let (started,receive)=std::sync::mpsc::channel();
        let (finish,wait)=std::sync::mpsc::channel();
        let worker=PlannerWorker::new(TraceProbe {started,finish:wait,pose:None,applied:Twist2::ZERO}).unwrap();
        (worker,receive,finish)
    }

    #[test]
    fn exact_proof_follows_stored_result_once_and_never_same_valued_next_result() {
        let (mut worker,started,finish)=traced_worker();
        let original_pose=route_pose(0.5,0);
        let command=Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.05,0.0),angular_z:0.0};
        worker.update_odometry(original_pose.clone());
        assert!(matches!(worker.step(0.3),PlannerStep::Pending(_)));
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        // A timestamp-only receipt preserves geometric authority and the exact
        // original computation provenance; moved-pose checks are separate.
        worker.update_odometry(route_pose(0.5,1));
        finish.send((PlannerStep::Command(command),Some(101))).unwrap();
        wait_for_source_result(&worker);
        let published_id=worker.shared.0.lock().unwrap().result_proof.as_ref().unwrap().proof.input.result_id;
        assert_eq!(worker.step(0.3),PlannerStep::Command(command));
        let delivered=worker.delivered_command_context_for_test().unwrap().clone();
        assert_eq!(delivered.proof.input.result_id,published_id);
        assert_eq!(delivered.proof.input.pose,Some(crate::command_proof_trace::PoseTrace::capture(&original_pose)));
        assert_eq!(delivered.proof.guard.observation_anchor_ns,Some(101));
        assert!(delivered.proof.worker_guard_pose_bits_equal && delivered.proof.worker_guard_applied_bits_equal);
        {
            let input=worker.shared.0.lock().unwrap();
            assert!(input.result.is_none() && input.result_proof.is_none());
        }
        assert!(matches!(worker.step(0.3),PlannerStep::Pending(_)));
        assert!(worker.delivered_command_context_for_test().is_none());
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        // Same command value, no scoped proof: diagnostics must be absent.
        finish.send((PlannerStep::Command(command),None)).unwrap();
        wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3),PlannerStep::Command(command));
        assert!(worker.delivered_command_context_for_test().is_none());
        worker.step(0.3);
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        finish.send((PlannerStep::Command(command),Some(303))).unwrap();
        wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3),PlannerStep::Command(command));
        let latest=worker.delivered_command_context_for_test().unwrap();
        assert!(latest.proof.input.result_id>published_id);
        assert_eq!(latest.proof.guard.observation_anchor_ns,Some(303));
        worker.stop();
        assert!(worker.delivered_command_context_for_test().is_none());
        assert!(worker.shared.0.lock().unwrap().result_proof.is_none());
    }

    #[test]
    fn source_fence_expiry_and_zero_never_deliver_stored_diagnostic_proof() {
        for changed_source in [false,true] {
            let (mut worker,started,finish)=traced_worker();
            let command=Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.05,0.0),angular_z:0.0};
            worker.update_odometry(route_pose(0.5,0));
            worker.step(0.3);
            started.recv_timeout(Duration::from_secs(1)).unwrap();
            finish.send((PlannerStep::Command(command),Some(101))).unwrap();
            wait_for_source_result(&worker);
            {
                let mut input=worker.shared.0.lock().unwrap();
                assert!(input.result_proof.is_some());
                if changed_source {input.map_version+=1;}
                else {input.result.as_mut().unwrap().0=std::time::Instant::now()-DEFAULT_CONTROL_HORIZON-Duration::from_millis(1);}
            }
            assert!(matches!(worker.step(0.3),PlannerStep::Pending(_)));
            assert!(worker.delivered_command_context_for_test().is_none());
            started.recv_timeout(Duration::from_secs(1)).unwrap();
            finish.send((PlannerStep::Command(Twist2::ZERO),Some(202))).unwrap();
            // The rejected positive result remains stored until replacement.
            // Wait for this released zero publication, not merely any Some.
            let deadline=std::time::Instant::now()+Duration::from_secs(1);
            loop {
                let input=worker.shared.0.lock().unwrap();
                if matches!(&input.result,Some((_,PlannerStep::Command(command))) if *command==Twist2::ZERO)
                    && input.result_map_version==input.map_version
                    && input.result_boundary_version==input.boundary_version {break;}
                drop(input);
                assert!(std::time::Instant::now()<deadline,"released zero result was not published");
                std::thread::yield_now();
            }
            assert_eq!(worker.step(0.3),PlannerStep::Command(Twist2::ZERO));
            assert!(worker.delivered_command_context_for_test().is_none());
            assert!(worker.shared.0.lock().unwrap().result_proof.is_none());
            worker.stop();
        }
    }

    #[test]
    fn stored_positive_command_is_rejected_after_blocking_map_update() {
        let (mut worker, start, release) = source_worker();
        worker.update_costmap(CostMap2D::new_all_free("map", 1, 1, 0.1));
        worker.step(0.3);
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (false, vec![]));
        release.send(()).unwrap();
        wait_for_source_result(&worker);
        // Keep a stored, unexpired positive result, making retrieval (not setter
        // clearing or command expiry) responsible for rejecting stale authority.
        {
            let input = worker.shared.0.lock().unwrap();
            assert!(matches!(input.result.as_ref().unwrap().1, PlannerStep::Command(t) if t.linear.x > 0.0));
        }
        worker.update_costmap(CostMap2D::new_all_occupied("map", 1, 1, 0.1));
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (true, vec![]));
        {
            let mut input = worker.shared.0.lock().unwrap();
            assert!(matches!(input.result.as_ref().unwrap().1, PlannerStep::Command(t) if t.linear.x > 0.0));
            input.result.as_mut().unwrap().0 = std::time::Instant::now();
        }
        assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()));
        assert!(worker.execution_wait_reason().unwrap().contains("地图或边界"));
        worker.stop();
        release.send(()).unwrap();
    }

    #[test]
    fn identical_surface_arc_still_fences_stored_and_inflight_commands() {
        struct SurfaceProbe {
            started: std::sync::mpsc::Sender<()>,
            release: std::sync::mpsc::Receiver<()>,
        }
        impl NavPlanner for SurfaceProbe {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn update_execution_map(&mut self, map: ExecutionMap)->Result<(),RuntimeError> {
                assert!(matches!(map,ExecutionMap::Surface(_))); Ok(())
            }
            fn step(&mut self, _: f64)->PlannerStep {
                let _=self.started.send(());
                let _=self.release.recv_timeout(Duration::from_secs(3));
                PlannerStep::Command(Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.2,0.0),angular_z:0.0})
            }
            fn stop(&mut self) {}
        }
        let (started,start)=std::sync::mpsc::channel();
        let (release,receive)=std::sync::mpsc::channel();
        let mut worker=PlannerWorker::new(SurfaceProbe {started,release:receive}).unwrap();
        let surface=crate::surface_follower::test_snapshot(0.0,false);
        worker.update_execution_map(ExecutionMap::Surface(surface.clone())).unwrap();
        worker.step(0.3); start.recv_timeout(Duration::from_secs(1)).unwrap();
        let old_version=worker.shared.0.lock().unwrap().map_version;
        // 原计算仍被屏障持有；重装同一Arc也必须退休它。
        worker.update_execution_map(ExecutionMap::Surface(surface.clone())).unwrap();
        assert_eq!(worker.shared.0.lock().unwrap().map_version,old_version+1);
        release.send(()).unwrap(); start.recv_timeout(Duration::from_secs(1)).unwrap();
        assert!(worker.shared.0.lock().unwrap().result.is_none());
        release.send(()).unwrap(); wait_for_source_result(&worker);
        worker.update_execution_map(ExecutionMap::Surface(surface)).unwrap();
        start.recv_timeout(Duration::from_secs(1)).unwrap();
        {
            let mut input=worker.shared.0.lock().unwrap();
            assert!(matches!(input.result.as_ref().unwrap().1,PlannerStep::Command(c) if c.linear.x>0.0));
            input.result.as_mut().unwrap().0=std::time::Instant::now();
        }
        assert_eq!(worker.step(0.3),PlannerStep::Pending(Vec::new()));
        worker.stop(); release.send(()).unwrap();
    }
    #[test]
    fn map_replacement_retires_inflight_result_and_wakes_without_control_tick() {
        let (mut worker, start, release) = source_worker();
        worker.update_costmap(CostMap2D::new_all_free("map", 1, 1, 0.1));
        worker.step(0.3);
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (false, vec![]));
        worker.update_costmap(CostMap2D::new_all_occupied("map", 1, 1, 0.1));
        release.send(()).unwrap();
        // No step or applied-command notification: map replacement must wake work.
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (true, vec![]));
        assert!(worker.shared.0.lock().unwrap().result.is_none(), "old-map result was published");
        release.send(()).unwrap();
        wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3), PlannerStep::Command(Twist2::ZERO));
    }
    #[test]
    fn boundary_replacement_retires_inflight_result_without_goal_change() {
        let (mut worker, start, release) = source_worker();
        let goal = nuwax_nav_astar::helpers::make_pose_stamped("map", 1.0, 0.0, 0.0);
        worker.configure_goal(&goal, Default::default()).unwrap();
        worker.step(0.3);
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (false, vec![]));
        let goal_version = worker.shared.0.lock().unwrap().goal_version;
        let boundary = vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
        worker.configure_boundary(&boundary);
        release.send(()).unwrap();
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (false, boundary.clone()));
        assert!(worker.shared.0.lock().unwrap().result.is_none(), "old-boundary result was published");
        assert_eq!(worker.shared.0.lock().unwrap().goal_version, goal_version);
        release.send(()).unwrap();
        wait_for_source_result(&worker);
        // A new goal stops the planner; unchanged boundary still must be restored.
        worker.configure_goal(&goal, Default::default()).unwrap();
        worker.step(0.3);
        assert_eq!(start.recv_timeout(Duration::from_secs(1)).unwrap(), (false, boundary));
        worker.stop();
        release.send(()).unwrap();
    }
    #[test]
    fn stored_outcomes_require_current_map_and_boundary_at_retrieval() {
        let (mut worker, start, release) = source_worker();
        worker.step(0.3);
        start.recv_timeout(Duration::from_secs(1)).unwrap();
        // Hold actual computation while independently exercising every cached
        // outcome and each source fence, including destructive stale replans.
        for map_changed in [true, false] {
            for outcome in [
                PlannerStep::Command(Twist2 { linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0), angular_z: 0.0 }),
                PlannerStep::Succeeded,
                PlannerStep::ReplanRequired("old source".into()),
                PlannerStep::Failed("old source".into()),
            ] {
                {
                    let mut input = worker.shared.0.lock().unwrap();
                    input.result = Some((std::time::Instant::now(), outcome));
                    input.result_map_version = input.map_version;
                    input.result_boundary_version = input.boundary_version;
                    if map_changed { input.map_version += 1; }
                    else { input.boundary_version += 1; }
                }
                assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()));
            }
        }
        worker.stop();
        release.send(()).unwrap();
        // Source notifications after stop must not restart control.
        worker.update_costmap(CostMap2D::new_all_free("map", 1, 1, 0.1));
        worker.configure_boundary(&[]);
        assert!(worker.shared.0.lock().unwrap().requested.is_none());
    }
    #[test]
    fn local_display_drops_late_worker_results_after_stop() {
        struct DisplayPlanner { release: std::sync::mpsc::Receiver<()>, started: std::sync::mpsc::Sender<()> }
        impl NavPlanner for DisplayPlanner {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn step(&mut self, _: f64) -> PlannerStep {
                self.started.send(()).unwrap();
                self.release.recv().unwrap();
                PlannerStep::Command(Twist2::ZERO)
            }
            fn local_path_for_display(&self) -> Vec<[f64; 3]> { vec![[0.0,0.0,0.4],[1.0,0.5,0.4]] }
            fn stop(&mut self) {}
        }
        let (release, receive) = std::sync::mpsc::channel();
        let (started, start) = std::sync::mpsc::channel();
        let mut worker = PlannerWorker::new(DisplayPlanner {release:receive, started}).unwrap();
        worker.step(0.3);
        start.recv_timeout(Duration::from_secs(1)).unwrap();
        assert!(worker.local_path_for_display().is_empty());
        release.send(()).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        while worker.local_path_for_display().is_empty() {
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(worker.local_path_for_display().len(), 2);
        worker.observe_applied_command(Twist2::ZERO);
        start.recv_timeout(Duration::from_secs(1)).unwrap();
        worker.stop();
        release.send(()).unwrap();
        std::thread::sleep(Duration::from_millis(10));
        assert!(worker.local_path_for_display().is_empty(), "取消后的旧局部轨迹不能复活");
    }

    #[test]
    fn stationary_geometry_wait_reason_crosses_the_compute_thread() {
        struct WaitingPlanner;
        impl NavPlanner for WaitingPlanner {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn step(&mut self, _: f64) -> PlannerStep { PlannerStep::Pending(Vec::new()) }
            fn execution_wait_reason(&self) -> Option<String> { Some("等待当前足下支撑恢复".into()) }
            fn stop(&mut self) {}
        }
        let mut worker = PlannerWorker::new(WaitingPlanner).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        loop {
            assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()));
            if worker.execution_wait_reason().as_deref() == Some("等待当前足下支撑恢复") { break; }
            assert!(std::time::Instant::now() < deadline, "计算线程必须交付实际等待原因");
            std::thread::sleep(Duration::from_millis(1));
        }
        worker.stop();
        assert!(worker.execution_wait_reason().is_none());
        assert!(worker.shared.0.lock().unwrap().result_wait_reason.is_none());
    }
    use std::sync::atomic::{AtomicBool, Ordering};
    #[test]
    fn slow_planner_outcomes_are_not_lost_to_velocity_expiry() {
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered: Arc::new(AtomicBool::new(false)),
            release: Arc::new(AtomicBool::new(true)),
        }).unwrap();
        // 重规划和终态本身不授权运动；慢查询的结论仍必须交给原调度器收尾。
        for outcome in [
            PlannerStep::ReplanRequired("现场近场轨迹受阻".into()),
            PlannerStep::Failed("局部输入无效".into()),
            PlannerStep::Succeeded,
        ] {
            worker.shared.0.lock().unwrap().result = Some((
                std::time::Instant::now() - DEFAULT_CONTROL_HORIZON - Duration::from_millis(1),
                outcome.clone(),
            ));
            assert_eq!(worker.step(0.3), outcome);
        }
        worker.shared.0.lock().unwrap().result = Some((
            std::time::Instant::now() - DEFAULT_CONTROL_HORIZON - Duration::from_millis(1),
            PlannerStep::Command(Twist2 { linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0), angular_z: 0.0 }),
        ));
        assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()),
            "传播慢查询结论不能延长速度许可");
        assert!(worker.execution_wait_reason().unwrap().contains("过期"));
    }

    // Both entry and completion are channel-controlled; no sleeps determine races.
    struct TerminalPlanner {
        started: std::sync::mpsc::Sender<()>,
        outcome: std::sync::mpsc::Receiver<PlannerStep>,
    }
    impl NavPlanner for TerminalPlanner {
        fn replace_path(&mut self, _: Path2D) {}
        fn update_odometry(&mut self, _: PoseStamped) {}
        fn update_costmap(&mut self, _: CostMap2D) {}
        fn step(&mut self, _: f64) -> PlannerStep {
            let _ = self.started.send(());
            self.outcome.recv_timeout(Duration::from_secs(5))
                .unwrap_or(PlannerStep::Pending(Vec::new()))
        }
        fn stop(&mut self) {}
    }
    fn terminal_worker() -> (PlannerWorker, std::sync::mpsc::Receiver<()>, std::sync::mpsc::Sender<PlannerStep>) {
        let (started, receive) = std::sync::mpsc::channel();
        let (send, outcome) = std::sync::mpsc::channel();
        (PlannerWorker::new(TerminalPlanner { started, outcome }).unwrap(), receive, send)
    }
    #[test]
    fn pose_only_update_retires_inflight_negative_and_accepts_current_failure() {
        for negative in [PlannerStep::ReplanRequired("old pose".into()), PlannerStep::Failed("old pose".into())] {
            let (mut worker, started, finish) = terminal_worker();
            worker.update_odometry(route_pose(0.5, 0));
            worker.step(0.3);
            started.recv_timeout(Duration::from_secs(1)).unwrap();
            worker.update_odometry(route_pose(0.500001, 1));
            worker.step(0.3); // Queue a current-pose computation before releasing the old one.
            finish.send(negative.clone()).unwrap();
            started.recv_timeout(Duration::from_secs(1)).unwrap();
            assert!(worker.shared.0.lock().unwrap().result.is_none(), "old-pose terminal was published");
            finish.send(negative.clone()).unwrap();
            wait_for_source_result(&worker);
            assert_eq!(worker.step(0.3), negative, "a genuine current failure must stay authoritative");
            worker.stop();
        }
    }
    #[test]
    fn stored_negative_requires_exact_pose_but_not_same_timestamp() {
        for negative in [PlannerStep::ReplanRequired("pose".into()), PlannerStep::Failed("pose".into())] {
            let (mut worker, started, finish) = terminal_worker();
            let mut pose = route_pose(0.0, 0);
            worker.update_odometry(pose.clone());
            worker.step(0.3);
            started.recv_timeout(Duration::from_secs(1)).unwrap();
            finish.send(negative.clone()).unwrap();
            wait_for_source_result(&worker);
            pose.stamp.nanos += 1;
            worker.update_odometry(pose.clone());
            assert_eq!(worker.step(0.3), negative);
            pose.pose.position.x = -0.0;
            worker.update_odometry(pose);
            assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()));
            worker.stop();
            let _ = finish.send(PlannerStep::Pending(Vec::new()));
        }
    }
    #[test]
    fn pose_only_motion_requires_current_first_delivery_and_cannot_replay_it() {
        let (mut worker, started, finish) = terminal_worker();
        worker.update_odometry(route_pose(0.5, 0));
        worker.step(0.3);
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        worker.update_odometry(route_pose(0.500001, 1));
        let command = PlannerStep::Command(Twist2 {
            linear: nuwax_nav_types_messages::Point2::new(0.1, 0.0), angular_z: 0.0,
        });
        worker.step(0.3); // Unsupported selected recheck queues a new current-state computation.
        finish.send(command.clone()).unwrap();
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        assert!(worker.shared.0.lock().unwrap().result.is_none(),"unrechecked old-pose command was published");
        finish.send(command.clone()).unwrap();
        wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3), command, "current-pose first delivery remains available");
        worker.update_odometry(route_pose(0.500002, 2));
        assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()),
            "a consumed velocity cannot grant another hold from the newer pose");
        worker.stop();
        let _ = finish.send(PlannerStep::Pending(Vec::new()));
    }

    #[test]
    fn repeated_identical_invalid_pose_preserves_current_failure() {
        let (mut worker, started, finish) = terminal_worker();
        let mut invalid = route_pose(0.0, 0);
        invalid.pose.position.x = f64::NAN;
        worker.update_odometry(invalid.clone());
        worker.step(0.3);
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        let failed = PlannerStep::Failed("invalid pose".into());
        worker.update_odometry(invalid);
        finish.send(failed.clone()).unwrap();
        wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3), failed);
    }
    #[test]
    fn stored_terminal_results_require_current_applied_state_but_commands_do_not() {
        let (mut worker, started, finish) = terminal_worker();
        worker.step(0.3);
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        for outcome in [PlannerStep::Succeeded, PlannerStep::Failed("old applied".into()),
            PlannerStep::ReplanRequired("old applied".into()), PlannerStep::Command(Twist2::ZERO)] {
            {
                let mut input = worker.shared.0.lock().unwrap();
                input.result = Some((std::time::Instant::now(), outcome.clone()));
                input.result_applied_version = input.applied_version;
                input.applied_version += 1;
            }
            if matches!(outcome, PlannerStep::Command(_)) {
                worker.update_odometry(route_pose(0.500001, 1));
            }
            assert_eq!(worker.step(0.3), if matches!(outcome, PlannerStep::Command(_)) {
                outcome
            } else { PlannerStep::Pending(Vec::new()) });
        }
        worker.stop();
        finish.send(PlannerStep::Pending(Vec::new())).unwrap();
    }

    #[test]
    fn arrival_event_requalification_preserves_strict_region_and_lifecycle_fences() {
        // Keep the real compute thread gated while independently testing retrieval.
        let (mut worker, started, finish) = terminal_worker();
        worker.step(0.3);
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        let goal = route_pose(0.0, 0);
        let options = GoalOptions { position_tolerance_m: 0.25, yaw_tolerance_rad: 0.25, ..Default::default() };
        let path = ExecutionPath::Surface(Path3D { poses: vec![goal.clone(), goal.clone()] });
        for case in ["inside", "optional_yaw", "outside", "z_edge", "yaw_edge", "frame", "nan", "quaternion", "goal", "path", "path_goal", "map", "boundary", "applied", "pending"] {
            let options = GoalOptions { yaw_tolerance_rad: if case == "optional_yaw" {
                std::f64::consts::PI
            } else {
                nuwax_nav_astar::controllers::angle_diff(0.0, Quaternion::from_yaw(0.25).yaw()).abs()
            }, ..options };
            worker.configure_goal(&goal, options).unwrap();
            worker.replace_execution_path(path.clone(), None, if case == "path_goal" {route_pose(1.0, 0)} else {goal.clone()}).unwrap();
            worker.update_costmap(CostMap2D::new_all_free("map", 1, 1, 0.1));
            worker.update_odometry(goal.clone());
            {
                let mut input = worker.shared.0.lock().unwrap();
                input.result = Some((std::time::Instant::now(), if case == "pending" {PlannerStep::Pending(Vec::new())} else {PlannerStep::Succeeded}));
                input.result_pose_version = input.pose_version;
                input.result_applied_version = input.applied_version;
                input.result_map_version = input.map_version;
                input.result_boundary_version = input.boundary_version;
            }
            let mut latest = goal.clone();
            latest.pose.position.x += 0.000001;
            match case {
                "optional_yaw" => latest.pose.orientation = Quaternion::from_yaw(2.0),
                "outside" => latest.pose.position.x = 0.3,
                "z_edge" => { latest.pose.position.x = 0.0; latest.pose.position.z += 0.25; }
                "yaw_edge" => latest.pose.orientation = Quaternion::from_yaw(0.25),
                "frame" => latest.frame_id = NavFrameId::Odom,
                "nan" => latest.pose.position.y = f64::NAN,
                "quaternion" => latest.pose.orientation = Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 0.0 },
                "goal" => worker.configure_goal(&goal, options).unwrap(),
                "path" => worker.replace_execution_path(path.clone(), None, goal.clone()).unwrap(),
                "map" => worker.update_costmap(CostMap2D::new_all_free("map", 1, 1, 0.1)),
                "boundary" => worker.configure_boundary(&[]),
                "applied" => worker.observe_applied_command(Twist2 { linear: nuwax_nav_types_messages::Point2::new(0.1, 0.0), angular_z: 0.0 }),
                _ => {}
            }
            worker.update_odometry(latest);
            let result = worker.step(0.3);
            assert_eq!(result, if matches!(case, "inside" | "optional_yaw") { PlannerStep::Succeeded } else { PlannerStep::Pending(Vec::new()) }, "case {case}");
        }
        worker.stop();
        finish.send(PlannerStep::Pending(Vec::new())).unwrap();
    }

    #[test]
    fn real_computed_arrival_survives_in_tolerance_pose_jitter() {
        let mut harness = RouteWorkerHarness::new(false);
        harness.worker.step(0.3);
        harness.next_computation();
        harness.release.send(()).unwrap();
        wait_for_source_result(&harness.worker);
        harness.worker.update_odometry(route_pose(3.0, 1));
        harness.worker.observe_applied_command(Twist2::ZERO);
        let computed = harness.next_computation();
        assert_eq!(computed.outcome, PlannerStep::Succeeded);
        harness.worker.update_odometry(route_pose(3.000001, 2));
        harness.release.send(()).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        while !harness.worker.shared.0.lock().unwrap().result.as_ref()
            .is_some_and(|(_, step)| matches!(step, PlannerStep::Succeeded)) {
            assert!(std::time::Instant::now() < deadline, "computed success was not published");
            std::thread::yield_now();
        }
        assert_eq!(harness.worker.step(0.3), PlannerStep::Succeeded,
            "an already computed arrival must survive tiny in-tolerance stationary jitter");
    }

    #[test]
    fn arrival_is_rechecked_when_pose_changes_after_computation() {
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered: Arc::new(AtomicBool::new(false)),
            release: Arc::new(AtomicBool::new(true)),
        }).unwrap();
        let mut pose = nuwax_nav_astar::helpers::make_pose_stamped("map", 1.0, 0.0, 0.0);
        worker.update_odometry(pose.clone());
        {
            let mut input = worker.shared.0.lock().unwrap();
            input.result = Some((std::time::Instant::now(), PlannerStep::Succeeded));
            input.result_pose_version = input.pose_version;
        }
        pose.stamp.nanos += 50_000_000;
        worker.update_odometry(pose.clone());
        assert_eq!(worker.step(0.3), PlannerStep::Succeeded,
            "同位姿新时间戳不应阻断已确认的位置到达");
        pose.pose.position.x += 0.5;
        worker.update_odometry(pose);
        assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()),
            "移动后不能沿旧位置报告成功");
        assert!(worker.execution_wait_reason().unwrap().contains("到达状态"));
    }
    struct SlowPlanner {
        entered: Arc<AtomicBool>,
        release: Arc<AtomicBool>,
    }
    impl NavPlanner for SlowPlanner {
        fn replace_path(&mut self, _: Path2D) {}
        fn update_odometry(&mut self, _: PoseStamped) {}
        fn update_costmap(&mut self, _: CostMap2D) {}
        fn step(&mut self, _: f64) -> PlannerStep {
            self.entered.store(true, Ordering::SeqCst);
            while !self.release.load(Ordering::SeqCst) {
                std::thread::sleep(Duration::from_millis(1));
            }
            PlannerStep::Command(Twist2 {
                linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0),
                angular_z: 0.0,
            })
        }
        fn stop(&mut self) {}
    }
    #[test]
    fn autonomous_worker_delivers_forward_only_goal_options() {
        let entered=Arc::new(AtomicBool::new(false));
        let mut worker=PlannerWorker::new(SlowPlanner {
            entered:Arc::clone(&entered),release:Arc::new(AtomicBool::new(true)),
        }).unwrap();
        let goal=nuwax_nav_astar::helpers::make_pose_stamped("map",1.0,0.0,0.0);
        let options=nuwax_nav_types_messages::GoalOptions {
            allow_reverse:true,position_tolerance_m:0.23,yaw_tolerance_rad:0.7,
        };
        worker.configure_goal(&goal,options).unwrap();
        worker.step(0.3);
        let deadline=std::time::Instant::now()+Duration::from_secs(1);
        while worker.shared.0.lock().unwrap().result.is_none() && std::time::Instant::now()<deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        let input=worker.shared.0.lock().unwrap();
        assert_eq!(input.goal.as_ref().unwrap().1,nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options});
        assert!(matches!(input.result.as_ref().map(|r|&r.1),Some(PlannerStep::Command(v))if v.linear.x>0.0),
            "默认不支持倒退的后端必须收到收窄后的目标，不能误拒旧授权");
        assert!(entered.load(Ordering::SeqCst));
    }
    #[test]
    fn replacement_path_preserves_goal_and_applied_control_state() {
        use std::sync::atomic::AtomicU64;
        struct Probe {
            goals: Arc<AtomicU64>,
            stops: Arc<AtomicU64>,
            applied: Twist2,
        }
        impl NavPlanner for Probe {
            fn configure_goal(
                &mut self,
                _: &PoseStamped,
                _: nuwax_nav_types_messages::GoalOptions,
            ) -> Result<(), RuntimeError> {
                self.goals.fetch_add(1, Ordering::SeqCst);
                Ok(())
            }
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn observe_applied_command(&mut self, command: Twist2) {
                self.applied = command;
            }
            fn step(&mut self, _: f64) -> PlannerStep {
                PlannerStep::Command(self.applied)
            }
            fn stop(&mut self) {
                self.stops.fetch_add(1, Ordering::SeqCst);
                self.applied = Twist2::ZERO;
            }
        }
        let goals = Arc::new(AtomicU64::new(0));
        let stops = Arc::new(AtomicU64::new(0));
        let mut worker = PlannerWorker::new(Probe {
            goals: goals.clone(),
            stops: stops.clone(),
            applied: Twist2::ZERO,
        })
        .unwrap();
        let goal = nuwax_nav_astar::helpers::make_pose_stamped("map", 1.0, 0.0, 0.0);
        worker.configure_goal(&goal, Default::default()).unwrap();
        worker.update_costmap(CostMap2D::new_all_free("map", 10, 10, 0.1));
        let command = Twist2 {
            linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0),
            angular_z: 0.1,
        };
        worker.observe_applied_command(command);
        let route = ExecutionPath::Planar(Path2D::stamped(
            nuwax_nav_types_messages::NavFrameId::Map,
            Default::default(),
            vec![nuwax_nav_types_messages::Point2::new(1.0, 0.0)],
        ));
        for revision in 0..2 {
            worker
                .replace_execution_path(route.clone(), None, goal.clone())
                .unwrap();
            worker.step(0.3);
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            while worker.shared.0.lock().unwrap().result.is_none()
                && std::time::Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(1));
            }
            assert_eq!(
                worker.shared.0.lock().unwrap().result.as_ref().unwrap().1,
                PlannerStep::Command(command)
            );
            assert_eq!(
                goals.load(Ordering::SeqCst),
                1,
                "第{revision}次换路径不能重新配置目标"
            );
            assert_eq!(
                stops.load(Ordering::SeqCst),
                1,
                "换路径不能额外重置运动历史"
            );
        }
    }

    #[test]
    fn blocked_geometry_does_not_block_stop_and_late_velocity_is_retired() {
        let entered = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered: entered.clone(),
            release: release.clone(),
        })
        .unwrap();
        assert_eq!(worker.step(0.3), PlannerStep::Pending(Vec::new()));
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while !entered.load(Ordering::SeqCst) && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        let started = std::time::Instant::now();
        worker.stop();
        let elapsed = started.elapsed();
        release.store(true, Ordering::SeqCst);
        assert!(elapsed < Duration::from_millis(20), "取消等待了几何计算");
        std::thread::sleep(Duration::from_millis(20));
        assert!(
            worker.shared.0.lock().unwrap().result.is_none(),
            "取消后的计算不能恢复旧速度"
        );
    }
    #[test]
    fn pending_next_computation_preserves_continuous_velocity_only_inside_verified_horizon() {
        let entered = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(true));
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered,
            release: release.clone(),
        })
        .unwrap();
        worker.step(0.3);
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while worker.shared.0.lock().unwrap().result.is_none()
            && std::time::Instant::now() < deadline
        {
            std::thread::sleep(Duration::from_millis(1));
        }
        release.store(false, Ordering::SeqCst);
        let result = worker.step(0.3);
        assert!(matches!(result,PlannerStep::Command(t) if t.linear.x>0.0));
        if let PlannerStep::Command(command) = result {
            worker.observe_applied_command(command);
        }
        std::thread::sleep(DEFAULT_CONTROL_HORIZON + Duration::from_millis(20));
        let stale = worker.step(0.3);
        release.store(true, Ordering::SeqCst);
        assert_eq!(
            stale,
            PlannerStep::Pending(Vec::new()),
            "不能无限重放旧速度"
        );
    }
    #[test]
    fn lifecycle_stop_reaches_inner_owner_without_another_control_tick() {
        use crate::planning_owner::PlanningOwner;
        struct Probe {
            owner: PlanningOwner<(),()>,
            stopped: std::sync::mpsc::Sender<()>,
            steps: Arc<std::sync::atomic::AtomicUsize>,
        }
        impl NavPlanner for Probe {
            fn replace_path(&mut self,_:Path2D) {}
            fn update_odometry(&mut self,_:PoseStamped) {}
            fn update_costmap(&mut self,_:CostMap2D) {}
            fn step(&mut self,_:f64)->PlannerStep {
                self.steps.fetch_add(1,Ordering::SeqCst);self.owner.submit(1,());PlannerStep::Pending(Vec::new())
            }
            fn stop(&mut self) {self.owner.cancel();let _=self.stopped.send(());}
        }
        let (started,starts)=std::sync::mpsc::channel();
        let (cancelled,cancels)=std::sync::mpsc::channel();
        let (stopped,stops)=std::sync::mpsc::channel();
        let steps=Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let owner=PlanningOwner::new(move |_,token:&crate::planning_owner::Cancellation| {
            started.send(()).unwrap();
            let deadline=std::time::Instant::now()+Duration::from_secs(3);
            while !token.cancelled() && std::time::Instant::now()<deadline {std::thread::sleep(Duration::from_millis(1));}
            assert!(token.cancelled());cancelled.send(()).unwrap();
        }).unwrap();
        let mut worker=PlannerWorker::new(Probe{owner,stopped,steps:steps.clone()}).unwrap();
        worker.step(0.3);starts.recv_timeout(Duration::from_secs(1)).unwrap();
        while stops.try_recv().is_ok() {}
        worker.stop();
        stops.recv_timeout(Duration::from_secs(1)).unwrap();
        cancels.recv_timeout(Duration::from_secs(1)).unwrap();
        worker.update_costmap(CostMap2D::new_all_free("map",2,2,0.1));
        worker.configure_boundary(&[[0.0,0.0],[1.0,0.0],[0.0,1.0]]);
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(steps.load(Ordering::SeqCst),1,"source updates cannot restart stopped control");
        assert!(worker.shared.0.lock().unwrap().result.is_none());
    }

    #[test]
    fn positive_result_is_consumed_once_and_fresh_same_value_requires_new_compute() {
        let (mut worker, started, release)=source_worker();
        assert_eq!(worker.step(0.3),PlannerStep::Pending(Vec::new()));
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        release.send(()).unwrap();wait_for_source_result(&worker);
        let first=worker.step(0.3);
        assert!(matches!(&first,PlannerStep::Command(command) if *command!=Twist2::ZERO));
        assert!(worker.delivered_command_deadline_for_test().is_some());
        assert!(worker.shared.0.lock().unwrap().result.is_none());
        // A retrieval cannot rename the old publication as a new computation.
        assert_eq!(worker.step(0.3),PlannerStep::Pending(Vec::new()));
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        assert_eq!(worker.step(0.3),PlannerStep::Pending(Vec::new()));
        release.send(()).unwrap();wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3),first,"same numeric command is allowed after real computation");
        worker.stop();
        assert!(worker.shared.0.lock().unwrap().result.is_none());
        // A coalesced request may already have entered the gated test planner;
        // release it after stop so its late result must retire without a long join.
        let _=release.send(());
    }

    /// Diagnostic falsifier: static evidence and a held next publication isolate
    /// command replay from source construction rate and from planner CPU work.
    fn run_replay_proof_falsifier(initial_speed:f64) {
        use nuwax_nav_mls_planner::{SurfaceObservation,SurfaceObservations};
        use nuwax_nav_types_messages::{MotionLimits,Point2};
        use nuwax_nav_control::PlanarPose;
        let unix_ns=||std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
        let stamped=|x:f64| {let mut p=route_pose(x,0);let ns=unix_ns();p.stamp=Timestamp::from_unix((ns/1_000_000_000)as i64,(ns%1_000_000_000)as u32);p};
        let limits=MotionLimits{model:MotionModel::Differential,max_forward_mps:0.5,max_reverse_mps:0.0,
            max_lateral_mps:0.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
            angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true};
        let source=crate::surface_follower::test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope{
            footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0}).unwrap();
        let sampled=unix_ns();
        let mut blocks=std::collections::BTreeMap::<_,std::collections::BTreeMap<_,_>>::new();
        for (x,y,z) in source.cell_keys(){let key=[x,y,z];blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
            SurfaceObservation{observed_ns:if x>=7{sampled.saturating_sub(2_000_000_000)}else{sampled},hits:2});}
        let mut observations=SurfaceObservations::new(source.source_voxel_size_m(),32).unwrap();
        for (key,block) in blocks{observations.replace_block(key,block).unwrap();}
        observations.mark_complete_geometry_source();
        let source=Arc::new(source.with_observations(Some(Arc::new(observations)))
            .with_recent_obstacle_points(1,sampled,0,&[],[(0,[0.75,1.0,0.5])]).unwrap());
        let mut config=SurfaceFollowerConfig::new(0.4);config.local_planning_enabled=false;
        config.controller.control_frequency=20.0;config.controller.speed=0.5;
        config.motion_limits=Some(limits.clone());config.max_observation_age_sec=Some(1.0);
        assert!(!config.controller.simulation);
        let (computed,receive)=std::sync::mpsc::channel();let (release,wait)=std::sync::mpsc::channel();
        let mut harness=RouteWorkerHarness{worker:PlannerWorker::new(GatedRouteFollower{
            inner:RouteFollower::new(config).unwrap(),computed,release:wait,map:None,boundary:Vec::new(),
            odometry:None,goal_updates:0,path_updates:0}).unwrap(),computed:receive,release,clear:source.clone()};
        let start=stamped(0.533);let goal=stamped(2.0);
        source.validate_body_pose([0.533,1.0,0.5],[0.0,0.0,0.0,1.0]).unwrap();
        assert!(source.validate_body_pose([0.56,1.0,0.5],[0.0,0.0,0.0,1.0]).is_err(),
            "real forward obstacle must separate initial clearance from unsafe continued travel");
        let previous=Twist2{linear:Point2::new(initial_speed,0.0),angular_z:0.0};
        harness.worker.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
        harness.worker.configure_goal(&goal,GoalOptions::default()).unwrap();
        harness.worker.update_odometry(start.clone());
        harness.worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![start.clone(),goal.clone()]}),None,goal).unwrap();
        harness.worker.observe_applied_command(previous);harness.worker.step(0.3);
        let initial=harness.next_computation();
        let PlannerStep::Command(first)=initial.outcome else{panic!("initial guarded command missing: {:?}",initial.outcome);};
        assert!(first.linear.x>=0.09 && first.angular_z.abs()<1e-9,"fixture must establish the maximal safe initial command: {first:?}");
        harness.release.send(()).unwrap();wait_for_source_result(&harness.worker);
        assert_eq!(harness.worker.step(0.3),PlannerStep::Command(first));
        let first_deadline=harness.worker.delivered_command_deadline_for_test().unwrap();
        // Actual calibrated midpoint transition over one real wall-clock period.
        std::thread::sleep(Duration::from_millis(50));
        let current=stamped(start.pose.position.x+(previous.linear.x+first.linear.x)*0.5*0.05);
        harness.worker.update_odometry(current.clone());harness.worker.observe_applied_command(first);
        let latest=harness.next_computation(); // hold the new guard outcome before publication
        let replay=harness.worker.step(0.3);
        let remaining=first_deadline.saturating_duration_since(std::time::Instant::now());
        eprintln!("REPLAY before={previous:?} initial={first:?} fresh_computed={:?} delivered={replay:?} x={} remaining_ms={}",latest.outcome,current.pose.position.x,remaining.as_secs_f64()*1000.0);
        assert!(remaining>Duration::from_millis(50),"fresh 50ms oracle must fit inside the selected original remaining lease");
        let command=match replay {
            PlannerStep::Command(command)=>command,
            PlannerStep::Pending(_)|PlannerStep::ReplanRequired(_)=>{
                // Correct retirement is also a successful desired-safety outcome.
                harness.worker.stop();harness.release.send(()).unwrap();return;
            }
            other=>panic!("unexpected replay diagnostic outcome: {other:?}"),
        };
        let pose=PlanarPose{x:current.pose.position.x,y:1.0,yaw:0.0};
        let body=|p:PlanarPose|([p.x,p.y,0.5],[0.0,0.0,0.0,1.0]);
        let view=source.with_current_body_support(body(pose).0,body(pose).1).unwrap();
        assert!(view.same_body_query_view(&source),"flat source must need no different current-body permission");
        let failure=std::cell::RefCell::new(None);
        let check=|a:PlanarPose,b:PlanarPose,_elapsed:f64| {
            let projected=view.body_pose_after_motion(body(a),[b.x,b.y],b.yaw).unwrap();
            assert!((projected.0[2]-0.5).abs()<1e-9,"oracle/source projection must agree");
            match view.validate_body_motion(body(a),projected,&[]) {
                Ok(())=>true,Err(e)=>{if failure.borrow().is_none(){*failure.borrow_mut()=Some(e.to_string());}false}
            }
        };
        let safe=nuwax_nav_control::check_stopping_trajectory_with_period_check(pose,first,command,&limits,0.05,
            f64::from(source.source_voxel_size_m())*0.5,source.body_envelope().unwrap().radius(),check,check).is_some();
        eprintln!("REPLAY oracle_safe={safe} reason={:?}",failure.borrow());
        harness.worker.stop();harness.release.send(()).unwrap();
        assert!(safe,"a retained command delivered inside its lease still needs current full braking permission");
    }

    #[test]
    #[ignore="real-clock geometry regression for single-consumption containment"]
    fn retained_command_replay_requires_current_full_braking_permission() {run_replay_proof_falsifier(0.05);}
    #[test]
    #[ignore="real-clock geometry regression with unchanged actual applied velocity"]
    fn retained_steady_command_replay_requires_current_full_braking_permission() {run_replay_proof_falsifier(0.1);}

    struct DispatchProbe {
        started:std::sync::mpsc::Sender<()>,
        finish:std::sync::mpsc::Receiver<(PlannerStep,Option<CommandDispatchBound>)>,
        bound:Option<CommandDispatchBound>,
    }
    impl NavPlanner for DispatchProbe {
        fn replace_path(&mut self,_:Path2D) {}
        fn update_odometry(&mut self,_:PoseStamped) {}
        fn update_costmap(&mut self,_:CostMap2D) {}
        fn step(&mut self,_:f64)->PlannerStep {
            self.bound=None;let _=self.started.send(());
            let (step,bound)=self.finish.recv_timeout(Duration::from_secs(2)).unwrap_or((PlannerStep::Pending(Vec::new()),None));
            self.bound=bound;step
        }
        fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {self.bound.take()}
        fn stop(&mut self) {self.bound=None;}
    }
    fn dispatch_worker()->(PlannerWorker,std::sync::mpsc::Receiver<()>,std::sync::mpsc::Sender<(PlannerStep,Option<CommandDispatchBound>)>) {
        let (started,receive)=std::sync::mpsc::channel();let (finish,wait)=std::sync::mpsc::channel();
        (PlannerWorker::new(DispatchProbe{started,finish:wait,bound:None}).unwrap(),receive,finish)
    }
    fn dispatch_test_bound(remaining_ns:u64)->CommandDispatchBound {
        let anchor=crate::command_dispatch::ObservationClockAnchor::capture();
        CommandDispatchBound::from_anchor(anchor,anchor.unix_ns+remaining_ns)
    }
    #[test]
    fn dispatch_expiry_during_compute_becomes_pending_without_route_penalty() {
        let (mut worker,started,finish)=dispatch_worker();
        let command=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.05,0.0),angular_z:0.0};
        assert!(matches!(worker.step(0.3),PlannerStep::Pending(_)));
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        let bound=dispatch_test_bound(0);
        assert!(bound.expired_now());
        finish.send((PlannerStep::Command(command),Some(bound))).unwrap();
        wait_for_source_result(&worker);
        assert!(matches!(worker.shared.0.lock().unwrap().result.as_ref().unwrap().1,PlannerStep::Pending(_)));
        assert!(matches!(worker.step(0.3),PlannerStep::Pending(_)));
        assert!(worker.execution_wait_reason().unwrap().contains("发布前"));
        assert!(worker.take_command_dispatch_bound().is_none());
        // A genuinely new computation with the same numeric command can recover.
        started.recv_timeout(Duration::from_secs(1)).unwrap();
        {worker.shared.0.lock().unwrap().result=None;}
        let fresh=dispatch_test_bound(1_000_000_000);
        finish.send((PlannerStep::Command(command),Some(fresh))).unwrap();
        wait_for_source_result(&worker);
        assert_eq!(worker.step(0.3),PlannerStep::Command(command));
        assert_eq!(worker.take_command_dispatch_bound(),Some(fresh));
        assert!(worker.take_command_dispatch_bound().is_none());
        worker.stop();let _=finish.send((PlannerStep::Pending(Vec::new()),None));
    }
    #[test]
    fn dispatch_bound_rejects_delayed_first_retrieval_without_extending_hold_lease() {
        let (mut worker,started,finish)=dispatch_worker();
        let command=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.05,0.0),angular_z:0.0};
        worker.step(0.3);started.recv_timeout(Duration::from_secs(1)).unwrap();
        let fresh=dispatch_test_bound(1_000_000_000);
        finish.send((PlannerStep::Command(command),Some(fresh))).unwrap();wait_for_source_result(&worker);
        {
            let mut input=worker.shared.0.lock().unwrap();
            assert!(matches!(input.result.as_ref().unwrap().1,PlannerStep::Command(_)));
            input.result_dispatch_bound=Some(dispatch_test_bound(0));
            assert!(input.result.as_ref().unwrap().0.elapsed()<DEFAULT_CONTROL_HORIZON);
        }
        assert!(matches!(worker.step(0.3),PlannerStep::Pending(_)));
        assert!(worker.execution_wait_reason().unwrap().contains("交付前"));
        assert!(worker.take_command_dispatch_bound().is_none());
        worker.stop();let _=finish.send((PlannerStep::Pending(Vec::new()),None));
    }
    #[test]
    fn dispatch_taken_result_metadata_is_retired_by_input_and_lifecycle_changes() {
        let command=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.05,0.0),angular_z:0.0};
        for change in 0..7 {
            let (mut worker,started,finish)=dispatch_worker();
            worker.step(0.3);started.recv_timeout(Duration::from_secs(1)).unwrap();
            finish.send((PlannerStep::Command(command),Some(dispatch_test_bound(1_000_000_000)))).unwrap();
            wait_for_source_result(&worker);assert_eq!(worker.step(0.3),PlannerStep::Command(command));
            assert!(worker.delivered_dispatch_bound.is_some());
            let goal=route_pose(2.0,0);
            match change {
                0=>worker.configure_goal(&goal,GoalOptions::default()).unwrap(),
                1=>worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![route_pose(0.5,0),goal.clone()]}),None,goal).unwrap(),
                2=>worker.configure_boundary(&[[0.0,0.0],[3.0,0.0],[3.0,3.0],[0.0,3.0]]),
                3=>worker.stop(),
                4=>worker.update_costmap(CostMap2D::new_all_free("map",2,2,0.1)),
                5=>worker.update_odometry(route_pose(0.51,1)),
                _=>worker.observe_applied_command(command),
            }
            assert!(worker.take_command_dispatch_bound().is_none(),"stale taken metadata survived change {change}");
            worker.stop();let _=finish.send((PlannerStep::Pending(Vec::new()),None));
        }
    }

}
