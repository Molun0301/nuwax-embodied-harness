//! 仅处理近场样条受阻的前向姿态接续；不承担长距离全局规划。
use super::{angle, yaw, BodyPose, LocalPlanError};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_pct::Tomogram;
use nuwax_nav_types_messages::MotionKinematics;
use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};
// Tests can reduce the limit but execute the identical production exhaustion branch.
#[cfg(test)]
thread_local! { static EXPANSION_LIMIT: std::cell::Cell<usize> = const { std::cell::Cell::new(256) }; }
#[cfg(test)]
pub(super) fn with_expansion_limit<T>(limit: usize, f: impl FnOnce() -> T) -> T {
    struct Reset(usize);
    impl Drop for Reset { fn drop(&mut self) { EXPANSION_LIMIT.set(self.0); } }
    let _reset = Reset(EXPANSION_LIMIT.replace(limit));
    f()
}
// Production uses the caller's configured bound; tests may substitute a smaller
// one so the identical exhaustion branch stays reachable without huge fixtures.
#[cfg(not(test))]
fn effective_expansion_limit(configured: usize) -> usize { configured }
#[cfg(test)]
fn effective_expansion_limit(_configured: usize) -> usize { EXPANSION_LIMIT.get() }

// 队列排序对照实验。只影响排队优先级，不改变真实碰撞、支撑与运动许可。
#[cfg(test)]
#[derive(Clone, Copy, Default, PartialEq)]
pub(super) enum HeuristicMode {
    /// 乘积形式下界：max(行程时间, 转向时间)。
    #[default]
    AdmissibleMax,
    /// 原始加权和，用于保持既有诊断可复现。
    ScaledSum(f64),
    /// 不加权的和，仍是下界但偏保守。
    AdmissibleSum,
}
#[cfg(test)]
thread_local! { static HEURISTIC_MODE: std::cell::Cell<HeuristicMode> = const { std::cell::Cell::new(HeuristicMode::AdmissibleMax) }; }
#[cfg(test)]
pub(super) fn with_heuristic_mode<T>(mode: HeuristicMode, f: impl FnOnce() -> T) -> T {
    struct Reset(HeuristicMode);
    impl Drop for Reset { fn drop(&mut self) { HEURISTIC_MODE.set(self.0); } }
    let _reset = Reset(HEURISTIC_MODE.replace(mode));
    f()
}


// Synthetic queue-order experiments only. Production builds and default tests do
// not select these modes; costs, admission, actions and termination are unchanged.
#[cfg(test)]
#[derive(Clone, Default)]
pub(super) enum DiagnosticPriority {
    #[default]
    Baseline,
    RelaxedGoal(f64),
    OracleGuide(Vec<[f64; 3]>),
}
#[cfg(test)]
thread_local! { static DIAGNOSTIC_PRIORITY: std::cell::RefCell<DiagnosticPriority> = const { std::cell::RefCell::new(DiagnosticPriority::Baseline) }; }
#[cfg(test)]
pub(super) fn with_diagnostic_priority<T>(mode: DiagnosticPriority, f: impl FnOnce() -> T) -> T {
    struct Reset(DiagnosticPriority);
    impl Drop for Reset { fn drop(&mut self) { DIAGNOSTIC_PRIORITY.with(|v| { v.replace(self.0.clone()); }); } }
    let _reset = Reset(DIAGNOSTIC_PRIORITY.with(|v| v.replace(mode)));
    f()
}

#[derive(Default, Debug)]
struct SearchWork {
    expanded: usize,
    candidates: usize,
    // Admission sweeps only; analytic connectors and rotations are not counted.
    sweeps: usize,
    nodes: usize,
    frontier_peak: usize,
}
impl Drop for SearchWork {
    fn drop(&mut self) {
        tracing::debug!(target: "navigation.scan.search", expanded=self.expanded,
            candidates=self.candidates, sweeps=self.sweeps, nodes=self.nodes,
            frontier_peak=self.frontier_peak, "bounded local search work");
        #[cfg(test)]
        {
            eprintln!("search_work {self:?}");
            LAST_WORK.set((self.expanded,self.nodes,self.frontier_peak));
        }
    }
}
#[cfg(test)]
thread_local! { static LAST_WORK: std::cell::Cell<(usize,usize,usize)> = const { std::cell::Cell::new((0,0,0)) }; }
#[cfg(test)]
pub(super) fn last_work() -> (usize,usize,usize) { LAST_WORK.get() }
#[cfg(test)]
pub(super) fn reset_last_work() { LAST_WORK.set((0,0,0)); }
#[cfg(test)]
thread_local! { static INITIAL_ADMISSIONS: std::cell::RefCell<Vec<(f64,f64,BodyPose)>> = const { std::cell::RefCell::new(Vec::new()) }; }
#[cfg(test)]
pub(super) fn initial_admissions() -> Vec<(f64,f64,BodyPose)> {
    INITIAL_ADMISSIONS.with(|v| v.borrow().clone())
}

type ProjectedMotion=(Vec<BodyPose>,Vec<nuwax_nav_mls_planner::SurfaceBodyQueryView>);
struct Node {
    pose: BodyPose,
    cost: f64,
    parent: Option<usize>,
    samples: Vec<BodyPose>,
}
#[derive(PartialEq)]
struct Entry(f64, usize);
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Ord for Entry {
    fn cmp(&self, b: &Self) -> Ordering {
        b.0.total_cmp(&self.0).then_with(|| b.1.cmp(&self.1))
    }
}
// Identity is the complete input pose, not a voxel/yaw equivalence class.
// Keep signed zeros and quaternion antipodes distinct; do not canonicalize inputs.
fn key(p: BodyPose) -> [u64; 7] {
    [p.0[0].to_bits(), p.0[1].to_bits(), p.0[2].to_bits(),
     p.1[0].to_bits(), p.1[1].to_bits(), p.1[2].to_bits(), p.1[3].to_bits()]
}

// Queue labels use exactly the same dominance relation as admission. A cheaper
// path to this exact seven-component pose supersedes the old entry even when
// the cost difference is smaller than a geometric tolerance.
fn is_stale(node: &Node, best: &HashMap<[u64; 7], f64>) -> bool {
    best.get(&key(node.pose)).is_some_and(|&cost| node.cost > cost)
}

pub fn probe_motion(
    map: &SurfaceSnapshot,
    from: BodyPose,
    length: f64,
    turn: f64,
    boundary: &[[f64; 2]],
) -> Option<Vec<BodyPose>> {
    probe_motion_until(map,from,length,turn,boundary,|_|false).ok()
}
/// Diagnostic only: run one production primitive through the same swept-body
/// admission the search uses, so a stall can be attributed to admission rather
/// than to queue ordering. Never grants clearance to search.
pub fn probe_swept_admission(
    map: &SurfaceSnapshot,
    from: BodyPose,
    length: f64,
    turn: f64,
    boundary: &[[f64; 2]],
) -> Result<Vec<BodyPose>, LocalPlanError> {
    let (poses,views)=project_motion_until(map,from,length,turn,|_|false)?;
    validate_projected(from,&poses,&views,boundary)?;
    Ok(poses)
}
fn probe_motion_until(
    map: &SurfaceSnapshot,
    from: BodyPose,
    length: f64,
    turn: f64,
    boundary: &[[f64;2]],
    reached: impl Fn(BodyPose)->bool,
) -> Result<Vec<BodyPose>, LocalPlanError> {
    let (poses,views)=project_motion_until(map,from,length,turn,reached)?;
    validate_projected(from,&poses,&views,boundary)?;
    Ok(poses)
}
fn project_motion_until(
    map:&SurfaceSnapshot,from:BodyPose,length:f64,turn:f64,reached:impl Fn(BodyPose)->bool,
) -> Result<ProjectedMotion, LocalPlanError> {
    super::cancellation::checkpoint()?;
    let radius = map.body_envelope().ok_or("近场搜索缺少机体")?.radius();
    let n = ((length + turn.abs() * radius) / 0.025).ceil().max(1.0) as usize;
    let heading = yaw(from.1);
    let mut previous = from;
    let mut poses = Vec::with_capacity(n);
    let mut views = Vec::with_capacity(n);
    for i in 1..=n {
        super::cancellation::checkpoint()?;
        if reached(previous) {break;}
        let t = i as f64 / n as f64;
        let theta = turn * t;
        let (dx, dy) = if turn.abs() < 1e-8 {
            (length * t, 0.0)
        } else {
            (
                length / turn * theta.sin(),
                length / turn * (1.0 - theta.cos()),
            )
        };
        let xy = [
            from.0[0] + dx * heading.cos() - dy * heading.sin(),
            from.0[1] + dx * heading.sin() + dy * heading.cos(),
        ];
        let (next,view)=super::project_endpoint(map,previous,xy,heading+theta)?;
        previous=next;
        poses.push(previous);
        views.push(view);
    }
    // 端点几何已拒绝的候选不再支付沿途扫掠；任何返回的候选仍逐段核验全部运动。
    // 保留生成端点时的同一只读查询视图，不把尚未验证的未来站姿写回地图。
    Ok((poses,views))
}
fn validate_projected(
    from:BodyPose,poses:&[BodyPose],views:&[nuwax_nav_mls_planner::SurfaceBodyQueryView],boundary:&[[f64;2]],
)->Result<(), LocalPlanError> {
    let mut previous=from;
    for (&next,view) in poses.iter().zip(views) {
        super::cancellation::checkpoint()?;
        super::check_projected_motion(view,previous,next,boundary)?;
        previous=next;
    }
    Ok(())
}
fn reference_distance(p: [f64; 3], reference: &[[f64; 3]]) -> f64 {
    reference
        .windows(2)
        .map(|pair| {
            let a = pair[0];
            let b = pair[1];
            let dx = b[0] - a[0];
            let dy = b[1] - a[1];
            let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy).max(1e-9))
                .clamp(0.0, 1.0);
            (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
        })
        .fold(reference.first().map_or(0.0, |a| (p[0]-a[0]).hypot(p[1]-a[1])), f64::min)
}
fn reference_progress(p: [f64; 3], reference: &[[f64; 3]]) -> (f64, f64, f64) {
    let mut best = (super::distance3(p,reference[0]), 0.0, reference[0][2]);
    let mut along = 0.0;
    for pair in reference.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let length = dx.hypot(dy);
        if length < 1e-9 {
            continue;
        }
        let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (length * length)).clamp(0.0, 1.0);
        let z = a[2] + t * (b[2] - a[2]);
        let distance = (p[0] - a[0] - t * dx)
            .hypot(p[1] - a[1] - t * dy)
            .hypot(p[2] - z);
        if distance < best.0 {
            best = (distance, along + t * length, z);
        }
        along += length;
    }
    best
}
#[derive(Clone, Copy)]
struct ReferenceTurn {
    heading: Option<f64>,
    suffix: f64,
    next: Option<usize>,
}
// One plan-local preparation; the guide's original >1e-9 heading rule is unchanged.
/// 所有偏离原末点的候选共用同一窗口交接条件，不能只交付受阻前缀。
pub(super) fn reference_handoff(p: BodyPose, reference: &[[f64; 3]], support_offset: f64,
    corridor: f64, layer_tolerance: f64, reference_length: f64, tangent: f64) -> bool {
    let Some(target) = reference.last() else { return false; };
    let support = [p.0[0], p.0[1], p.0[2] - support_offset];
    let (deviation, progress, height) = reference_progress(support, reference);
    if (p.0[0]-target[0]).hypot(p.0[1]-target[1]) > corridor || deviation > corridor
        || progress < reference_length - 1e-6 || (support[2]-height).abs() > layer_tolerance { return false; }
    angle(yaw(p.1)-tangent).cos()>0.0
}

fn reference_turns(reference: &[[f64; 3]]) -> Result<Vec<ReferenceTurn>, LocalPlanError> {
    let mut turns = Vec::with_capacity(reference.len().saturating_sub(1));
    for pair in reference.windows(2) {
        super::cancellation::checkpoint()?;
        let dx = pair[1][0] - pair[0][0];
        let dy = pair[1][1] - pair[0][1];
        turns.push(ReferenceTurn {
            heading: if dx.hypot(dy) > 1e-9 { Some(dy.atan2(dx)) } else { None },
            suffix: 0.0, next: None,
        });
    }
    let mut next: Option<usize> = None;
    for index in (0..turns.len()).rev() {
        super::cancellation::checkpoint()?;
        turns[index].next = next;
        if let Some(heading) = turns[index].heading {
            if let Some(next) = next {
                turns[index].suffix = angle(heading - turns[next].heading.unwrap()).abs() + turns[next].suffix;
            }
            next = Some(index);
        }
    }
    Ok(turns)
}
#[derive(Clone, Copy)]
struct ReferenceGuide {
    point: [f64; 3],
    segment: Option<usize>,
    at_vertex: bool,
    matched: bool,
}
fn reference_guide(reference: &[[f64; 3]], lookahead: f64, reference_length: f64) -> ReferenceGuide {
    let mut guide = ReferenceGuide { point: *reference.last().unwrap(), segment: None, at_vertex: false, matched: false };
    let mut along = 0.0;
    for (index, pair) in reference.windows(2).enumerate() {
        let length = (pair[1][0]-pair[0][0]).hypot(pair[1][1]-pair[0][1]);
        if along+length >= lookahead && length > 1e-9 {
            let t = (lookahead-along)/length;
            guide.point = std::array::from_fn(|i| pair[0][i]+t*(pair[1][i]-pair[0][i]));
            // A terminal guide has no reference continuation or invented goal heading.
            guide.segment = if lookahead < reference_length { Some(index) } else { None };
            guide.at_vertex = t >= 1.0;
            guide.matched = true;
            break;
        }
        along += length;
    }
    guide
}
// Experimental angular accounting changes ordering, never body admission.
fn reference_guide_turn(position: [f64; 2], heading: f64, guide: ReferenceGuide, turns: &[ReferenceTurn]) -> f64 {
    let dx = guide.point[0]-position[0];
    let dy = guide.point[1]-position[1];
    let coincident = dx.hypot(dy) < 1e-6;
    let segment = if coincident && guide.at_vertex {
        guide.segment.and_then(|index| turns[index].next)
    } else { guide.segment };
    let continuation = segment.and_then(|index| turns[index].heading.map(|theta| (theta,turns[index].suffix)));
    // Collapse an absent approach leg; never assign atan2(0,0) an eastward bearing.
    if coincident {
        return continuation.map_or(0.0, |(theta,suffix)| angle(heading-theta).abs()+suffix);
    }
    let bearing = dy.atan2(dx);
    angle(heading-bearing).abs()
        + continuation.map_or(0.0, |(theta,suffix)| angle(bearing-theta).abs()+suffix)
}
fn heading_potential(p: BodyPose, target: [f64;3], tangent: f64, corridor: f64,
    arrival: bool, guide: Option<ReferenceGuide>, turns: &[ReferenceTurn]) -> f64 {
    if let Some(guide) = guide {
        if guide.matched || arrival {
            return reference_guide_turn([p.0[0],p.0[1]],yaw(p.1),guide,turns);
        }
    }
    // Preserve the original terminal cross-section fallback when no guide was found.
    let deficit=|xy:[f64;2]| {
        let dx=xy[0]-p.0[0];let dy=xy[1]-p.0[1];
        if dx.hypot(dy)<1e-6 {0.0} else {
            let error=angle(dy.atan2(dx)-yaw(p.1)).abs();
            if guide.is_some() {error} else {(error-std::f64::consts::FRAC_PI_2).max(0.0)}
        }
    };
    if arrival {
        deficit([target[0],target[1]])
    } else {
        [0.0,corridor*0.5,-corridor*0.5,corridor,-corridor].into_iter().map(|offset| {
            deficit([target[0]-tangent.sin()*offset,target[1]+tangent.cos()*offset])
        }).fold(f64::INFINITY,f64::min)
    }
}
pub(super) fn validate_caps(motion: &MotionKinematics) -> Result<(), LocalPlanError> {
    if !motion.max_forward_mps.is_finite() || motion.max_forward_mps<=0.0
        || !motion.max_angular_rps.is_finite() || motion.max_angular_rps<=0.0 {
        return Err("近场搜索需要有限正值线速和角速上限".into());
    }
    Ok(())
}
// Additive weighted nominal seconds. Translation and turning happen together;
// terrain/reference preferences accrue only with translation, never per action.
fn step_cost(distance: f64, turn: f64, preference: f64, motion: &MotionKinematics) -> f64 {
    (distance / motion.max_forward_mps).max(turn.abs() / motion.max_angular_rps)
        + distance * preference / motion.max_forward_mps
}
fn preference(tomogram: &Tomogram, support: [f64;3], reference: &[[f64;3]]) -> f64 {
    let terrain=tomogram.cell_at(support).map_or(2.0, |id| tomogram.cells[id].cost);
    0.15 * terrain.max(0.0) + 0.1 * reference_distance(support,reference)
}
fn projected_cost(from: BodyPose, samples: &[BodyPose], offset: f64,
    tomogram: &Tomogram, reference: &[[f64;3]], motion: &MotionKinematics) -> f64 {
    let soft=|p:BodyPose|preference(tomogram,[p.0[0],p.0[1],p.0[2]-offset],reference);
    let mut previous=from; let mut cost=0.0;
    let mut previous_soft=None;
    for &next in samples {
        if super::cancellation::checkpoint().is_err() { return f64::INFINITY; }
        let distance=super::distance3(previous.0,next.0);
        let weight=if distance==0.0 {
            previous_soft=None;
            0.0
        } else {
            let left=previous_soft.unwrap_or_else(||soft(previous));
            let right=soft(next);
            previous_soft=Some(right);
            (left+right)*0.5
        };
        cost+=step_cost(distance,angle(yaw(next.1)-yaw(previous.1)),weight,motion);
        previous=next;
    }
    cost
}
// This is a bounded soft-cost quadrature, NOT geometry or collision sampling.
// It ranks nominal connectors, including sequential initial/final rotations.
// Actual projection, arrival truncation and alternate swept turns may cost more/less.
fn soft_intervals(length: f64) -> usize { (length/0.05).ceil().clamp(1.0,64.0) as usize }
fn connector_cost(from: BodyPose, heading: f64, length: f64, turn: f64,
    final_yaw: Option<f64>, target_z: f64, offset: f64, tomogram: &Tomogram,
    reference: &[[f64;3]], motion: &MotionKinematics) -> f64 {
    let n=soft_intervals(length);
    let mut previous=[from.0[0],from.0[1],from.0[2]-offset];
    let mut weight=preference(tomogram,previous,reference);
    let mut cost=step_cost(0.0,angle(heading-yaw(from.1)),0.0,motion);
    for i in 1..=n {
        if super::cancellation::checkpoint().is_err() { return f64::INFINITY; }
        let t=i as f64/n as f64; let theta=turn*t;
        let (dx,dy)=if turn.abs()<1e-8 {(length*t,0.0)}
            else {(length/turn*theta.sin(),length/turn*(1.0-theta.cos()))};
        let next=[from.0[0]+dx*heading.cos()-dy*heading.sin(),
            from.0[1]+dx*heading.sin()+dy*heading.cos(),
            (from.0[2]-offset)*(1.0-t)+target_z*t];
        let next_weight=preference(tomogram,next,reference);
        // Use exact nominal arc length, not quadrature chords.
        let distance=(length/n as f64).hypot(next[2]-previous[2]);
        cost+=step_cost(distance,turn/n as f64,if distance==0.0 {0.0}else{(weight+next_weight)*0.5},motion);
        previous=next; weight=next_weight;
    }
    cost+step_cost(0.0,final_yaw.map_or(0.0,|y|angle(y-heading-turn)),0.0,motion)
}
pub fn plan(
    map: &SurfaceSnapshot,
    tomogram: &Tomogram,
    start: BodyPose,
    reference: &[[f64; 3]],
    motion: &MotionKinematics,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    boundary: &[[f64; 2]],
    horizon: f64,
    corridor: f64,
    follow_reference: bool,
    expansion_limit: usize,
) -> Result<Vec<BodyPose>, LocalPlanError> {
    plan_progress(map,tomogram,start,reference,motion,final_yaw,arrival,boundary,horizon,corridor,
        follow_reference,expansion_limit,None).map(|path|path.poses)
}
#[derive(Debug)]
pub(super) struct SearchPath {
    pub poses: Vec<BodyPose>,
    pub partial: bool,
}
pub(super) fn plan_progress(
    map: &SurfaceSnapshot,
    tomogram: &Tomogram,
    start: BodyPose,
    reference: &[[f64; 3]],
    motion: &MotionKinematics,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    boundary: &[[f64; 2]],
    horizon: f64,
    corridor: f64,
    follow_reference: bool,
    expansion_limit: usize,
    progress_policy: Option<super::ProgressPrefixPolicy<'_>>,
) -> Result<SearchPath, LocalPlanError> {
    super::cancellation::checkpoint()?;
    validate_caps(motion)?;
    let target = *reference.last().ok_or("近场搜索缺少目标")?;
    let support_offset = map
        .body_envelope()
        .ok_or("近场搜索缺少机体")?
        .body_to_support_z_m;
    let distance = |p: BodyPose| (p.0[0] - target[0]).hypot(p.0[1] - target[1]);
    if distance(start) > horizon + tomogram.resolution {
        return Err("姿态搜索目标超出近场视距".into());
    }
    let reference_length = reference
        .windows(2)
        .map(|pair| (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]))
        .sum::<f64>();
    let corridor = (corridor * 0.9).min(horizon * 0.3);
    let model = map.body_envelope().unwrap();
    let layer_tolerance = (model.body_to_support_z_m + model.z_bounds_m[1]) * 0.5;
    let tangent = reference.windows(2).rfind(|p| (p[1][0]-p[0][0]).hypot(p[1][1]-p[0][1])>1e-9).map_or(yaw(start.1), |pair| {
        (pair[1][1] - pair[0][1]).atan2(pair[1][0] - pair[0][0])
    });
    let reach = |p: BodyPose| {
        if let Some((goal, tolerance)) = arrival {
            return ((p.0[0] - goal[0]).powi(2)
                + (p.0[1] - goal[1]).powi(2)
                + (p.0[2] - goal[2]).powi(2))
            .sqrt()
                < tolerance
                && final_yaw.is_none_or(|heading| angle(yaw(p.1) - heading).abs() < 0.1);
        }
        // 中间轨迹必须走到窗口末端的横截面，并保持前向朝向。
        // 不能把障碍前的六成前缀当作已经接通交接点。
        reference_handoff(p, reference, support_offset, corridor, layer_tolerance, reference_length, tangent)
    };
    // 绕行窗口的目标可能就在墙另一侧；直线距离会把搜索反复拉回已失败的通道。
    // 沿已给出的参考走廊估计剩余行程，仍由实际机体动作决定能否通行。
    let remaining = |p: BodyPose| {
        if !follow_reference {
            return (distance(p), 0.0);
        }
        let (deviation, progress, _) =
            reference_progress([p.0[0], p.0[1], p.0[2] - support_offset], reference);
        ((reference_length - progress).max(0.0) + deviation, progress)
    };
    // Forward-only search must rank the turn needed to make progress as well as
    // translation. A distance-only heuristic starves a reversed departure behind
    // cheap small turns and centimetre translations within the finite budget.
    // These weighted nominal seconds guide ordering; only swept body checks admit edges.
    // Detours aim a body radius ahead on the reference, so a corner can be approached
    // with its outgoing direction before reaching the exact reference vertex.
    let guidance_distance=model.radius().max(tomogram.resolution);
    let turns = if follow_reference { reference_turns(reference)? } else { Vec::new() };
    #[cfg(test)]
    let diagnostic = DIAGNOSTIC_PRIORITY.with(|v| v.borrow().clone());
    #[cfg(test)]
    let oracle = if let DiagnosticPriority::OracleGuide(points) = &diagnostic {
        Some((points, reference_turns(points)?, points.windows(2).map(|w| (w[1][0]-w[0][0]).hypot(w[1][1]-w[0][1])).sum::<f64>()))
    } else { None };
    let heuristic = |p: BodyPose| {
        #[cfg(test)]
        {
            if let DiagnosticPriority::RelaxedGoal(weight) = diagnostic {
                // XY goal-ball relaxation: every accepted 3D goal also lies in
                // this disk. projected_cost charges >= XY travel/v and >= yaw/omega
                // per edge; their maximum (not sum) is a lower bound on that cost.
                let (travel, turn) = if let Some((goal, tolerance)) = arrival {
                    (((p.0[0]-goal[0]).hypot(p.0[1]-goal[1])-tolerance).max(0.0),
                        final_yaw.map_or(0.0, |y| (angle(yaw(p.1)-y).abs()-0.1).max(0.0)))
                } else {
                    ((distance(p)-corridor).max(0.0),
                        (angle(yaw(p.1)-tangent).abs()-std::f64::consts::FRAC_PI_2).max(0.0))
                };
                return weight * (travel/motion.max_forward_mps).max(turn/motion.max_angular_rps);
            }
            if let Some((points, oracle_turns, length)) = &oracle {
                let (deviation, progress, _) = reference_progress([p.0[0],p.0[1],p.0[2]-support_offset],points);
                let guide = reference_guide(points,progress+guidance_distance,*length);
                let turn = heading_potential(p,target,tangent,corridor,arrival.is_some(),Some(guide),oracle_turns);
                return 4.0*((length-progress).max(0.0)+deviation)/motion.max_forward_mps + 4.0*turn/motion.max_angular_rps;
            }
        }
        let (remaining_distance,progress)=remaining(p);
        let guide = follow_reference.then(|| reference_guide(reference,progress+guidance_distance,reference_length));
        let turn = heading_potential(p,target,tangent,corridor,arrival.is_some(),guide,&turns);
        // Guided weighted nominal seconds. The weight deliberately over-estimates
        // the per-metre floor so the queue stays focused on the reference corridor;
        // every admitted edge still passes the full swept-body checks, so this
        // ranks work rather than granting clearance. A pure lower bound expands
        // far more states to reach the same connection.
        #[cfg(test)]
        {
            match HEURISTIC_MODE.with(|v|v.get()) {
                HeuristicMode::AdmissibleMax => return (remaining_distance/motion.max_forward_mps)
                    .max(turn/motion.max_angular_rps),
                HeuristicMode::ScaledSum(w) => return w*remaining_distance/motion.max_forward_mps
                    + w*turn/motion.max_angular_rps,
                HeuristicMode::AdmissibleSum => return remaining_distance/motion.max_forward_mps
                    + turn/motion.max_angular_rps,
            }
        }
        #[cfg(not(test))]
        {
            4.0*remaining_distance/motion.max_forward_mps+4.0*turn/motion.max_angular_rps
        }
    };
    // Exact identity retains legal centimetre departures and small rotations.
    // More distinct states compete within the same finite expansion budget.
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        samples: Vec::new(),
    }];
    let mut open = BinaryHeap::from([Entry(heuristic(start), 0)]);
    let mut best = HashMap::from([(key(start), 0.0)]);
    let mut rejected_sample=None;
    fn admitted<T>(result:Result<T,LocalPlanError>,sample:&mut Option<nuwax_nav_mls_planner::surface::BodyFailureWitness>)
        ->Result<Option<T>,LocalPlanError> {
        if let Err(error)=&result {if sample.is_none() {*sample=error.body_witness;}}
        super::admit_candidate(result)
    }
    let mut work = SearchWork::default();
    work.nodes = 1;
    work.frontier_peak = 1;
    #[cfg(test)]
    INITIAL_ADMISSIONS.with(|v|v.borrow_mut().clear());
    let mut expanded = 0;
    let mut closest = start;
    let mut budget_exhausted = false;
    let expansion_limit = effective_expansion_limit(expansion_limit);
    while let Some(Entry(_priority, id)) = open.pop() {
        super::cancellation::checkpoint()?;
        let from = nodes[id].pose;
        let cost = nodes[id].cost;
        if is_stale(&nodes[id], &best) {
            continue;
        }
        #[cfg(all(test,feature="profile"))]
        if std::env::var_os("NEH_ON_POP_TRACE").is_some() {
            eprintln!("ON_POP {}",serde_json::json!({"id":id,"parent":nodes[id].parent,
                "pose":from,"g":cost,"h":heuristic(from),"f":_priority,
                "start_xy":[start.0[0],start.0[1]],"expanded":expanded,
                "travel":(from.0[0]-start.0[0]).hypot(from.0[1]-start.0[1]),
                "nodes":nodes.len(),"frontier":open.len()}));
        }
        #[cfg(all(test,feature="profile"))]
        if std::env::var_os("NEH_ON_DIAGNOSTIC").is_some() {
            let support=[from.0[0],from.0[1],from.0[2]-support_offset];
            eprintln!("ON_POP {}",serde_json::json!({"id":id,"parent":nodes[id].parent,"pose":from,"g":cost,"h":heuristic(from),"translation_h":4.0*remaining(from).0/motion.max_forward_mps,"f":_priority,"remaining":distance(from),"progress":reference_progress(support,reference).1,"cell":tomogram.cell_at(support).map(|i|format!("{:?}",tomogram.cells[i]))}));
        }
        if distance(from) < distance(closest) {closest=from;}
        // 终端区域内优先尝试完整直线接续；离散动作只用于绕过确实阻挡直线的障碍。
        // 解析接续避免用交替左右固定角度去逼近一个本来可以直达的交接横截面。
        let mut connection=None;
        // A reached state already has a fully admitted prefix. Do not turn
        // toward the click center and back after its arrival region is satisfied.
        let reached = reach(from);
        let offsets=if reached {Vec::new()}else if arrival.is_some() {vec![0.0]}else{vec![0.0,corridor*0.5,-corridor*0.5,corridor,-corridor]};
        let mut proposals=Vec::new();
        for offset in offsets {
            super::cancellation::checkpoint()?;
            let xy=[target[0]-tangent.sin()*offset,target[1]+tangent.cos()*offset];
            let dx=xy[0]-from.0[0];let dy=xy[1]-from.0[1];
            let distance=dx.hypot(dy);
            let heading=if distance==0.0 {yaw(from.1)}else{dy.atan2(dx)};
            let error=angle(heading-yaw(from.1));
            let line_time=connector_cost(from,heading,distance,0.0,final_yaw,target[2],support_offset,tomogram,reference,motion);
            proposals.push((line_time,heading,distance,0.0));
            if error.abs()>1e-6 && error.abs()<std::f64::consts::FRAC_PI_2 {
                let length=distance*error/error.sin();let turn=2.0*error;
                if motion.minimum_turning_radius_m==0.0 || length/turn.abs()>=motion.minimum_turning_radius_m {
                    let time=connector_cost(from,yaw(from.1),length,turn,final_yaw,target[2],support_offset,tomogram,reference,motion);
                    proposals.push((time,yaw(from.1),length,turn));
                }
            }
        }
        // 按声明线速、角速算接续时间，优先核验可执行的直线或单一圆弧。
        // 不再按固定离散转角拼出波浪，也不为省一段几何检查放弃真实净空。
        proposals.sort_by(|a,b|a.0.total_cmp(&b.0));
        for (_,heading,length,turn) in proposals {
            super::cancellation::checkpoint()?;
            // 解析长接续先用层析中心路线排除穿墙提案；未通过仍可由原姿态动作搜索绕行。
            // 这一步不授予机体许可，入选提案随后保留完整地形、旋转和扫掠检查。
            let samples=(length/(tomogram.resolution*0.5)).ceil().max(1.0) as usize;
            let mut previous=[from.0[0],from.0[1],from.0[2]-support_offset];
            let Some(reference_height)=tomogram.calibrated_reference_height(previous) else {continue;};
            previous[2]=reference_height;
            let mut visible=true;
            for i in 1..=samples {
                super::cancellation::checkpoint()?;
                let t=i as f64/samples as f64;
                let theta=turn*t;
                let (dx,dy)=if turn.abs()<1e-8 {(length*t,0.0)}
                    else {(length/turn*theta.sin(),length/turn*(1.0-theta.cos()))};
                let xy=[from.0[0]+dx*heading.cos()-dy*heading.sin(),
                    from.0[1]+dx*heading.sin()+dy*heading.cos()];
                if arrival.is_some_and(|(goal,tolerance)|(xy[0]-goal[0]).hypot(xy[1]-goal[1])<tolerance) {break;}
                // 预检沿真实地形更新参考，缺测只继承已有高度。直插目标Z会在
                // 缺测列里造出斜面，再把自己虚构的高度误差当作障碍拒绝。
                // 这里只核对参考几何，不用额外圆盘替换下方完整非对称机体扫掠。
                let height=tomogram.cell_at([xy[0],xy[1],previous[2]])
                    .map_or(previous[2],|id|tomogram.position(id)[2]);
                let point=[xy[0],xy[1],height];
                if !tomogram.segment_clear(previous,point,boundary) {visible=false;break;}
                previous=point;
            }
            if !visible {continue;}
            let mut prefix=vec![from];
            if admitted(super::rotate(map,&mut prefix,heading,motion,boundary),&mut rejected_sample)?.is_none() {continue;}
            let in_goal=|pose:BodyPose|arrival.is_some_and(|(goal,tolerance)|super::distance3(pose.0,goal)<tolerance);
            let Some(line)=admitted(probe_motion_until(map,*prefix.last().unwrap(),length,turn,boundary,in_goal),&mut rejected_sample)? else{continue;};
            prefix.extend(line);
            if let Some(heading)=final_yaw {
                if admitted(super::rotate(map,&mut prefix,heading,motion,boundary),&mut rejected_sample)?.is_none(){continue;}
            }
            if reach(*prefix.last().unwrap()) {
                connection=Some(prefix);break;
            }
        }
        if reached || connection.is_some() {
            let mut chain = Vec::new();
            let mut current = id;
            while let Some(parent) = nodes[current].parent {
                super::cancellation::checkpoint()?;
                chain.push(current);
                current = parent;
            }
            chain.reverse();
            let mut path = vec![start];
            for id in chain {
                super::cancellation::checkpoint()?;
                path.extend_from_slice(&nodes[id].samples);
            }
            if let Some(connection)=connection {
                path.extend(connection.into_iter().skip(1));
            }
            super::cancellation::checkpoint()?;
            return Ok(SearchPath {poses:path,partial:false});
        }
        expanded += 1;
        if expanded > expansion_limit {
            budget_exhausted = true;
            break;
        }
        work.expanded += 1;
        let fine = (f64::from(map.source_voxel_size_m()) * 0.5).clamp(0.01, 0.05);
        let body_radius = map
            .body_envelope()
            .unwrap()
            .radius()
            .max(motion.minimum_turning_radius_m);
        let body_step = (body_radius * 0.35).clamp(fine * 2.0, 0.2);
        let body_turn = body_step / body_radius.max(fine);
        let actions = [
            (0.25_f64, 0.0_f64),
            (0.2, 0.25),
            (0.2, -0.25),
            (0.12, 0.45),
            (0.12, -0.45),
            (body_step, body_turn),
            (body_step, -body_turn),
            (0.0, 0.3),
            (0.0, -0.3),
            (fine, 0.0),
            (fine, 0.1),
            (fine, -0.1),
            (0.0, 0.08),
            (0.0, -0.08),
        ];
        // A legal coarse edge does not prove that its continuation can escape.
        // Every existing fine primitive receives the same admission checks.
        for (length, turn) in actions {
            super::cancellation::checkpoint()?;
            if length == 0.0 && !motion.can_rotate_in_place {
                continue;
            }
            if length > 0.0 && turn != 0.0 && length / turn.abs() < motion.minimum_turning_radius_m
            {
                continue;
            }
            work.candidates += 1;
            let Some((samples,views)) = admitted(project_motion_until(map, from, length, turn, |_|false),&mut rejected_sample)? else {
                continue;
            };
            let next = *samples.last().unwrap();
            if (next.0[0] - start.0[0]).hypot(next.0[1] - start.0[1]) > distance(start) + 1.0 {
                continue;
            }
            let next_cost = cost + projected_cost(from,&samples,support_offset,tomogram,reference,motion);
            let k = key(next);
            if best.get(&k).is_some_and(|&old| next_cost >= old) {
                continue;
            }
            // 已有合法路径代价更低时不重复做扫掠；真正入队的边仍完整核验。
            work.sweeps += 1;
            if admitted(validate_projected(from,&samples,&views,boundary),&mut rejected_sample)?.is_none() {continue;}
            #[cfg(test)]
            if id == 0 { INITIAL_ADMISSIONS.with(|v|v.borrow_mut().push((length,turn,next))); }
            #[cfg(all(test,feature="profile"))]
            if std::env::var_os("NEH_ON_DIAGNOSTIC").is_some() && id==0 {
                eprintln!("ON_ADMIT {}",serde_json::json!({"length":length,"turn":turn,"pose":next,"g":next_cost,"h":heuristic(next),"translation_h":4.0*remaining(next).0/motion.max_forward_mps,"progress":reference_progress([next.0[0],next.0[1],next.0[2]-support_offset],reference).1}));
            }
            best.insert(k,next_cost);
            let next_id = nodes.len();
            nodes.push(Node {
                pose: next,
                cost: next_cost,
                parent: Some(id),
                samples,
            });
            let heading_cost = final_yaw.map_or(0.0, |y| angle(y - yaw(next.1)).abs() / motion.max_angular_rps * 0.05);
            #[cfg(test)]
            let heading_cost = if matches!(diagnostic, DiagnosticPriority::RelaxedGoal(_)) { 0.0 } else { heading_cost };
            open.push(Entry(
                next_cost + heuristic(next) + heading_cost,
                next_id,
            ));
            work.nodes = nodes.len();
            work.frontier_peak = work.frontier_peak.max(open.len());
        }
    }
    super::cancellation::checkpoint()?;
    if let Some(policy)=progress_policy.filter(|_|budget_exhausted) {
        let reference=policy.reference;
        let initial_progress=reference_progress([start.0[0],start.0[1],start.0[2]-support_offset],reference).1;
        let mut selected:Option<(usize,f64)>=None;
        for (id,node) in nodes.iter().enumerate().skip(1) {
            super::cancellation::checkpoint()?;
            if is_stale(node,&best) {continue;}
            let p=node.pose;let support=[p.0[0],p.0[1],p.0[2]-support_offset];
            let (deviation,progress,height)=reference_progress(support,reference);
            if progress-initial_progress<model.radius()
                || (p.0[0]-start.0[0]).hypot(p.0[1]-start.0[1])<policy.minimum_distance_m.max(model.radius())
                || deviation>corridor || (support[2]-height).abs()>layer_tolerance {continue;}
            let mut along=0.0;let mut local_heading=tangent;
            for pair in reference.windows(2) {
                let dx=pair[1][0]-pair[0][0];let dy=pair[1][1]-pair[0][1];let length=dx.hypot(dy);
                if length>1e-9 && progress<=along+length+1e-9 {local_heading=dy.atan2(dx);break;}
                along+=length;
            }
            if angle(yaw(p.1)-local_heading).cos()<=0.0 {continue;}
            if selected.is_none_or(|(old,best_progress)|progress>best_progress
                || progress==best_progress&&node.cost<nodes[old].cost) {selected=Some((id,progress));}
        }
        if let Some((id,progress))=selected {
            let mut chain=Vec::new();let mut current=id;
            while let Some(parent)=nodes[current].parent {super::cancellation::checkpoint()?;chain.push(current);current=parent;}
            chain.reverse();let mut poses=vec![start];
            for node in chain {poses.extend_from_slice(&nodes[node].samples);}
            // 每条精确边已完整扫掠；这只交付候选，发速仍由当前输入的完整停车检查决定。
            super::cancellation::checkpoint()?;
            tracing::info!(target:"navigation.scan",event="navigation.scan.progress_prefix",
                progress_m=progress-initial_progress,poses=poses.len(),expanded,
                "原预算耗尽后交付有前向进度的有限候选，原目标保持未到达");
            let partial=!reach(*poses.last().unwrap());
            return Ok(SearchPath {poses,partial});
        }
    }
    // Neither a drained discretized search nor an expansion limit proves obstruction.
    Err(LocalPlanError {
        reason: format!(
            "近场前向搜索{}（{expanded}次访问，{}个状态，{}个待选，最近{:.3}米，姿态{closest:?}）",
            if budget_exhausted { "预算耗尽" } else { "离散候选耗尽" },
            nodes.len(), open.len(), distance(closest)
        ),
        kind: if budget_exhausted { super::LocalPlanFailureKind::BudgetExhausted }
            else { super::LocalPlanFailureKind::SearchExhausted },
        closest_pose: Some(closest),
        blocked: None,
        body_witness: rejected_sample,
    })
}

#[cfg(test)]
mod identity_tests {
    use super::*;
    #[test]
    fn exact_pose_dominance_skips_even_one_ulp_costlier_queue_labels() {
        let pose = ([0.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let current_cost = 1.0_f64;
        let best = HashMap::from([(key(pose), current_cost)]);
        let mut node = Node { pose, cost: f64::from_bits(current_cost.to_bits() + 1),
            parent: None, samples: Vec::new() };
        assert!(is_stale(&node, &best), "an obsolete exact-pose label must not use expansion work");
        node.cost = current_cost;
        assert!(!is_stale(&node, &best));
        node.cost = f64::from_bits(current_cost.to_bits() - 1);
        assert!(!is_stale(&node, &best), "a genuinely cheaper label remains eligible");
        node.cost = 2.0;
        node.pose.0[0] = -0.0;
        assert!(!is_stale(&node, &best), "different raw poses cannot be merged");
        node.pose = (pose.0, pose.1.map(|v| -v));
        assert!(!is_stale(&node, &best), "quaternion antipodes keep existing exact identity");
    }

    #[test]
    fn raw_pose_key_preserves_all_seven_components_and_antipodes() {
        let pose=([0.0,0.0,0.4],[0.0,0.0,0.0,1.0]);
        let original=key(pose);
        let mut admitted=HashMap::from([(original,0)]);
        for index in 0..7 {
            let mut changed=pose;
            let component=if index<3 {&mut changed.0[index]} else {&mut changed.1[index-3]};
            *component=f64::from_bits(component.to_bits()+1);
            assert_ne!(original,key(changed),"component {index}");
            assert!(admitted.insert(key(changed),index+1).is_none());
        }
        let zeros=([0.0;3],[0.0;4]);
        for index in 0..7 {
            let mut changed=zeros;
            if index<3 {changed.0[index]=-0.0;} else {changed.1[index-3]=-0.0;}
            assert_ne!(key(zeros),key(changed),"signed zero component {index}");
        }
        let antipode=(pose.0,pose.1.map(|v|-v));
        assert_ne!(original,key(antipode));
        assert!(admitted.insert(key(antipode),8).is_none());
        assert_eq!(admitted.insert(original,9),Some(0));
    }
}

#[cfg(test)]
mod projected_cost_reuse_tests {
    use super::*;
    use super::super::tests::{motion,coarse_open_floor};

    // Independent pre-reuse implementation: retain original expressions/checkpoints.
    fn original_projected_cost(from: BodyPose, samples: &[BodyPose], offset: f64,
        tomogram: &Tomogram, reference: &[[f64;3]], motion: &MotionKinematics) -> f64 {
        let soft=|p:BodyPose|preference(tomogram,[p.0[0],p.0[1],p.0[2]-offset],reference);
        let mut previous=from; let mut cost=0.0;
        for &next in samples {
            if super::super::cancellation::checkpoint().is_err() { return f64::INFINITY; }
            let distance=super::super::distance3(previous.0,next.0);
            let weight=if distance==0.0 {0.0} else {(soft(previous)+soft(next))*0.5};
            cost+=step_cost(distance,angle(yaw(next.1)-yaw(previous.1)),weight,motion);
            previous=next;
        }
        cost
    }
    fn pose(x: f64, y: f64, z: f64, heading: f64) -> BodyPose {
        ([x,y,z],[0.0,0.0,(heading*0.5).sin(),(heading*0.5).cos()])
    }
    fn same_bits(label: &str, from: BodyPose, samples: &[BodyPose], offset: f64,
        tomo: &Tomogram, reference: &[[f64;3]]) {
        let caps=motion();
        let expected=original_projected_cost(from,samples,offset,tomo,reference,&caps);
        let actual=projected_cost(from,samples,offset,tomo,reference,&caps);
        assert_eq!(actual.to_bits(),expected.to_bits(),"{label}: actual={actual:?}, original={expected:?}");
    }

    #[test]
    fn actual_projected_cost_matches_original_bits_across_terrain_and_references() {
        let map=coarse_open_floor();let mut tomo=Tomogram::from_surface(&map).unwrap();
        let from=pose(0.0,0.2,0.4,0.0);
        let projected=probe_motion(&map,from,0.5,0.0,&[]).unwrap();
        assert!(projected.len()>2);
        let a=pose(0.1,0.2,0.4,0.0);let rotated=pose(0.1,0.2,0.4,0.3);
        let b=pose(0.2,0.2,0.4,0.3);
        let cases=vec![
            ("empty",vec![]), ("one",vec![a]), ("projected",projected),
            ("rotate_repeat_between_runs",vec![a,rotated,rotated,b,pose(0.3,0.25,0.45,0.5)]),
            ("zero_only",vec![from,pose(0.0,0.2,0.4,0.3),from]),
            ("signed_zero_antipode",vec![pose(-0.0,0.2,0.4,0.0),(from.0,from.1.map(|v|-v)),a,b]),
            ("missing_cell",vec![pose(50.0,0.2,0.4,0.0),pose(51.0,0.2,0.5,0.1)]),
        ];
        let references=vec![vec![],vec![[0.0,0.2,0.0]],vec![[0.0,0.2,0.0];3],
            vec![[0.0,0.2,0.0],[0.5,0.2,0.0]],
            vec![[-0.1,0.1,0.0],[0.2,0.1,0.0],[0.2,0.1,0.4],[0.2,0.5,0.4],[0.5,0.2,0.0]]];
        for terrain in 0..7 {
            for cell in &mut tomo.cells {
                cell.cost=match terrain {
                    0 => if cell.column[0]<0 {0.25} else {3.0+(cell.column[1].rem_euclid(3) as f64)},
                    1 => -1.0, 2 => f64::NAN, 3 => f64::NEG_INFINITY,
                    4 => 0.0, 5 => 1.0, _ => f64::INFINITY,
                };
            }
            for offset in [0.4,0.0,0.6] { for reference in &references { for (label,samples) in &cases {
                same_bits(label,from,samples,offset,&tomo,reference);
            }}}
        }
    }

    #[test]
    fn underflow_zero_segment_discards_the_previous_endpoint_preference() {
        let map=coarse_open_floor();let mut tomo=Tomogram::from_surface(&map).unwrap();
        for cell in &mut tomo.cells {cell.cost=if cell.column[0]<0 {1.0} else {9.0};}
        let left=pose(-1e-200,0.2,0.4,0.0);let right=pose(1e-200,0.2,0.4,0.0);
        assert_ne!(left.0[0].to_bits(),right.0[0].to_bits());
        assert_eq!(super::super::distance3(left.0,right.0),0.0);
        let left_id=tomo.cell_at([left.0[0],0.2,0.0]).unwrap();
        let right_id=tomo.cell_at([right.0[0],0.2,0.0]).unwrap();
        assert_ne!(left_id,right_id);
        assert_ne!(tomo.cells[left_id].cost,tomo.cells[right_id].cost);
        let reference=[[0.0,0.2,0.0],[0.5,0.2,0.0]];
        same_bits("underflow crossing cells",pose(0.1,0.2,0.4,0.0),
            &[left,right,pose(0.2,0.2,0.4,0.0)],0.4,&tomo,&reference);
    }

    #[test]
    fn nonfinite_helper_inputs_retain_original_result_bits() {
        let map=coarse_open_floor();let mut tomo=Tomogram::from_surface(&map).unwrap();
        for cell in &mut tomo.cells {cell.cost=0.75;}
        let from=pose(0.0,0.2,0.4,0.0);let a=pose(0.1,0.2,0.4,0.0);
        let b=pose(0.2,0.2,0.4,0.2);
        let reference=[[0.0,0.2,0.0],[0.5,0.2,0.0]];
        for value in [f64::INFINITY,f64::NEG_INFINITY,f64::NAN,f64::from_bits(0x7ff8000000000042)] {
            same_bits("nonfinite x",from,&[a,pose(value,0.2,0.4,0.0),b],0.4,&tomo,&reference);
            same_bits("nonfinite z",from,&[a,pose(0.15,0.2,value,0.0),b],0.4,&tomo,&reference);
            same_bits("nonfinite offset",from,&[a,b],value,&tomo,&reference);
            same_bits("nonfinite reference",from,&[a,b],0.4,&tomo,&[[value,0.2,0.0],[0.5,0.2,0.0]]);
            same_bits("nonfinite rotation",from,&[a,(a.0,[0.0,0.0,value,1.0]),b],0.4,&tomo,&reference);
        }
    }

    #[test]
    fn cancellation_at_each_sample_preserves_result_and_checkpoint_consumption() {
        let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
        let caps=motion();let from=pose(0.0,0.2,0.4,0.0);
        let reference=[[0.0,0.2,0.0],[0.5,0.2,0.0]];
        let a=pose(0.1,0.2,0.4,0.0);
        let samples=[a,pose(0.2,0.2,0.4,0.0),pose(0.2,0.2,0.4,0.3),pose(0.3,0.2,0.4,0.3)];
        for length in 0..=samples.len() { for budget in 0..=length+1 {
            let selected=&samples[..length];
            let expected=super::super::cancellation::after_checkpoints(budget,|| {
                let cost=original_projected_cost(from,selected,0.4,&tomo,&reference,&caps);
                (cost.to_bits(),super::super::cancellation::checkpoint().is_err())
            });
            let actual=super::super::cancellation::after_checkpoints(budget,|| {
                let cost=projected_cost(from,selected,0.4,&tomo,&reference,&caps);
                (cost.to_bits(),super::super::cancellation::checkpoint().is_err())
            });
            assert_eq!(actual,expected,"length={length}, budget={budget}");
            assert_eq!(actual.1,budget<=length);
            if budget<length {assert_eq!(actual.0,f64::INFINITY.to_bits());}
            else {assert!(f64::from_bits(actual.0).is_finite());}
        }}
    }
}

#[cfg(test)]
mod objective_tests {
    use super::*;
    use super::super::tests::{motion,coarse_open_floor};
    #[test]
    fn objective_subdivision_is_additive_without_edge_floors() {
        let motion=motion();
        for (distance,turn,preference) in [(0.25,0.0,0.4),(0.01,0.1,0.7),(0.0,0.3,0.0),(0.0,0.0,0.0)] {
            let whole=step_cost(distance,turn,preference,&motion);
            for n in [2,5,25,100] {
                let subdivided=(0..n).map(|_|step_cost(distance/n as f64,turn/n as f64,preference,&motion)).sum::<f64>();
                assert!((whole-subdivided).abs()<1e-12);
            }
        }
        // Trapezoidal integration of a linearly varying preference also adds.
        let whole=step_cost(0.25,0.0,0.5,&motion);
        let split=step_cost(0.1,0.0,0.2,&motion)+step_cost(0.15,0.0,0.7,&motion);
        assert!((whole-split).abs()<1e-12);
        assert_eq!(soft_intervals(1e300),64);
        assert_eq!(soft_intervals(0.0),1);
        let mut tiny=motion; tiny.max_forward_mps=f64::MIN_POSITIVE;
        assert!(!step_cost(100.0,0.0,0.0,&tiny).is_nan());
    }
    #[test]
    fn objective_terrain_and_sampled_subdivision_semantics() {
        let map=coarse_open_floor();let mut tomo=Tomogram::from_surface(&map).unwrap();
        let from=([0.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let reference=[[0.0,0.2,0.0],[0.5,0.2,0.0]];
        let samples=probe_motion(&map,from,0.5,0.0,&[]).unwrap();
        for terrain in [-1.0,f64::NAN,f64::NEG_INFINITY,0.0,1.0,f64::INFINITY] {
            for cell in &mut tomo.cells {cell.cost=terrain;}
            let whole=projected_cost(from,&samples,0.4,&tomo,&reference,&motion());
            let middle=samples.len()/2;
            let split=projected_cost(from,&samples[..middle],0.4,&tomo,&reference,&motion())
                +projected_cost(samples[middle-1],&samples[middle..],0.4,&tomo,&reference,&motion());
            assert!(!whole.is_nan());
            if terrain==f64::INFINITY {assert_eq!(whole,f64::INFINITY);assert_eq!(split,whole);}
            else {assert!((whole-split).abs()<1e-12);assert!(whole>=0.5-1e-12);}
            assert_eq!(projected_cost(from,&[from],0.4,&tomo,&reference,&motion()),0.0);
        }
    }
    #[test]
    fn nominal_support_offset_and_lateral_connectors_match_flat_projected_cost() {
        let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
        let from=([0.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        for lateral in [0.0_f64,0.15,-0.15] {
            let heading=lateral.atan2(0.5_f64);let length=0.5_f64.hypot(lateral);
            let reference=[[0.0,0.2,0.0],[0.5,0.2+lateral,0.0]];
            let mut path=vec![from];super::super::rotate(&map,&mut path,heading,&motion(),&[]).unwrap();
            path.extend(probe_motion(&map,*path.last().unwrap(),length,0.0,&[]).unwrap());
            let actual=projected_cost(from,&path,0.4,&tomo,&reference,&motion());
            let nominal=connector_cost(from,heading,length,0.0,None,0.0,0.4,&tomo,&reference,&motion());
            assert!((actual-nominal).abs()<1e-10,"lateral={lateral} actual={actual} nominal={nominal}");
        }
    }
    #[test]
    fn projected_objective_counts_full_turn_and_ignores_duplicate_points() {
        let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([0.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let reference=[[0.0,0.2,0.0];2];
        let poses:Vec<_>=(1..=100).flat_map(|i| {
            let yaw=std::f64::consts::TAU*i as f64/100.0;
            let p=(start.0,[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()]);[p,p]
        }).collect();
        let cost=projected_cost(start,&poses,0.4,&tomo,&reference,&motion());
        assert!((cost-std::f64::consts::TAU/motion().max_angular_rps).abs()<1e-10);
        assert_eq!(projected_cost(start,&[start,start],0.4,&tomo,&reference,&motion()),0.0);
        assert_eq!(reference_distance([1.0,0.2,0.0],&reference),1.0);
        assert_eq!(reference_progress([1.0,0.2,0.0],&reference),(1.0,0.0,0.0));
    }
}

#[cfg(test)]
mod complete_guide_turn_tests {
    use super::*;
    fn close(a: f64, b: f64) { assert!((a-b).abs()<1e-12,"{a} != {b}"); }
    fn length(reference: &[[f64;3]]) -> f64 {
        reference.windows(2).map(|p|(p[1][0]-p[0][0]).hypot(p[1][1]-p[0][1])).sum()
    }
    fn potential(reference: &[[f64;3]], lookahead: f64, position: [f64;2], heading: f64) -> f64 {
        let turns=reference_turns(reference).unwrap();
        reference_guide_turn(position,heading,reference_guide(reference,lookahead,length(reference)),&turns)
    }

    #[test]
    fn angular_accounting_redistributes_bearing_and_has_no_forward_switch_jump() {
        let theta=std::f64::consts::FRAC_PI_2;
        for bearing in [0.0,0.3,0.9,theta] {
            close(angle(0.0-bearing).abs()+angle(bearing-theta).abs(),theta);
        }
        for (incoming,outgoing) in [(0.0,theta),(3.0,-3.0),(-2.0,1.0)] {
            for heading in [-3.0,0.0,2.0] { for bearing in [-3.1,-0.1,0.8,3.1] {
                let before=angle(heading-bearing).abs()+angle(bearing-incoming).abs()+angle(incoming-outgoing).abs()+0.7;
                let after=angle(heading-bearing).abs()+angle(bearing-outgoing).abs()+0.7;
                assert!(after<=before+1e-12);
            }}
        }
    }

    #[test]
    fn actual_tie_region_turns_match_original_guide_slope_and_reverse_fixed_segment_penalty() {
        for (mirror,position,heading,delta,progress,incoming) in [
            (false,[-1.6940133792580234,1.268233336269405],1.549999999999999,-0.3,2.7,std::f64::consts::FRAC_PI_2),
            (true,[-1.576267083366934,0.11019567393312495],-3.133185307179587,0.3,1.5,std::f64::consts::PI),
        ] {
            let sign=if mirror {-1.0}else{1.0};
            let reference=[[0.0,0.0,0.0],[-1.5,0.0,0.0],[-1.5,sign*1.2,0.0],[1.2,sign*1.2,0.0],[1.2,0.0,0.0]];
            let turns=reference_turns(&reference).unwrap();
            let start_guide=reference_guide(&reference,0.3_f64.hypot(0.16),length(&reference));
            close(reference_guide_turn([0.0,0.0],0.3,start_guide,&turns)
                -reference_guide_turn([0.0,0.0],0.0,start_guide,&turns),-0.3);
            let guide=reference_guide(&reference,progress+0.3_f64.hypot(0.16),length(&reference));
            let before=reference_guide_turn(position,heading,guide,&turns);
            let after=reference_guide_turn(position,heading+delta,guide,&turns);
            let bearing=(guide.point[1]-position[1]).atan2(guide.point[0]-position[0]);
            let old_delta=angle(heading+delta-bearing).abs()-angle(heading-bearing).abs();
            close(after-before,old_delta); // Added continuation debt is yaw-independent here.
            close(delta.abs()/0.8+4.0*(after-before)/0.8,-1.125);
            let fixed_delta=angle(heading+delta-incoming).abs()-angle(heading-incoming).abs();
            close(delta.abs()/0.8+4.0*fixed_delta/0.8,1.875);
        }
    }

    #[test]
    fn terminal_guide_never_imposes_last_segment_heading() {
        let reference=[[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,1.0,0.0]];
        for lookahead in [2.0,3.0] {
            let guide=reference_guide(&reference,lookahead,length(&reference));
            assert!(guide.segment.is_none());
            close(potential(&reference,lookahead,[0.0,1.0],0.0),0.0);
            close(potential(&reference,lookahead,[0.0,1.0],std::f64::consts::PI),std::f64::consts::PI);
            for heading in [-2.0,0.0,2.0] { close(potential(&reference,lookahead,[1.0,1.0],heading),0.0); }
        }
        let point=[[0.0,0.0,0.0]];
        close(potential(&point,1.0,[0.0,0.0],2.0),0.0);
        close(potential(&point,1.0,[-1.0,0.0],0.0),0.0);
    }

    #[test]
    fn coincident_internal_guide_uses_outgoing_segment_across_duplicates_and_heights() {
        let reference=[[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,2.0],[1.0,1.0,2.0]];
        let guide=reference_guide(&reference,1.0,length(&reference));
        assert_eq!(guide.segment,Some(0)); assert!(guide.at_vertex);
        close(potential(&reference,1.0,[1.0,0.0],std::f64::consts::FRAC_PI_2),0.0);
        close(potential(&reference,1.0,[1.0+0.5e-6,0.0],0.0),std::f64::consts::FRAC_PI_2);
        // At exactly the old 1e-6 cutoff the nonzero approach bearing remains defined.
        let simple=[[0.0,0.0,0.0],[0.0,1.0,0.0]];
        let turns=reference_turns(&simple).unwrap();
        let internal=ReferenceGuide {point:[0.0,0.0,0.0],segment:Some(0),at_vertex:false,matched:true};
        close(reference_guide_turn([-1e-6,0.0],std::f64::consts::FRAC_PI_2,internal,&turns),std::f64::consts::PI);
        close(reference_guide_turn([0.0,0.0],std::f64::consts::FRAC_PI_2,internal,&turns),0.0);
    }

    #[test]
    fn collinear_subdivision_wrap_and_original_threshold_difference_are_explicit() {
        let original=[[0.0,0.0,0.0],[2.0,0.0,0.0],[2.0,2.0,0.0]];
        let divided=[[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0],[2.0,0.0,0.0],[2.0,1.0,0.0],[2.0,2.0,0.0]];
        for lookahead in [0.5,1.0,1.5,2.0,2.5,4.0] { for heading in [-3.0,0.0,3.0] {
            close(potential(&original,lookahead,[0.2,0.1],heading),potential(&divided,lookahead,[0.2,0.1],heading));
        }}
        let a=179.0_f64.to_radians();let b=-179.0_f64.to_radians();let point=[a.cos(),a.sin(),0.0];
        let wrapped=[[0.0,0.0,0.0],point,point,[point[0]+b.cos(),point[1]+b.sin(),0.0]];
        close(reference_turns(&wrapped).unwrap()[0].suffix,2.0_f64.to_radians());
        let tiny=[[0.0,0.0,0.0],[1e-9,0.0,0.0]];
        assert!(reference_turns(&tiny).unwrap()[0].heading.is_none()); // Guide retains >1e-9.
        close(reference_progress([1e-9,0.0,0.0],&tiny).1,1e-9); // Projection retains <1e-9 skip.
        assert!(reference_guide(&tiny,0.5e-9,length(&tiny)).segment.is_none());
    }
    #[test]
    fn missed_terminal_guide_preserves_full_error_cross_section_fallback() {
        let reference=[[-2.0,0.0,0.0],[0.0,0.0,0.0]];
        let target=[0.0,0.0,0.0];
        let pose=([-1.0,1.0,0.0],[0.0,0.0,0.0,1.0]);
        let turns=reference_turns(&reference).unwrap();
        let terminal=reference_guide(&reference,2.0,length(&reference));
        let beyond=reference_guide(&reference,3.0,length(&reference));
        assert!(terminal.matched && terminal.segment.is_none());
        assert!(!beyond.matched && beyond.segment.is_none());
        // Exactly terminal still matched the old loop and therefore aims at the center.
        close(heading_potential(pose,target,0.0,2.0,false,Some(terminal),&turns),std::f64::consts::FRAC_PI_4);
        // Beyond it, no arrival means the unchanged five-offset full-error minimum.
        close(heading_potential(pose,target,0.0,2.0,false,Some(beyond),&turns),0.0);
        close(heading_potential(pose,target,0.0,2.0,true,Some(beyond),&turns),std::f64::consts::FRAC_PI_4);
        let point=[target];let point_guide=reference_guide(&point,1.0,0.0);
        assert!(!point_guide.matched);
        close(heading_potential(pose,target,0.0,2.0,false,Some(point_guide),&[]),0.0);
        close(heading_potential(pose,target,0.0,2.0,true,Some(point_guide),&[]),std::f64::consts::FRAC_PI_4);
        // Direct mode keeps its existing pi/2 clamp rather than reference full error.
        close(heading_potential(pose,target,0.0,2.0,true,None,&[]),0.0);
    }

}
