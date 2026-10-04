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

pub fn probe_motion(
    map: &SurfaceSnapshot,
    from: BodyPose,
    length: f64,
    turn: f64,
    boundary: &[[f64; 2]],
) -> Option<Vec<BodyPose>> {
    probe_motion_until(map,from,length,turn,boundary,|_|false).ok()
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
    for &next in samples {
        if super::cancellation::checkpoint().is_err() { return f64::INFINITY; }
        let distance=super::distance3(previous.0,next.0);
        let weight=if distance==0.0 {0.0} else {(soft(previous)+soft(next))*0.5};
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
) -> Result<Vec<BodyPose>, LocalPlanError> {
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
        let (deviation, progress, height) =
            reference_progress([p.0[0], p.0[1], p.0[2] - support_offset], reference);
        if distance(p) > corridor
            || deviation > corridor
            || progress < reference_length - 1e-6
            || (p.0[2] - support_offset - height).abs() > layer_tolerance
        {
            return false;
        }
        // 中间轨迹必须走到窗口末端的横截面，并保持前向朝向。
        // 不能把障碍前的六成前缀当作已经接通交接点。
        angle(yaw(p.1) - tangent).cos() > 0.0
    };
    // 绕行窗口的目标可能就在墙另一侧；直线距离会把搜索反复拉回已失败的通道。
    // 沿已给出的参考走廊估计剩余行程，仍由实际机体动作决定能否通行。
    let remaining = |p: BodyPose| {
        if !follow_reference {
            return distance(p);
        }
        let (deviation, progress, _) =
            reference_progress([p.0[0], p.0[1], p.0[2] - support_offset], reference);
        (reference_length - progress).max(0.0) + deviation
    };
    // Exact identity retains legal centimetre departures and small rotations.
    // More distinct states compete within the same finite expansion budget.
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        samples: Vec::new(),
    }];
    let mut open = BinaryHeap::from([Entry(remaining(start) / motion.max_forward_mps * 4.0, 0)]);
    let mut best = HashMap::from([(key(start), 0.0)]);
    let mut work = SearchWork::default();
    work.nodes = 1;
    work.frontier_peak = 1;
    #[cfg(test)]
    INITIAL_ADMISSIONS.with(|v|v.borrow_mut().clear());
    let mut expanded = 0;
    let mut closest = start;
    let mut budget_exhausted = false;
    #[cfg(not(test))]
    let expansion_limit = 256;
    #[cfg(test)]
    let expansion_limit = EXPANSION_LIMIT.get();
    while let Some(Entry(_, id)) = open.pop() {
        super::cancellation::checkpoint()?;
        let from = nodes[id].pose;
        let cost = nodes[id].cost;
        if best
            .get(&key(from))
            .is_some_and(|&v| cost > v + 1e-9)
        {
            continue;
        }
        if distance(from) < distance(closest) {closest=from;}
        // 终端区域内优先尝试完整直线接续；离散动作只用于绕过确实阻挡直线的障碍。
        // 解析接续避免用交替左右固定角度去逼近一个本来可以直达的交接横截面。
        let mut connection=None;
        let offsets=if arrival.is_some() {vec![0.0]}else{vec![0.0,corridor*0.5,-corridor*0.5,corridor,-corridor]};
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
            let mut visible=true;
            for i in 1..=samples {
                super::cancellation::checkpoint()?;
                let t=i as f64/samples as f64;
                let theta=turn*t;
                let (dx,dy)=if turn.abs()<1e-8 {(length*t,0.0)}
                    else {(length/turn*theta.sin(),length/turn*(1.0-theta.cos()))};
                let point=[from.0[0]+dx*heading.cos()-dy*heading.sin(),
                    from.0[1]+dx*heading.sin()+dy*heading.cos(),
                    (from.0[2]-support_offset)*(1.0-t)+target[2]*t];
                if arrival.is_some_and(|(goal,tolerance)|(point[0]-goal[0]).hypot(point[1]-goal[1])<tolerance) {break;}
                if !tomogram.segment_clear(previous,point,boundary) {visible=false;break;}
                previous=point;
            }
            if !visible {continue;}
            let mut prefix=vec![from];
            if super::admit_candidate(super::rotate(map,&mut prefix,heading,motion,boundary))?.is_none() {continue;}
            let in_goal=|pose:BodyPose|arrival.is_some_and(|(goal,tolerance)|super::distance3(pose.0,goal)<tolerance);
            let Some(line)=super::admit_candidate(probe_motion_until(map,*prefix.last().unwrap(),length,turn,boundary,in_goal))? else{continue;};
            prefix.extend(line);
            if let Some(heading)=final_yaw {
                if super::admit_candidate(super::rotate(map,&mut prefix,heading,motion,boundary))?.is_none(){continue;}
            }
            if reach(*prefix.last().unwrap()) {
                connection=Some(prefix);break;
            }
        }
        if reach(from) || connection.is_some() {
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
            return Ok(path);
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
            let Some((samples,views)) = super::admit_candidate(project_motion_until(map, from, length, turn, |_|false))? else {
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
            if super::admit_candidate(validate_projected(from,&samples,&views,boundary))?.is_none() {continue;}
            #[cfg(test)]
            if id == 0 { INITIAL_ADMISSIONS.with(|v|v.borrow_mut().push((length,turn,next))); }
            best.insert(k,next_cost);
            let next_id = nodes.len();
            nodes.push(Node {
                pose: next,
                cost: next_cost,
                parent: Some(id),
                samples,
            });
            let heading_cost = final_yaw.map_or(0.0, |y| angle(y - yaw(next.1)).abs() / motion.max_angular_rps * 0.05);
            open.push(Entry(
                next_cost + 4.0 * remaining(next) / motion.max_forward_mps + heading_cost,
                next_id,
            ));
            work.nodes = nodes.len();
            work.frontier_peak = work.frontier_peak.max(open.len());
        }
    }
    super::cancellation::checkpoint()?;
    // Neither a drained discretized search nor an expansion limit proves obstruction.
    Err(LocalPlanError {
        reason: format!(
            "近场前向搜索未连通（{expanded}次访问，{}个状态，{}个待选，最近{:.3}米，姿态{closest:?}）",
            nodes.len(), open.len(), distance(closest)
        ),
        kind: if budget_exhausted { super::LocalPlanFailureKind::BudgetExhausted }
            else { super::LocalPlanFailureKind::SearchExhausted },
        closest_pose: Some(closest),
        blocked: None,
    })
}

#[cfg(test)]
mod identity_tests {
    use super::*;
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
