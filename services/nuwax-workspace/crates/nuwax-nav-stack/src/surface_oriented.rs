//! 按连续机体位姿搜索三维路径，平面运动基元在真实支撑面上逐段核验。
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_types_messages::{GoalOptions, MotionKinematics, MotionLimits, MotionModel, Point2, Twist2};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

type BodyPose = ([f64; 3], [f64; 4]);
// 优先较快取得可执行路线；加权启发式不承诺最短路径，碰撞/支撑检查不变。
const SEARCH_HEURISTIC_WEIGHT: f64 = 10.0;
fn angle(v: f64) -> f64 {
    (v + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
fn planar(p: BodyPose) -> PlanarPose {
    let [x, y, z, w] = p.1;
    PlanarPose {
        x: p.0[0],
        y: p.0[1],
        yaw: (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
    }
}
fn yaw_rotated(q: [f64; 4], angle: f64) -> [f64; 4] {
    let (s, c) = (angle * 0.5).sin_cos();
    [
        c * q[0] - s * q[1],
        c * q[1] + s * q[0],
        c * q[2] + s * q[3],
        c * q[3] - s * q[2],
    ]
}
#[derive(PartialEq)]
struct Queue(f64, usize);
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Ord for Queue {
    fn cmp(&self, b: &Self) -> Ordering {
        b.0.total_cmp(&self.0).then_with(|| b.1.cmp(&self.1))
    }
}
struct Node {
    pose: BodyPose,
    cost: f64,
    parent: Option<usize>,
    samples: Vec<BodyPose>,
}

fn action(
    map: &SurfaceSnapshot,
    from: BodyPose,
    velocity: Twist2,
    duration: f64,
    boundary: &[[f64; 2]],
) -> Option<Vec<BodyPose>> {
    let model = map.body_envelope()?;
    let start = planar(from);
    // 固定空间/角度步长让相近连接复用完全相同的旋转前缀查询。
    // 最后一步仍精确落在终点，每段弧长和角度上限与原检查一致。
    let rate = ((velocity.linear.x.hypot(velocity.linear.y)
        + model.radius() * velocity.angular_z.abs())
        / (f64::from(map.config().voxel_size) * 0.5))
        .max(velocity.angular_z.abs() / 0.08);
    let step = if rate > 0.0 { 1.0 / rate } else { duration };
    let n = (duration / step).ceil().max(1.0) as usize;
    if n > 1024 {
        return None;
    }
    let mut samples = Vec::with_capacity(n);
    let mut previous = from;
    let mut previous_time=0.0;
    for i in 1..=n {
        let time=(step*i as f64).min(duration);
        let next = integrate(start, velocity, time);
        // 平面执行器可以直接保持当前站姿。厚地面或局部噪声不能强迫每一拍
        // 抬到支撑格顶；先检查实际站姿的完整运动，只有不成立时才随地形投影。
        let maintained=([next.x,next.y,previous.0[2]],
            yaw_rotated(previous.1,velocity.angular_z*(time-previous_time)));
        if map.validate_body_motion(previous,maintained,boundary).is_ok() {
            samples.push(maintained);previous=maintained;previous_time=time;continue;
        }
        let support = map
            .locate_support_from_body(previous.0, previous.1, model.body_to_support_z_m)
            .ok()?.support_xyz;
        let mut lifted = map
            .body_pose_on_surface([next.x, next.y], next.yaw, support)
            .inspect_err(|error| tracing::trace!(?error,?previous,?next,?support,"三维基元投影被拒绝"))
            .ok()?;
        // 原地转向没有升降通道；地形姿态可以核验，但不能借转向抬高机体避障。
        if velocity.linear.x.hypot(velocity.linear.y)<1e-9 {lifted.0[2]=previous.0[2];}
        map.validate_body_motion(previous, lifted, boundary)
            .inspect_err(|error| tracing::trace!(?error,?previous,?lifted,"三维基元扫掠被拒绝"))
            .ok()?;
        samples.push(lifted);
        previous = lifted;
        previous_time=time;
    }
    Some(samples)
}
fn connect(
    map: &SurfaceSnapshot,
    from: BodyPose,
    to: BodyPose,
    boundary: &[[f64; 2]],
    rotate: bool,
    reverse: bool,
    omni: bool,
    require_orientation: bool,
) -> Option<Vec<BodyPose>> {
    let a = planar(from);
    let b = planar(to);
    let distance = (a.x - b.x).hypot(a.y - b.y);
    let heading = (b.y - a.y).atan2(b.x - a.x);
    let mut headings = vec![heading];
    if reverse {
        headings.push(angle(heading + std::f64::consts::PI));
    }
    if distance < 1e-9 {
        headings.insert(0, a.yaw);
    } else if omni {
        headings.push(a.yaw);
    }
    // 优先转向目标后前进；受阻时才利用明确支持的倒退、横移通道。
    // 明确指定最终朝向时，比较完整转向代价，避免先掉头再转回请求朝向。
    if require_orientation {
        headings.sort_by(|x,y|(angle(*x-a.yaw).abs()+angle(b.yaw-*x).abs())
            .total_cmp(&(angle(*y-a.yaw).abs()+angle(b.yaw-*y).abs())));
    }
    for heading in headings {
        // 位置目标的终点方向沿本段运动自然生成，不强制回到占位四元数。
        let to = if require_orientation { to } else {
            let model = map.body_envelope()?;
            let Ok(support) = map.locate_goal_support_from_body(to.0, to.1, model.body_to_support_z_m) else {continue;};
            let Ok(pose) = map.body_pose_on_surface([b.x, b.y], heading, support.surface_xyz) else {continue;};
            pose
        };
        let b = planar(to);
        if !rotate && (angle(heading - a.yaw).abs() > 1e-5 || angle(b.yaw - heading).abs() > 1e-5) {
            continue;
        }
        let shortest = angle(heading-a.yaw);
        let mut turns=vec![shortest];
        // 后方目标先检查左右两种转向；非对称机体在两侧的扫掠空间不同。
        if rotate && shortest.abs()>std::f64::consts::FRAC_PI_2 {
            turns.push(shortest-shortest.signum()*std::f64::consts::TAU);
        }
        for delta in turns {
        let candidate = (|| {
            let mut out = vec![from];
            if delta.abs() > 1e-6 {
                out.extend(action(
                    map,
                    from,
                    Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: delta.signum(),
                    },
                    delta.abs(),
                    boundary,
                )?);
            }
            let aligned = *out.last()?;
            let velocity = Twist2 {
                linear: Point2::new(
                    (b.x - a.x) * heading.cos() + (b.y - a.y) * heading.sin(),
                    -(b.x - a.x) * heading.sin() + (b.y - a.y) * heading.cos(),
                ),
                angular_z: 0.0,
            };
            if velocity.linear.x < -1e-8 && !reverse { return None; }
            if distance > 1e-9 {
                out.extend(action(map, aligned, velocity, 1.0, boundary)?);
            }
            let current = *out.last()?;
            let delta = angle(b.yaw - planar(current).yaw);
            if delta.abs() > 1e-6 {
                out.extend(action(
                    map,
                    current,
                    Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: delta.signum(),
                    },
                    delta.abs(),
                    boundary,
                )?);
            }
            map.validate_body_motion(*out.last()?, to, boundary).ok()?;
            out.push(to);
            Some(out)
        })();
        if candidate.is_some() {
            return candidate;
        }
        }
    }
    None
}

/// 点击高度选择支撑层，最终姿态仍由真实表面与完整机体包络决定。
fn prepare_goal_pose(map: &SurfaceSnapshot, goal: BodyPose, require_orientation: bool) -> Result<BodyPose, String> {
    let model=map.body_envelope().ok_or("三维完整机体标定缺失")?;
    if map.locate_support_from_body(goal.0,goal.1,model.body_to_support_z_m).is_ok()
        && map.validate_body_pose(goal.0,goal.1).is_ok() {return Ok(goal);}
    let support=map.locate_goal_support_from_body(goal.0,goal.1,model.body_to_support_z_m)
        .map_err(|error|format!("目标支撑层: {error}"))?;
    let projected=map.body_pose_on_surface([goal.0[0],goal.0[1]],planar(goal).yaw,support.surface_xyz);
    if require_orientation {return projected.map_err(|error|format!("目标支撑面投影: {error}"));}
    projected.or_else(|error|(0..32).find_map(|index|map.body_pose_on_surface(
        [goal.0[0],goal.0[1]],f64::from(index)*std::f64::consts::TAU/32.0,support.surface_xyz).ok())
        .ok_or(error)).map_err(|error|format!("目标支撑面投影: {error}"))
}
fn position_distance(a:BodyPose,b:BodyPose)->f64 {
    (a.0[0]-b.0[0]).hypot(a.0[1]-b.0[1]).hypot(a.0[2]-b.0[2])
}
/// 有界采样容差区域中的真实落点；不为缺测格补造地面。
fn goal_region_samples(map:&SurfaceSnapshot,start:BodyPose,goal:BodyPose,tolerance:f64)->Vec<BodyPose> {
    if tolerance<=0.0 {return vec![];}
    // 容差比规划格小时也要检查格内落点；机体边缘净空随实际坐标变化。
    // 大容差仍限制每轴最多八步，避免把精细采样扩展到整个地图。
    let step=f64::from(map.config().voxel_size).min(tolerance/2.0).max(tolerance/8.0);
    let radius=(tolerance/step).ceil().min(8.0) as i32;
    let mut candidates=Vec::new();
    for x in -radius..=radius {for y in -radius..=radius {for z in -radius..=radius {
        let candidate=([goal.0[0]+f64::from(x)*step,goal.0[1]+f64::from(y)*step,goal.0[2]+f64::from(z)*step],goal.1);
        let offset=position_distance(candidate,goal);
        if offset>1e-6 && offset<tolerance-1e-6 {candidates.push(candidate);}
    }}}
    candidates.sort_by(|a,b|position_distance(*a,start).total_cmp(&position_distance(*b,start)));
    candidates
}

/// 已声明全向通道时，平移与转向独立展开；候选边出队后才核验扫掠。
fn omnidirectional_connection(
    map: &SurfaceSnapshot,
    start: BodyPose,
    reference: BodyPose,
    tolerance: f64,
    boundary: &[[f64; 2]],
    reverse: bool,
    rotate: bool,
    goal_costs: &nuwax_nav_mls_planner::surface::SurfaceGoalCosts,
    cancelled: &impl Fn() -> bool,
) -> Option<Vec<BodyPose>> {
    let model = map.body_envelope()?;
    let step = f64::from(map.source_voxel_size_m()) * 0.5;
    let initial_yaw = planar(start).yaw;
    let pose_key = |pose: BodyPose| {
        [
            pose.0[0], pose.0[1], pose.0[2], pose.1[0], pose.1[1], pose.1[2], pose.1[3],
        ]
        .map(f64::to_bits)
    };
    let heuristic = |pose: BodyPose| {
        let remaining = (position_distance(pose, reference) - tolerance).max(0.0);
        let distance = map
            .locate_goal_support_from_body(pose.0, pose.1, model.body_to_support_z_m)
            .ok()
            .and_then(|location| goal_costs.cost(location.cell))
            .unwrap_or(remaining);
        remaining.max(distance)
    };
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        samples: vec![],
    }];
    let mut positions = vec![(0_i32, 0_i32, 0_i32)];
    let mut stances = vec![start.1];
    let mut pending_motions = vec![None];
    let mut best = HashMap::new();
    let mut queue = BinaryHeap::from([Queue(0.0, 0)]);
    let mut expanded = 0;
    while let Some(Queue(_, index)) = queue.pop() {
        if cancelled() || expanded >= 1024 {
            return None;
        }
        let mut here = nodes[index].pose;
        let (x, y, heading) = positions[index];
        let yaw = initial_yaw + f64::from(heading) * std::f64::consts::PI / 16.0;
        let key = (x, y, heading, pose_key(here));
        if best
            .get(&key)
            .is_some_and(|&(cost, known)| known != index && cost <= nodes[index].cost)
        {
            continue;
        }
        if let Some((velocity, duration)) = pending_motions[index] {
            let from = nodes[nodes[index].parent?].pose;
            let Some(mut samples) = action(map, from, velocity, duration, boundary) else {
                continue;
            };
            let before = samples.iter().rev().nth(1).copied().unwrap_or(from);
            if map.validate_body_motion(before, here, boundary).is_err() {
                // 只有实际扫掠证明需要随地形改变站姿时，才提交对应三维终点。
                // 未检查的平面候选不能写入可达表，更不能直接返回给执行器。
                let projected=*samples.last()?;
                here=([here.0[0],here.0[1],projected.0[2]],projected.1);
                if map.validate_body_motion(before,here,boundary).is_err() {continue;}
                nodes[index].pose=here;
                nodes[index].cost=nodes[nodes[index].parent?].cost+position_distance(from,here)
                    +model.radius()*velocity.angular_z.abs()*duration+0.001;
                stances[index]=yaw_rotated(here.1,-f64::from(heading)*std::f64::consts::PI/16.0);
            }
            *samples.last_mut()? = here;
            nodes[index].samples = samples;
        }
        let key=(x,y,heading,pose_key(here));
        if best.get(&key).is_some_and(|&(cost,known)|known!=index && cost<=nodes[index].cost) {continue;}
        best.insert(key,(nodes[index].cost,index));
        if position_distance(here, reference) <= tolerance {
            let mut chain = Vec::new();
            let mut cursor = index;
            while let Some(parent) = nodes[cursor].parent {
                chain.push(cursor);
                cursor = parent;
            }
            let mut path = vec![start];
            for index in chain.into_iter().rev() {
                path.extend_from_slice(&nodes[index].samples);
            }
            #[cfg(test)]
            eprintln!(
                "全向平移搜索：展开{expanded}个状态，生成{}个节点",
                nodes.len()
            );
            return Some(path);
        }
        expanded += 1;
        // 大步优先推进，半体素小步保留窄口和短距离挪动的可达性。
        for (dx, dy) in [
            (8, 0), (8, 8), (0, 8), (-8, 8), (-8, 0), (-8, -8), (0, -8), (8, -8),
            (4, 0), (4, 4), (0, 4), (-4, 4), (-4, 0), (-4, -4), (0, -4), (4, -4),
            (2, 0),
            (2, 2),
            (0, 2),
            (-2, 2),
            (-2, 0),
            (-2, -2),
            (0, -2),
            (2, -2),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (0, -1),
            (1, -1),
        ] {
            let nx = x + dx;
            let ny = y + dy;
            let xy = [
                start.0[0] + f64::from(nx) * step,
                start.0[1] + f64::from(ny) * step,
            ];
            let delta = [xy[0] - here.0[0], xy[1] - here.0[1]];
            let velocity = Twist2 {
                linear: Point2::new(
                    delta[0] * yaw.cos() + delta[1] * yaw.sin(),
                    -delta[0] * yaw.sin() + delta[1] * yaw.cos(),
                ),
                angular_z: 0.0,
            };
            if !reverse && velocity.linear.x < -1e-9 {
                continue;
            }
            let target = ([xy[0], xy[1], here.0[2]], here.1);
            let key = (nx, ny, heading, pose_key(target));
            let cost = nodes[index].cost + position_distance(here, target) + 0.001;
            if best.get(&key).is_some_and(|&(known, _)| known <= cost) {
                continue;
            }
            let next = nodes.len();
            nodes.push(Node {
                pose: target,
                cost,
                parent: Some(index),
                samples: vec![],
            });
            positions.push((nx, ny, heading));
            stances.push(stances[index]);
            pending_motions.push(Some((velocity, 1.0)));
            queue.push(Queue(cost + 6.0 * heuristic(target), next));
        }
        for sign in [-4, -2, -1, 1, 2, 4] {
            if !rotate {
                break;
            }
            let next_heading = (heading + sign).rem_euclid(32);
            let target = (
                here.0,
                yaw_rotated(
                    stances[index],
                    f64::from(next_heading) * std::f64::consts::PI / 16.0,
                ),
            );
            let cost = nodes[index].cost
                + model.radius() * f64::from(sign).abs() * std::f64::consts::PI / 16.0
                + 0.001;
            let key = (x, y, next_heading, pose_key(target));
            if best.get(&key).is_some_and(|&(known, _)| known <= cost) {
                continue;
            }
            let next = nodes.len();
            nodes.push(Node {
                pose: target,
                cost,
                parent: Some(index),
                samples: vec![],
            });
            let turn = f64::from(sign) * std::f64::consts::PI / 16.0;
            positions.push((x, y, next_heading));
            stances.push(stances[index]);
            pending_motions.push(Some((
                Twist2 {
                    linear: Point2::new(0.0, 0.0),
                    angular_z: turn.signum(),
                },
                turn.abs(),
            )));
            queue.push(Queue(cost + 6.0 * heuristic(target), next));
        }
    }
    None
}

/// 复用生产机体规划查询；返回真实三维机体路径，不包含运动副作用。
pub fn plan(
    map: &SurfaceSnapshot,
    start: BodyPose,
    goal: BodyPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    boundary: &[[f64; 2]],
    budget: usize,
) -> Result<Vec<BodyPose>, String> {
    plan_cancellable(map, start, goal, options, limits, boundary, budget, None)
}

/// 后台规划按明确取消退出，不用墙钟截止替代任务生命周期。
#[allow(clippy::too_many_arguments)]
pub(crate) fn plan_cancellable(
    map: &SurfaceSnapshot,
    start: BodyPose,
    goal: BodyPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    boundary: &[[f64; 2]],
    budget: usize,
    cancelled: Option<&std::sync::atomic::AtomicBool>,
) -> Result<Vec<BodyPose>, String> {
    if let Some(limits)=limits {limits.validate()?;}
    let motion=limits.map(MotionLimits::kinematics);
    plan_with_motion_cancellable(map,start,goal,options,motion.as_ref(),boundary,budget,cancelled)
}

/// 规划只依赖执行器运动通道，保证制动参数仍由控制器单独消费。
#[allow(clippy::too_many_arguments)]
pub fn plan_with_motion_cancellable(
    map: &SurfaceSnapshot,
    start: BodyPose,
    mut goal: BodyPose,
    options: GoalOptions,
    limits: Option<&MotionKinematics>,
    boundary: &[[f64; 2]],
    budget: usize,
    cancelled: Option<&std::sync::atomic::AtomicBool>,
) -> Result<Vec<BodyPose>, String> {
    let cancelled = || cancelled.is_some_and(|flag|flag.load(std::sync::atomic::Ordering::Relaxed));
    #[cfg(test)] let diagnostic_started = std::time::Instant::now();
    if cancelled() { return Err("三维规划已取消".into()); }
    options.validate()?;
    if let Some(limits) = limits {
        limits.validate()?;
    }
    if !options.requires_orientation() { goal.1 = start.1; }
    map.validate_body_pose(start.0, start.1)
        .map_err(|e| format!("起点机体包络: {e}"))?;
    let model = map.body_envelope().ok_or("三维完整机体标定缺失")?;
    let rotate = limits.is_none_or(|m| m.can_rotate_in_place);
    let reverse = options.allow_reverse && limits.is_some_and(|m| m.max_reverse_mps > 0.0);
    if options.allow_reverse && !reverse {
        return Err("倒车缺少标定".into());
    }
    let omni =
        limits.is_some_and(|m| m.model == MotionModel::Omnidirectional && m.max_lateral_mps > 0.0);
    let requested=goal;
    let original_tolerance=options.position_tolerance_m;
    let prepared=prepare_goal_pose(map,goal,options.requires_orientation());
    let reference=prepared.as_ref().copied().unwrap_or(requested);
    if let Ok(centre)=prepared.as_ref() {
        goal=*centre;
        if let Some(path)=connect(map,start,goal,boundary,rotate,reverse,omni,options.requires_orientation()) {return Ok(path);}
    }
    let mut regional_targets: Vec<_> = prepared.as_ref().copied().into_iter().collect();
    let mut region_seen=HashSet::new();
    let mut region_input_seen=HashSet::new();
    for candidate in goal_region_samples(map,start,reference,original_tolerance) {
        if cancelled() {return Err("三维规划已取消".into());}
        // 位置目标的连接最终按同一支撑层重新投影。同一水平落点、同一层
        // 的不同高度采样会产生相同连接，不反复进行32方向机体拟合。
        if !options.requires_orientation() {
            let Ok(layer)=map.locate_goal_support_from_body(candidate.0,candidate.1,model.body_to_support_z_m) else {continue;};
            if !region_input_seen.insert((candidate.0[0].to_bits(),candidate.0[1].to_bits(),layer.cell)) {continue;}
        }
        let Ok(target)=prepare_goal_pose(map,candidate,options.requires_orientation()) else {continue;};
        if position_distance(target,reference)>=original_tolerance-1e-6 {continue;}
        let key=[target.0[0].to_bits(),target.0[1].to_bits(),target.0[2].to_bits(),
            target.1[0].to_bits(),target.1[1].to_bits(),target.1[2].to_bits(),target.1[3].to_bits()];
        if !region_seen.insert(key) {continue;}
        regional_targets.push(target);
        if let Some(path)=connect(map,start,target,boundary,rotate,reverse,omni,options.requires_orientation()) {
            if path.last().is_some_and(|end|position_distance(*end,reference)<=original_tolerance) {return Ok(path);}
        }
    }
    if regional_targets.is_empty() {
        return Err(format!("{}；目标容差区域没有已观测的可通行机体落点",prepared.unwrap_err()));
    }
    #[cfg(test)] eprintln!("目标区域准备 {:?}，有效落点{}", diagnostic_started.elapsed(), regional_targets.len());
    if cancelled() {return Err("三维规划已取消".into());}
    // 直连失败后，以原目标区域内真实连通的落点引导绕行，不能只追踪孤立的中心或首样本。
    // 一次有向支撑图查询覆盖全部落点；点可达仍须通过后续完整机体扫掠。
    let start_support=map.locate_support_from_body(start.0,start.1,model.body_to_support_z_m)
        .map_err(|error|format!("起点支撑层: {error}"))?;
    let targets:Vec<_>=regional_targets.iter().map(|target| {
        map.locate_goal_support_from_body(target.0,target.1,model.body_to_support_z_m)
            .map(|location|location.support_xyz)
    }).collect::<Result<_,_>>().map_err(|error|format!("目标支撑层: {error}"))?;
    let costs=map.frontier_travel_costs(start_support.support_xyz,&targets,boundary)
        .map_err(|error|format!("目标区域支撑连通查询: {error}"))?;
    if cancelled() {return Err("三维规划已取消".into());}
    // 原首选落点（有效中心或首个替代点）已连通时继续复用，只有孤立才换引导点。
    // 支撑图只排序引导点；未找到连通样本仍交给完整机体搜索，不能新增点图硬准入。
    let reachable=if costs[0].is_finite() {0} else {
        costs.iter().enumerate().filter(|(_,cost)|cost.is_finite())
            .min_by(|(_,a),(_,b)|a.total_cmp(b)).map_or(0,|(index,_)|index)
    };
    goal=regional_targets[reachable];
    // 引导点可以偏移，终点验收始终使用原容差球，不叠加也不缩小位置容差。
    let endpoint_allowed=|pose:BodyPose|position_distance(pose,reference)<=original_tolerance;
    let goal_costs = map.goal_travel_costs(&targets,boundary)
        .map_err(|error|format!("目标支撑距离场: {error}"))?;
    if budget>0 && omni && !options.requires_orientation() {
        if let Some(path)=omnidirectional_connection(map,start,reference,original_tolerance,boundary,reverse,rotate,&goal_costs,&cancelled) {
            return Ok(path);
        }
    }
    // 起步盲区/近墙常使原地转向不可行。先尝试沿当前可执行方向
    // 连续驶出一个机身长度，再连接目标，避免细角度搜索反复试同一转弯。
    let probe_step = (model.radius()*0.5).max(f64::from(map.config().voxel_size)*2.0);
    for sign in [1.0,-1.0] {
        if sign<0.0 && !reverse {continue;}
        for step in 1..=5 {
            if cancelled() {return Err("三维规划已取消".into());}
            let Some(prefix)=action(map,start,Twist2 {linear:Point2::new(sign,0.0),angular_z:0.0},
                probe_step*f64::from(step),boundary) else {break;};
            let from=*prefix.last().expect("前进基元");
            if let Some(tail)=connect(map,from,goal,boundary,rotate,reverse,omni,options.requires_orientation())
                .filter(|path|path.last().is_some_and(|pose|endpoint_allowed(*pose))) {
                let mut path=vec![start];path.extend(prefix);path.extend(tail.into_iter().skip(1));return Ok(path);
            }
        }
    }
    // 后方目标先尝试连续圆弧驶离起步足迹，避免把可行的大转弯拆成
    // 数百次小角度状态扩展。圆弧仍沿同一投影和完整扫掠验证。
    let turn_radius = limits.map_or(0.0, |m| m.minimum_turning_radius_m)
        .max(model.radius()).max(0.2);
    for radius in [turn_radius, turn_radius * 2.0] {
        for direction in [-1.0, 1.0] {
            for step in 1..=4 {
                if cancelled() {return Err("三维规划已取消".into());}
                let duration = radius * f64::from(step) * std::f64::consts::FRAC_PI_4;
                let Some(prefix) = action(map, start, Twist2 {
                    linear: Point2::new(1.0, 0.0), angular_z: direction / radius,
                }, duration, boundary) else {break;};
                let from = *prefix.last().expect("圆弧基元");
                let tail = if endpoint_allowed(from) && (!options.requires_orientation()
                    || angle(planar(from).yaw-planar(goal).yaw).abs() <= options.yaw_tolerance_rad) {
                    Some(vec![from])
                } else {
                    connect(map,from,goal,boundary,rotate,reverse,omni,options.requires_orientation())
                };
                if let Some(tail) = tail.filter(|path|path.last().is_some_and(|pose|endpoint_allowed(*pose))) {
                    let mut path=vec![start];path.extend(prefix);path.extend(tail.into_iter().skip(1));return Ok(path);
                }
            }
        }
    }
    // 先利用已维护的支撑图生成引导线，再逐段核对真实机体及转向。
    // 开阔地和普通绕行无需展开完整的32方向状态空间；引导线不通过
    // 机体扫掠时仍进入下方完整搜索，不把点可达当作机体可达。
    if let (Ok(a), Ok(b)) = (
        map.locate_support_from_body(start.0, start.1, model.body_to_support_z_m),
        map.locate_goal_support_from_body(goal.0, goal.1, model.body_to_support_z_m),
    ) {
        if let Ok(guide) = map.plan_in_boundary(a.support_xyz, b.support_xyz, boundary) {
            let mut path = vec![start];
            let mut valid = true;
            for (index, support) in guide.iter().enumerate().skip(1) {
                if cancelled() { return Err("三维规划已取消".into()); }
                let last = *path.last().expect("起点");
                let final_segment = index + 1 == guide.len();
                let yaw = if final_segment && options.requires_orientation() { planar(goal).yaw }
                    else { (support[1]-last.0[1]).atan2(support[0]-last.0[0]) };
                let Ok(target) = map.body_pose_on_surface([support[0],support[1]],yaw,*support) else { valid=false;break; };
                if let Some(segment) = connect(map,last,target,boundary,rotate,reverse,omni,
                    final_segment && options.requires_orientation()) {
                    path.extend(segment.into_iter().skip(1));
                } else { valid=false;break; }
            }
            if valid && path.len()>1 && path.last().is_some_and(|pose|endpoint_allowed(*pose)) { return Ok(path); }
        }
    }
    let resolution = f64::from(map.config().voxel_size);
    #[cfg(test)] eprintln!("进入朝向搜索 {:?}", diagnostic_started.elapsed());
    let yaw_step = std::f64::consts::TAU / 32.0;
    let radius = limits
        .map_or(0.0, |m| m.minimum_turning_radius_m)
        .max(model.radius())
        .max(0.2);
    let length = resolution * 2.0;
    let key = |p: BodyPose| {
        let s = map
            .locate_support_from_body(p.0, p.1, model.body_to_support_z_m)
            .ok()?;
        Some((
            s.cell,
            ((planar(p).yaw / yaw_step).round() as i32).rem_euclid(32),
        ))
    };
    let distance = |p: BodyPose| {
        (p.0[0] - goal.0[0])
            .hypot(p.0[1] - goal.0[1])
            .hypot(p.0[2] - goal.0[2])
    };
    // 只看直线距离会反复扩展朝目标背面行走的状态；把必要转向纳入
    // 同一米制代价。全向运动保留横移能力，不人为增加转向要求。
    let estimate = |p:BodyPose| {
        let d=distance(p);
        let heading=(goal.0[1]-p.0[1]).atan2(goal.0[0]-p.0[0]);
        let forward=angle(heading-planar(p).yaw).abs();
        let turn=if omni {0.0} else if reverse {forward.min(angle(heading+std::f64::consts::PI-planar(p).yaw).abs())} else {forward};
        // 最终朝向目标的转向代价绑定选定落点，不能混入另一个区域落点的距离。
        let graph_distance=if options.requires_orientation() {d} else {
            map.locate_support_from_body(p.0,p.1,model.body_to_support_z_m)
                .ok().and_then(|location|goal_costs.cost(location.cell)).unwrap_or(d)
        };
        d.max(graph_distance) + 2.0*model.radius()*turn*(d/options.position_tolerance_m.max(0.1)).min(1.0)
    };
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        samples: vec![],
    }];
    let mut best = HashMap::from([(key(start).ok_or("起点支撑缺失")?, (0.0, 0))]);
    let mut queue = BinaryHeap::from([Queue(estimate(start)*SEARCH_HEURISTIC_WEIGHT, 0)]);
    let mut expanded = 0;
    while let Some(Queue(_, index)) = queue.pop() {
        if cancelled() { return Err("三维规划已取消".into()); }
        let here = nodes[index].pose;
        let cost = nodes[index].cost;
        if best
            .get(&key(here).ok_or("搜索状态支撑缺失")?)
            .is_none_or(|&(_, known)| known != index)
        {
            continue;
        }
        expanded += 1;
        if expanded > budget || nodes.len() > budget * 4 {
            return Err("三维朝向搜索预算耗尽".into());
        }
        let close = endpoint_allowed(here)
            && (!options.requires_orientation() || angle(planar(here).yaw - planar(goal).yaw).abs()
                <= options.yaw_tolerance_rad.min(yaw_step).max(1e-6));
        let tail = if close {
            Some(vec![here])
        } else if distance(here) < length * 3.0 || expanded % 8 == 0 {
            connect(map, here, goal, boundary, rotate, reverse, omni, options.requires_orientation())
        } else {
            None
        };
        if let Some(tail) = tail.filter(|path|path.last().is_some_and(|pose|endpoint_allowed(*pose))) {
            let mut chain = vec![];
            let mut cursor = index;
            while let Some(parent) = nodes[cursor].parent {
                chain.push(cursor);
                cursor = parent;
            }
            chain.reverse();
            let mut path = vec![start];
            for i in chain {
                path.extend_from_slice(&nodes[i].samples);
            }
            path.extend(tail.into_iter().skip(1));
            #[cfg(test)] eprintln!("完整机体搜索：展开{expanded}个状态，生成{}个节点",nodes.len());
            return Ok(path);
        }
        let mut actions = vec![];
        for sign in [1.0, -1.0] {
            if sign < 0.0 && !reverse {
                continue;
            }
            for curvature in [0.0, -1.0 / radius, 1.0 / radius] {
                actions.push((
                    Twist2 {
                        linear: Point2::new(sign, 0.0),
                        angular_z: sign * curvature,
                    },
                    if curvature == 0.0 {
                        length
                    } else {
                        radius * yaw_step
                    },
                ));
            }
        }
        if rotate {
            for sign in [-1.0, 1.0] {
                actions.push((
                    Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: sign,
                    },
                    yaw_step,
                ));
            }
        }
        if omni {
            for sign in [-1.0, 1.0] {
                actions.push((
                    Twist2 {
                        linear: Point2::new(0.0, sign),
                        angular_z: 0.0,
                    },
                    length,
                ));
            }
        }
        if distance(here) < radius + length * 3.0 {
            let delta = angle(planar(goal).yaw - planar(here).yaw).clamp(-yaw_step, yaw_step);
            if options.requires_orientation() && delta.abs() > 1e-4 {
                actions.push((
                    Twist2 {
                        linear: Point2::new(1.0, 0.0),
                        angular_z: delta.signum() / radius,
                    },
                    radius * delta.abs(),
                ));
            }
            actions.push((
                Twist2 {
                    linear: Point2::new(1.0, 0.0),
                    angular_z: 0.0,
                },
                distance(here).clamp(resolution * 0.25, length),
            ));
        }
        for (velocity, duration) in actions {
            let Some(samples) = action(map, here, velocity, duration, boundary) else {
                continue;
            };
            let Some(&end) = samples.last() else {
                continue;
            };
            let Some(k) = key(end) else {
                continue;
            };
            if Some(k) == key(here) {
                continue;
            }
            let next = cost
                + velocity.linear.x.hypot(velocity.linear.y) * duration
                + model.radius() * velocity.angular_z.abs() * duration
                + 0.001;
            if best.get(&k).is_some_and(|&(old, _)| old <= next) {
                continue;
            }
            let id = nodes.len();
            nodes.push(Node {
                pose: end,
                cost: next,
                parent: Some(index),
                samples,
            });
            best.insert(k, (next, id));
            queue.push(Queue(next + estimate(end) * SEARCH_HEURISTIC_WEIGHT, id));
        }
    }
    Err("不存在满足完整机体与运动约束的三维路线".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope};
    use nuwax_nav_runtime::{
        ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, SurfaceFollower,
        SurfaceFollowerConfig,
    };
    use nuwax_nav_types_messages::{Path3D, Point3, Pose3, PoseStamped, Quaternion, Timestamp};
    use std::sync::Arc;
    fn scene(obstacle: bool) -> SurfaceSnapshot {
        let mut points: Vec<_> = (-35..35)
            .flat_map(|x| {
                (-20..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
            })
            .collect();
        if obstacle {
            for x in -2..2 {
                for y in -2..2 {
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.55));
                }
            }
        }
        let cfg = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.35,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.1,
            wall_buffer_m: 0.0,
            wall_buffer_weight: 0.0,
            max_slope_deg: 20.0,
            step_threshold_m: 0.2,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        SurfaceSnapshot::from_points(&points, cfg, 0.1)
            .unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.3, -0.12], [0.3, -0.12], [0.3, 0.12], [-0.3, 0.12]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 15.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap()
    }
    fn pose(x: f64, y: f64, yaw: f64) -> BodyPose {
        (
            [x, y, 0.5],
            [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()],
        )
    }
    fn stamped(p: BodyPose, nanos: i64) -> PoseStamped {
        PoseStamped {
            frame_id: nuwax_nav_types_messages::NavFrameId::Map,
            stamp: Timestamp::from_unix(nanos / 1_000_000_000, (nanos % 1_000_000_000) as u32),
            pose: Pose3 {
                position: Point3::new(p.0[0], p.0[1], p.0[2]),
                orientation: Quaternion {
                    x: p.1[0],
                    y: p.1[1],
                    z: p.1[2],
                    w: p.1[3],
                },
            },
        }
    }
    fn limits() -> MotionLimits {
        MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.3,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.7,
            linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0,
            angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0,
            reaction_time_sec: 0.1,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        }
    }
    #[test]
    fn short_escape_before_rotation_is_not_consumed_by_position_tolerance() {
        let base = scene(false);
        let mut points: Vec<_> = (-20..20)
            .flat_map(|x| {
                (-20..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
            })
            .collect();
        points.push((-0.35, 0.05, 0.55));
        let mut envelope = base.body_envelope().unwrap().clone();
        envelope.footprint_xy_m = vec![[-0.29, -0.12], [0.29, -0.12], [0.29, 0.12], [-0.29, 0.12]];
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1)
                .unwrap()
                .with_body_envelope(envelope)
                .unwrap(),
        );
        let mut current = pose(0.0, 0.0, 0.0);
        let goal = pose(0.06, 0.8, std::f64::consts::FRAC_PI_2);
        let route = vec![
            current,
            pose(0.06, 0.0, 0.0),
            pose(0.06, 0.0, std::f64::consts::FRAC_PI_2),
            goal,
        ];
        assert!(map
            .validate_body_motion(current, pose(0.0, 0.0, std::f64::consts::FRAC_PI_2), &[])
            .is_err());
        for edge in route.windows(2) {
            map.validate_body_motion(edge[0], edge[1], &[]).unwrap();
        }
        let options = GoalOptions {
            position_tolerance_m: 0.12,
            yaw_tolerance_rad: std::f64::consts::PI,
            ..Default::default()
        };
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.speed = 0.3;
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        let mut now = 1_000_000_000;
        follower
            .configure_goal(&stamped(goal, now), options)
            .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map.clone()))
            .unwrap();
        follower.update_odometry(stamped(current, now));
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: route.into_iter().map(|p| stamped(p, now)).collect(),
                }),
                None,
                stamped(goal, now),
            )
            .unwrap();
        let mut reached = false;
        for tick in 0..600 {
            match follower.step(0.3) {
                PlannerStep::Command(command) => {
                    if tick == 0 {
                        assert!(
                            command.linear.x > 0.0,
                            "必须先平移腾出转向净空: {command:?}"
                        );
                    }
                    follower.observe_applied_command(command);
                    let next = integrate(planar(current), command, 0.05);
                    current = pose(next.x, next.y, next.yaw);
                }
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                other => panic!("短距离脱离转向不能卡死: {other:?}, pose={current:?}"),
            }
            now += 50_000_000;
            follower.update_odometry(stamped(current, now));
        }
        assert!(reached);
    }

    #[test]
    fn declared_basic_omnidirectional_channels_back_out_when_turning_is_blocked() {
        let base = scene(false);
        let mut points: Vec<_> = (-25..25)
            .flat_map(|x| {
                (-3..3).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
            })
            .collect();
        for x in -25..25 {
            for y in [-0.25, 0.25] {
                points.push(((x as f32 + 0.5) * 0.1, y, 0.55));
            }
        }
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap(),
        );
        let mut current = pose(0.5, 0.0, 0.0);
        let goal = pose(-0.5, 0.0, 0.0);
        assert!(map
            .validate_body_motion(current, pose(0.5, 0.0, std::f64::consts::PI), &[])
            .is_err());
        let motion = MotionKinematics {
            model: MotionModel::Omnidirectional,
            max_forward_mps: 0.2,
            max_reverse_mps: 0.1,
            max_lateral_mps: 0.08,
            max_angular_rps: 0.5,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        };
        let options = GoalOptions {
            allow_reverse: true,
            position_tolerance_m: 0.12,
            yaw_tolerance_rad: std::f64::consts::PI,
        };
        let route =
            plan_with_motion_cancellable(&map, current, goal, options, Some(&motion), &[], 0, None)
                .unwrap();
        assert!(route.iter().all(|p| angle(planar(*p).yaw).abs() < 1e-8));
        for edge in route.windows(2) {
            map.validate_body_motion(edge[0], edge[1], &[]).unwrap();
        }
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.basic_kinematics = Some(motion);
        config.controller.control_frequency = 20.0;
        assert!(config.motion_limits.is_none(), "基本通道不冒充保证制动标定");
        let mut follower = SurfaceFollower::new(config).unwrap();
        let mut now = 1_000_000_000;
        follower
            .configure_goal(&stamped(goal, now), options)
            .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map.clone()))
            .unwrap();
        follower.update_odometry(stamped(current, now));
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: route.into_iter().map(|p| stamped(p, now)).collect(),
                }),
                None,
                stamped(goal, now),
            )
            .unwrap();
        let mut reached = false;
        for tick in 0..800 {
            match follower.step(0.3) {
                PlannerStep::Command(command) => {
                    if tick == 0 {
                        assert!(command.linear.x < 0.0);
                    }
                    assert!(
                        command.linear.x >= -0.1 - 1e-9
                            && command.linear.y.abs() <= 0.08 + 1e-9
                            && command.angular_z.abs() <= 0.5 + 1e-9
                    );
                    follower.observe_applied_command(command);
                    let next = integrate(planar(current), command, 0.05);
                    current = pose(next.x, next.y, next.yaw);
                }
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                other => panic!("基本全向通道应能沿真实窄通道倒退: {other:?}"),
            }
            now += 50_000_000;
            follower.update_odometry(stamped(current, now));
        }
        assert!(reached);
        let mut calibrated = SurfaceFollowerConfig::new(0.4);
        calibrated.basic_kinematics = Some(motion);
        let mut disabled = limits();
        disabled.max_reverse_mps = 0.0;
        calibrated.motion_limits = Some(disabled);
        assert!(
            SurfaceFollower::new(calibrated)
                .unwrap()
                .configure_goal(&stamped(goal, now), options)
                .is_err(),
            "显式完整标定关闭倒退时，基本通道不能覆盖它"
        );
    }

    #[test]
    fn surface_height_correction_does_not_turn_rotation_into_an_unreachable_vertical_move() {
        let base = scene(false);
        let make_map = |height: f32| {
            let points: Vec<_> = (-20..20)
                .flat_map(|x| {
                    (-20..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, height))
                })
                .collect();
            Arc::new(
                SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
                    .unwrap()
                    .with_body_envelope(base.body_envelope().unwrap().clone())
                    .unwrap(),
            )
        };
        let original = make_map(0.125);
        let corrected = make_map(0.075);
        let mut current = pose(-0.5, 0.0, 0.0);
        let goal = pose(-0.5, 0.8, std::f64::consts::FRAC_PI_2);
        let mut turn = pose(-0.5, 0.0, std::f64::consts::FRAC_PI_2);
        turn.0[2] = 0.6;
        let mut endpoint = goal;
        endpoint.0[2] = 0.6;
        let route = vec![current, turn, endpoint];
        for edge in route.windows(2) {
            original
                .validate_body_motion(edge[0], edge[1], &[])
                .unwrap();
        }
        assert!(corrected
            .locate_support_from_body(turn.0, turn.1, 0.4)
            .is_err());
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        let mut now = 1_000_000_000;
        follower
            .configure_goal(
                &stamped(goal, now),
                GoalOptions {
                    position_tolerance_m: 0.12,
                    yaw_tolerance_rad: 0.15,
                    ..Default::default()
                },
            )
            .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(original))
            .unwrap();
        follower.update_odometry(stamped(current, now));
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: route.into_iter().map(|p| stamped(p, now)).collect(),
                }),
                None,
                stamped(goal, now),
            )
            .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(corrected))
            .unwrap();
        let mut reached = false;
        for _ in 0..800 {
            match follower.step(0.3) {
                PlannerStep::Command(command) => {
                    follower.observe_applied_command(command);
                    let next = integrate(planar(current), command, 0.05);
                    current = pose(next.x, next.y, next.yaw);
                }
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                other => panic!("同一平面支撑高度修正不能让平面执行器追逐竖直误差: {other:?}"),
            }
            now += 50_000_000;
            follower.update_odometry(stamped(current, now));
        }
        assert!(reached);
    }

    #[test]
    fn regional_path_endpoint_waits_until_actual_pose_enters_requested_goal_region() {
        let map = Arc::new(scene(false));
        let mut current = pose(0.0, 0.0, 0.0);
        let goal = pose(1.0, 0.0, 0.0);
        let endpoint = pose(0.66, 0.0, 0.0);
        let options = GoalOptions {
            position_tolerance_m: 0.35,
            yaw_tolerance_rad: std::f64::consts::PI,
            ..Default::default()
        };
        assert!(position_distance(endpoint, goal) < options.position_tolerance_m);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        let mut now = 1_000_000_000;
        follower
            .configure_goal(&stamped(goal, now), options)
            .unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map))
            .unwrap();
        follower.update_odometry(stamped(current, now));
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![stamped(current, now), stamped(endpoint, now)],
                }),
                None,
                stamped(goal, now),
            )
            .unwrap();
        let mut reached = false;
        for _ in 0..800 {
            match follower.step(0.3) {
                PlannerStep::Command(command) => {
                    follower.observe_applied_command(command);
                    let next = integrate(planar(current), command, 0.05);
                    current = pose(next.x, next.y, next.yaw);
                }
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                other => panic!("不能在到达区域外提前消费替代终点: {other:?}, pose={current:?}"),
            }
            now += 50_000_000;
            follower.update_odometry(stamped(current, now));
        }
        assert!(reached && position_distance(current, goal) < options.position_tolerance_m);
    }

    #[test]
    fn blocked_translation_replans_instead_of_emitting_aligned_tiny_rotations() {
        let clear=Arc::new(scene(false));let blocked=Arc::new(scene(true));
        let start=pose(-0.51,0.05,0.0);let goal=pose(0.55,0.05,0.001);
        let motion=MotionKinematics {model:MotionModel::Omnidirectional,max_forward_mps:0.4,
            max_reverse_mps:0.3,max_lateral_mps:0.3,max_angular_rps:0.6,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true};
        let mut config=SurfaceFollowerConfig::new(0.4);config.basic_kinematics=Some(motion);
        config.controller.speed=0.4;config.controller.control_frequency=20.0;
        let mut follower=SurfaceFollower::new(config).unwrap();let now=1_000_000_000;
        follower.configure_goal(&stamped(goal,now),GoalOptions {position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,..Default::default()}).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();follower.update_odometry(stamped(start,now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![stamped(start,now),stamped(goal,now)]}),None,stamped(goal,now)).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
        assert!(matches!(follower.step(0.3),PlannerStep::ReplanRequired(_)),
            "朝向已对齐，所有平移都被新障碍阻挡时必须重规划");
    }

    #[test]
    fn inserted_obstacle_replans_around_it_and_continues_to_original_position() {
        let clear=Arc::new(scene(false));let blocked=Arc::new(scene(true));
        let mut current=pose(-1.55,0.05,0.0);let goal=pose(1.55,0.05,2.0);
        let options=GoalOptions {position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        let motion=limits();let mut config=SurfaceFollowerConfig::new(0.4);
        config.motion_limits=Some(motion.clone());config.controller.control_frequency=20.0;
        let mut follower=SurfaceFollower::new(config).unwrap();
        let mut now=1_000_000_000;let mut map=clear;
        follower.configure_goal(&stamped(goal,now),options).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();
        follower.update_odometry(stamped(current,now));
        let initial=plan(&map,current,goal,options,Some(&motion),&[],8000).unwrap();
        assert!(initial.iter().all(|p|(p.0[1]-0.05).abs()<0.01));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:initial.iter().map(|p|stamped(*p,now)).collect()}),None,stamped(goal,now)).unwrap();
        let mut inserted=false;let mut reached=false;let mut maximum_lateral=0.0_f64;let mut replans=0;
        for _ in 0..2500 {
            if !inserted && current.0[0]>-1.2 {
                inserted=true;map=blocked.clone();
                follower.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();
                let started=std::time::Instant::now();
                let detour=plan(&map,current,goal,options,Some(&motion),&[],8000).unwrap();
                eprintln!("动态障碍：重新规划耗时 {:?}，路径{}点",started.elapsed(),detour.len());
                assert!(detour.iter().any(|p|p.0[1].abs()>0.3));
                follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:detour.into_iter().map(|p|stamped(p,now)).collect()}),None,stamped(goal,now)).unwrap();
            }
            match follower.step(0.24) {
                PlannerStep::Command(command)=>{
                    let next=integrate(planar(current),command,0.05);
                    let to=pose(next.x,next.y,next.yaw);
                    map.validate_body_motion(current,to,&[]).expect("动态障碍闭环不能穿过障碍");
                    current=to;maximum_lateral=maximum_lateral.max(current.0[1].abs());
                }
                PlannerStep::Succeeded=>{reached=true;break;}
                PlannerStep::ReplanRequired(reason)=>{
                    replans+=1;assert!(replans<=8,"局部重规划不能无限循环: {reason}, pose={current:?}");
                    // 按生产拥有方顺序：停止旧意图，经过已标定制动窗口，再从
                    // 实际位置提交新路径；测试的运动学模型在零速后保持位置。
                    follower.stop();std::thread::sleep(std::time::Duration::from_secs_f64(motion.maximum_stopping_time_sec()));
                    let detour=plan(&map,current,goal,options,Some(&motion),&[],8000).unwrap();
                    follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:detour.into_iter().map(|p|stamped(p,now)).collect()}),None,stamped(goal,now)).unwrap();
                }
                other=>panic!("动态障碍后执行中断: {other:?}, pose={current:?}"),
            }
            now+=50_000_000;follower.update_odometry(stamped(current,now));
        }
        assert!(inserted && reached && maximum_lateral>0.3,"必须绕行并到达原目标位置");
        eprintln!("动态障碍到达：局部接续{replans}次，最大侧移{maximum_lateral}米");
    }

    #[test]
    fn observed_goal_region_can_arrive_when_exact_clicked_column_is_missing() {
        let base=scene(false);
        let points:Vec<_>=(-35..35).flat_map(|x|(-20..20).filter_map(move |y| {
            (x!=5||y!=0).then_some(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))
        })).collect();
        let map=Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        let start=pose(-0.55,0.05,0.0);let goal=pose(0.55,0.05,2.0);
        let options=GoalOptions {position_tolerance_m:0.35,yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        assert!(map.locate_goal_support_from_body(goal.0,goal.1,0.4).is_err());
        let motion=limits();
        let path=plan(&map,start,goal,options,Some(&motion),&[],8000).expect("容差区域有真实地面可达");
        assert!(position_distance(*path.last().unwrap(),goal)<=options.position_tolerance_m);
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        let mut config=SurfaceFollowerConfig::new(0.4);config.motion_limits=Some(motion);
        config.controller.speed=0.35;config.controller.control_frequency=20.0;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(stamped(start,1_000_000_000));
        follower.configure_goal(&stamped(goal,1_000_000_000),options).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:path.into_iter().map(|p|stamped(p,1_000_000_000)).collect()}),None,stamped(goal,1_000_000_000)).unwrap();
        let mut current=start;let mut reached=false;
        for tick in 1..600 {
            follower.update_odometry(stamped(current,1_000_000_000+tick*50_000_000));
            match follower.step(0.3) {
                PlannerStep::Succeeded=>{reached=true;break;},
                PlannerStep::Command(command)=>{
                    follower.observe_applied_command(command);
                    let next=nuwax_nav_control::integrate(planar(current),command,0.05);
                    let next=pose(next.x,next.y,next.yaw);
                    map.validate_body_motion(current,next,&[]).unwrap();current=next;
                },
                other=>panic!("目标区域执行中断：{other:?}"),
            }
        }
        assert!(reached && position_distance(current,goal)<0.35);
    }

    #[test]
    fn reachable_centre_keeps_original_detour_endpoint() {
        let base = scene(true);
        let mut envelope = base.body_envelope().unwrap().clone();
        envelope.footprint_xy_m = vec![[-0.06,-0.06],[0.06,-0.06],[0.06,0.06],[-0.06,0.06]];
        let map = base.with_body_envelope(envelope).unwrap();
        let start = pose(-1.55,0.05,0.0);
        let target = pose(1.55,0.05,0.0);
        let options = GoalOptions {position_tolerance_m:0.35,yaw_tolerance_rad:0.05,..Default::default()};
        assert!(connect(&map,start,target,&[],true,false,false,true).is_none());
        // 中心可达且可完整绕障，不应无理由改成较近的偏移落点。
        let path = plan(&map,start,target,options,Some(&limits()),&[],300).unwrap();
        assert!(position_distance(*path.last().unwrap(),target)<1e-6,
            "保持正常中心绕行的终点，实际 {:?}",path.last().unwrap());
        assert!(path.iter().any(|p|p.0[1].abs()>0.2));
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    }

    #[test]
    fn goal_region_detours_to_reachable_support_instead_of_isolated_first_sample() {
        let base = scene(false);
        let mut points: Vec<_> = (-35..35).flat_map(|x| (-20..20).filter_map(move |y| {
            let moat = (3..21).contains(&x) && (-5..5).contains(&y);
            let island = (5..19).contains(&x) && (-3..3).contains(&y);
            (!moat || island).then_some(((x as f32 + 0.5)*0.1, (y as f32 + 0.5)*0.1, 0.025))
        })).collect();
        for x in -1..1 { for y in -10..10 {
            points.push(((x as f32 + 0.5)*0.1, (y as f32 + 0.5)*0.1, 0.55));
        }}
        let mut envelope = base.body_envelope().unwrap().clone();
        envelope.footprint_xy_m = vec![[-0.1,-0.08],[0.1,-0.08],[0.1,0.08],[-0.1,0.08]];
        let map = SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
            .with_body_envelope(envelope).unwrap();
        let start = pose(-1.55,0.05,0.0);
        let target = pose(1.95,0.05,0.0);
        let options = GoalOptions {position_tolerance_m:0.75,yaw_tolerance_rad:0.05,..Default::default()};
        let mut motion = limits();
        motion.model = MotionModel::Omnidirectional;
        motion.max_lateral_mps = 0.5;
        assert!(prepare_goal_pose(&map,target,true).is_err());
        let samples: Vec<_> = goal_region_samples(&map,start,target,options.position_tolerance_m)
            .into_iter().filter_map(|sample|prepare_goal_pose(&map,sample,true).ok())
            .filter(|sample|position_distance(*sample,target)<options.position_tolerance_m).collect();
        assert!(!samples.is_empty());
        assert!(samples.iter().all(|sample|connect(&map,start,*sample,&[],true,false,true,true).is_none()));
        let first = samples[0];
        let first_support = map.locate_support_from_body(first.0,first.1,0.4).unwrap();
        let start_support = map.locate_support_from_body(start.0,start.1,0.4).unwrap();
        assert!(map.plan_in_boundary(start_support.support_xyz,first_support.support_xyz,&[]).is_err(),
            "旧策略选中的首个可站立落点位于真实支撑孤岛");
        // 独立指定的绕行路线逐段通过正式连接器，证明容差区域内确实可达。
        let route = [start,pose(-1.55,1.35,0.0),pose(2.35,1.35,0.0),pose(2.35,0.05,0.0)];
        for pair in route.windows(2) {
            assert!(connect(&map,pair[0],pair[1],&[],true,false,true,true).is_some());
        }
        assert!(position_distance(*route.last().unwrap(),target)<options.position_tolerance_m);
        let path = plan(&map,start,target,options,Some(&motion),&[],300).expect("应绕过墙体到达容差内连通落点");
        assert!(position_distance(*path.last().unwrap(),target)<options.position_tolerance_m);
        assert!(angle(planar(*path.last().unwrap()).yaw).abs()<options.yaw_tolerance_rad);
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        let boundary = [[-3.0,-0.8],[3.0,-0.8],[3.0,0.8],[-3.0,0.8]];
        assert!(plan(&map,start,target,options,Some(&motion),&boundary,300).is_err(),
            "边界截断完整机体绕行时不能把支撑点连通当作机体可达");
    }

    #[test]
    fn sub_voxel_goal_tolerance_finds_collision_free_endpoint() {
        let map = scene(true);
        let start = pose(-1.05, 0.05, 0.0);
        // 障碍左沿在 -0.2 米，机身半长 0.3 米；点击仅侵入 1 厘米。
        let target = pose(-0.49, 0.05, 0.0);
        let options = GoalOptions {
            position_tolerance_m: 0.04,
            yaw_tolerance_rad: 0.05,
            ..Default::default()
        };
        assert!(map.validate_body_pose(target.0, target.1).is_err());
        map.validate_body_pose(pose(-0.52, 0.05, 0.0).0, target.1).unwrap();
        let path = plan(&map, start, target, options, Some(&limits()), &[], 2000)
            .expect("小于规划格的容差内仍存在无碰撞落点");
        assert!(position_distance(*path.last().unwrap(), target) < options.position_tolerance_m);
        for pair in path.windows(2) {
            map.validate_body_motion(pair[0], pair[1], &[]).unwrap();
        }
    }

    #[test]
    fn goal_region_keeps_support_real_when_body_height_has_changed() {
        let map=scene(false);
        let start=pose(-0.55,0.05,0.0);
        let mut goal=pose(0.55,0.05,0.0);goal.0[2]=0.18;
        let options=GoalOptions {position_tolerance_m:0.35,yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        assert!(map.locate_goal_support_from_body(goal.0,goal.1,0.4).is_err());
        let path=plan(&map,start,goal,options,Some(&limits()),&[],8000).unwrap();
        assert!(position_distance(*path.last().unwrap(),goal)<0.35);
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        goal.0[2]=3.5;
        assert!(plan(&map,start,goal,options,Some(&limits()),&[],8000).is_err(),"不把另一个高度层降到当前地面");
    }

    #[test]
    fn behind_goal_tries_other_in_place_turn_when_shortest_sweep_is_blocked() {
        let base=scene(false);
        let mut points:Vec<_>=(-35..35).flat_map(|x|(-20..20).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        points.push((0.05,-0.35,0.55));
        let mut model=base.body_envelope().unwrap().clone();
        model.footprint_xy_m=vec![[-0.05,-0.1],[0.55,-0.1],[0.55,0.1],[-0.05,0.1]];
        let map=SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap().with_body_envelope(model).unwrap();
        let start=pose(0.05,0.05,0.0);let goal=pose(-1.05,0.05,std::f64::consts::PI);
        assert!(action(&map,start,Twist2 {linear:Point2::new(0.0,0.0),angular_z:-1.0},std::f64::consts::PI,&[]).is_none());
        let path=connect(&map,start,goal,&[],true,false,false,false).expect("另一原地转向方向有完整净空");
        assert!(planar(path[1]).yaw>0.0,"必须使用未受阻的一侧");
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        assert!(position_distance(*path.last().unwrap(),goal)<1e-6);
    }

    #[test]
    fn unobserved_entire_goal_region_does_not_invent_support() {
        let base=scene(false);
        let points:Vec<_>=(-35..0).flat_map(|x|(-20..20).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        let map=SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
        let options=GoalOptions {position_tolerance_m:0.35,yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        assert!(plan(&map,pose(-0.55,0.05,0.0),pose(0.55,0.05,0.0),options,Some(&limits()),&[],8000)
            .unwrap_err().contains("目标容差区域没有已观测"));
    }

    #[test]
    fn rotation_keeps_planned_direction_after_pose_overshoots_small_samples() {
        // 路径明确选择大于半圈的正向旋转；控制不能跳到平移段后反向走短路。
        let map=Arc::new(scene(false));
        let start=pose(-0.55,0.05,0.0);
        let heading=3.6_f64;
        let goal=pose(-0.55+heading.cos(),0.05+heading.sin(),heading);
        let options=GoalOptions {position_tolerance_m:0.12,
            yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        let mut config=SurfaceFollowerConfig::new(0.4);
        config.controller.speed=0.35;
        config.max_angular_speed_rps=0.8;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(stamped(start,1_000_000_000));
        follower.configure_goal(&stamped(goal,1_000_000_000),options).unwrap();
        let mut route=vec![start];
        route.extend((1..=72).map(|i|pose(-0.55,0.05,f64::from(i)*0.05)));
        route.extend((1..=20).map(|i|pose(-0.55+f64::from(i)*0.05*heading.cos(),
            0.05+f64::from(i)*0.05*heading.sin(),heading)));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses:route.into_iter().map(|p|stamped(p,1_000_000_000)).collect()
        }),None,stamped(goal,1_000_000_000)).unwrap();
        // 位姿采样故意一次跨过数个离散角度，必须始终沿规划方向继续旋转。
        for (index,yaw) in [0.0,0.19,0.41,0.73,1.1,1.6,2.2,2.8,3.15].into_iter().enumerate() {
            follower.update_odometry(stamped(pose(-0.55,0.05,yaw),1_000_000_000+index as i64*50_000_000));
            let PlannerStep::Command(command)=follower.step(0.3) else {panic!("可行旋转不能停走");};
            assert!(command.angular_z>0.0,"不得反向追旧角度或改走短旋转：{command:?}");
            assert!(command.linear.x.abs()<1e-9,"对齐前保持原地旋转");
        }
    }

    #[test]
    fn small_rotation_waypoint_reaches_position_with_angular_deadband() {
        // 步态响应模型独立于控制器：小于0.15弧度/秒的转向没有实际角位移。
        for initial_yaw in [-0.10, 0.10] {
            let map=Arc::new(scene(false));
            let mut current=pose(-0.55,0.05,initial_yaw);
            let goal=pose(0.55,0.05,2.0);
            let options=GoalOptions {position_tolerance_m:0.12,
                yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
            let mut config=SurfaceFollowerConfig::new(0.4);
            config.motion_limits=Some(limits());
            config.controller.speed=0.35;
            config.controller.control_frequency=20.0;
            let mut follower=SurfaceFollower::new(config).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
            follower.update_odometry(stamped(current,1_000_000_000));
            follower.configure_goal(&stamped(goal,1_000_000_000),options).unwrap();
            let mut route=vec![current,pose(-0.55,0.05,0.0)];
            route.extend((1..=22).map(|i|pose(-0.55+f64::from(i)*0.05,0.05,0.0)));
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses:route.into_iter().map(|p|stamped(p,1_000_000_000)).collect()
            }),None,stamped(goal,1_000_000_000)).unwrap();
            let mut reached=false;
            for tick in 1..=500 {
                follower.update_odometry(stamped(current,1_000_000_000+tick*50_000_000));
                match follower.step(0.3) {
                    PlannerStep::Succeeded=>{reached=true;break;},
                    PlannerStep::Command(command)=>{
                        follower.observe_applied_command(command);
                        let mut actual=command;
                        if actual.angular_z.abs()<0.15 {actual.angular_z=0.0;}
                        let next=nuwax_nav_control::integrate(planar(current),actual,0.05);
                        let next_pose=pose(next.x,next.y,next.yaw);
                        map.validate_body_motion(current,next_pose,&[]).unwrap();
                        current=next_pose;
                    },
                    other=>panic!("连续机体控制未能接续：{other:?}"),
                }
            }
            assert!(reached,"微小转向不得永久阻止沿可行路线前进");
        }
    }

    #[test]
    fn complete_body_search_detours_and_preserves_rotation() {
        let map = scene(true);
        let start = pose(-1.55, 0.05, 0.0);
        let goal = pose(1.55, 0.05, 0.0);
        let path = plan(
            &map,
            start,
            goal,
            GoalOptions::default(),
            Some(&limits()),
            &[],
            8000,
        )
        .unwrap();
        assert!(path.iter().any(|p| p.0[1].abs() > 0.3));
        for p in path.windows(2) {
            map.validate_body_motion(p[0], p[1], &[]).unwrap();
        }
        let turn = plan(
            &scene(false),
            pose(0.05, 0.05, 0.0),
            pose(0.05, 0.05, 1.2),
            GoalOptions::default(),
            Some(&limits()),
            &[],
            100,
        )
        .unwrap();
        assert!(turn.len() > 2);
        assert!(turn
            .iter()
            .all(|p| (p.0[0] - 0.05).hypot(p.0[1] - 0.05) < 1e-5));
    }
    #[test]
    fn blind_start_planning_and_following_reach_observed_ground() {
        let base = scene(false);
        let start = pose(0.027, 0.013, 0.0);
        let goal = pose(0.827, 0.013, 0.0);
        let points: Vec<_> = (-40..40).flat_map(|x| (-30..30).map(move |y|
            ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, 0.075)))
            .filter(|&(x, y, _)| (f64::from(x)-start.0[0]).abs() > 0.3
                || (f64::from(y)-start.0[1]).abs() > 0.12).collect();
        let map = Arc::new(SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
            .unwrap().with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        assert!(map.validate_body_pose(start.0, start.1).is_err());
        let view = map.with_current_body_support(start.0, start.1).unwrap();
        let motion = limits();
        let options = GoalOptions { allow_reverse: false, position_tolerance_m: 0.08,
            yaw_tolerance_rad: 0.08 };
        let path = plan(&view, start, goal, options, Some(&motion), &[], 2000).unwrap();
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.motion_limits = Some(motion);
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        let mut current = start;
        let mut nanos = 1_000_000_000;
        follower.update_odometry(stamped(current, nanos));
        follower.configure_goal(&stamped(goal, nanos), options).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
        }), None, stamped(goal, nanos)).unwrap();
        let mut reached = false;
        for _ in 0..1000 {
            let command = match follower.step(0.24) {
                PlannerStep::Command(v) => v,
                PlannerStep::Succeeded => { reached = true; break; }
                other => panic!("盲区起步执行失败: {other:?} at {current:?}"),
            };
            let next = integrate(planar(current), command, 0.05);
            // 反馈来自独立平面运动积分，真实高度保持不变。
            current = ([next.x, next.y, start.0[2]],
                [0.0, 0.0, (next.yaw*0.5).sin(), (next.yaw*0.5).cos()]);
            nanos += 50_000_000;
            follower.update_odometry(stamped(current, nanos));
        }
        assert!(reached, "盲区起步未到达目标: {current:?}");
        assert!((current.0[0]-goal.0[0]).abs() < 0.08);
        assert!(map.validate_body_pose(start.0, start.1).is_err(), "执行不能补写原始盲区");
    }

    #[test]
    fn current_occupied_goal_uses_the_same_view_as_start_and_path() {
        let base = scene(false);
        let start = pose(0.027, 0.013, 0.0);
        let points: Vec<_> = (-40..40).flat_map(|x| (-30..30).map(move |y|
            ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, 0.075)))
            .filter(|&(x, y, _)| (f64::from(x)-start.0[0]).abs() > 0.3
                || (f64::from(y)-start.0[1]).abs() > 0.12).collect();
        let map = SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
            .unwrap().with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
        let view = map.with_current_body_support(start.0, start.1).unwrap();
        assert!(map.validate_body_pose(start.0, start.1).is_err());
        let options = GoalOptions { allow_reverse: false, position_tolerance_m: 0.08,
            yaw_tolerance_rad: 0.08 };
        plan(&view, start, start, options, Some(&limits()), &[], 2000)
            .expect("当前占据区内的目标不能重新用未处理自体的原图拒绝");
    }

    #[test]
    fn position_only_goal_does_not_append_a_default_final_rotation() {
        let map=scene(false);
        let start=([-0.55,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let goal=([0.55,0.05,0.5],[0.0,0.0,1.0,0.0]);
        let options=GoalOptions {yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        let path=plan(&map,start,goal,options,Some(&limits()),&[],2000).unwrap();
        assert!(planar(*path.last().unwrap()).yaw.abs()<0.01);
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.4)).unwrap();
        follower.configure_goal(&stamped(goal,1),options).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::new(map))).unwrap();
        follower.update_odometry(stamped(start,1));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:path.iter().map(|p|stamped(*p,1)).collect()}),None,stamped(goal,1)).unwrap();
        follower.update_odometry(stamped((goal.0,start.1),2));
        assert_eq!(follower.step(0.3),PlannerStep::Succeeded);
    }

    #[test]
    fn observed_ground_band_height_error_does_not_stall_before_final_rotation() {
        let base = scene(false);
        let points: Vec<_> = (-20..20)
            .flat_map(|x| {
                (-15..15).flat_map(move |y| {
                    [0.025, 0.075, 0.125]
                        .into_iter()
                        .map(move |z| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z))
                })
            })
            .collect();
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap(),
        );
        let start = ([-0.55, 0.05, 0.46], [0.0, 0.0, 0.0, 1.0]);
        let goal = ([0.55, 0.05, 0.70], [0.0, 0.0, 0.3_f64.sin(), 0.3_f64.cos()]);
        let options = GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.1,
            yaw_tolerance_rad: 0.08,
        };
        let motion = limits();
        let path = plan(&map, start, goal, options, Some(&motion), &[], 3000).unwrap();
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.motion_limits = Some(motion);
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map))
            .unwrap();
        let mut current = start;
        let mut nanos = 1_000_000_000;
        follower.update_odometry(stamped(current, nanos));
        follower
            .configure_goal(&stamped(goal, nanos), options)
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                }),
                None,
                stamped(goal, nanos),
            )
            .unwrap();
        let mut reached = false;
        for _ in 0..1200 {
            let command = match follower.step(0.24) {
                PlannerStep::Command(v) => v,
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                other => panic!("{other:?} at {current:?}"),
            };
            let next = integrate(planar(current), command, 0.05);
            // 独立平面模型保持真实高度，不让规划高度成为模拟反馈。
            current = (
                [next.x, next.y, start.0[2]],
                [0.0, 0.0, (next.yaw * 0.5).sin(), (next.yaw * 0.5).cos()],
            );
            nanos += 50_000_000;
            follower.update_odometry(stamped(current, nanos));
        }
        assert!(reached, "同一支撑层高度偏差导致停止推进: {current:?}");
        assert!((current.0[0] - goal.0[0]).hypot(current.0[1] - goal.0[1])
            < options.position_tolerance_m);
        assert!(angle(planar(current).yaw - planar(goal).yaw).abs() < 0.08);
    }

    #[test]
    fn same_xy_on_another_observed_floor_never_completes_body_goal() {
        let base = scene(false);
        let points: Vec<_> = (-20..20).flat_map(|x| (-15..15).flat_map(move |y|
            [0.025, 3.025].map(|z| ((x as f32 + 0.5) * 0.1,
                (y as f32 + 0.5) * 0.1, z)))).collect();
        let map = Arc::new(SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1)
            .unwrap().with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        let start = pose(0.05, 0.05, 0.0);
        let mut goal = start;
        goal.0[2] += 3.0;
        map.validate_body_pose(start.0, start.1).unwrap();
        map.validate_body_pose(goal.0, goal.1).unwrap();
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.motion_limits = Some(limits());
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(stamped(start, 1_000_000_000));
        follower.configure_goal(&stamped(goal, 1_000_000_000), GoalOptions {
            allow_reverse: false, position_tolerance_m: 0.1, yaw_tolerance_rad: 0.08,
        }).unwrap();
        // 当前层的短路径不能借水平距离为零而宣称到达另一层。
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![stamped(start, 1_000_000_000), stamped(start, 1_000_000_000)],
        }), None, stamped(goal, 1_000_000_000)).unwrap();
        for index in 1..=4 {
            follower.update_odometry(stamped(start, 1_000_000_000 + index * 50_000_000));
            assert!(!matches!(follower.step(0.24), PlannerStep::Succeeded));
        }
    }

    #[test]
    fn shared_surface_control_reaches_forward_reverse_and_final_yaw() {
        for reverse in [false, true] {
            let map = Arc::new(scene(false));
            let start = pose(-0.55, 0.05, 0.0);
            let goal = pose(
                if reverse { -1.55 } else { 0.55 },
                0.05,
                if reverse { 0.0 } else { 1.0 },
            );
            let options = GoalOptions {
                allow_reverse: reverse,
                position_tolerance_m: 0.08,
                yaw_tolerance_rad: 0.08,
            };
            let limits = limits();
            let path = plan(&map, start, goal, options, Some(&limits), &[], 2000).unwrap();
            if reverse {
                assert!(path.iter().all(|p| planar(*p).yaw.abs() < 0.1));
            }
            let mut config = SurfaceFollowerConfig::new(0.4);
            config.motion_limits = Some(limits.clone());
            config.controller.control_frequency = 20.0;
            let mut follower = SurfaceFollower::new(config).unwrap();
            follower
                .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
                .unwrap();
            let mut current = start;
            let mut nanos = 1_000_000_000;
            follower.update_odometry(stamped(current, nanos));
            follower
                .configure_goal(&stamped(goal, nanos), options)
                .unwrap();
            follower
                .replace_execution_path(
                    ExecutionPath::Surface(Path3D {
                        poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                    }),
                    None,
                    stamped(goal, nanos),
                )
                .unwrap();
            let mut reached = false;
            for _ in 0..1000 {
                let command = match follower.step(0.2) {
                    PlannerStep::Command(v) => v,
                    PlannerStep::Succeeded => {
                        reached = true;
                        break;
                    }
                    other => panic!("{other:?} at {current:?}"),
                };
                let next = integrate(planar(current), command, 0.05);
                let support = map
                    .support_position_from_body(current.0, current.1, 0.4)
                    .unwrap();
                let next = map
                    .body_pose_on_surface([next.x, next.y], next.yaw, support)
                    .unwrap();
                map.validate_body_motion(current, next, &[]).unwrap();
                current = next;
                nanos += 50_000_000;
                follower.update_odometry(stamped(current, nanos));
            }
            assert!(reached, "倒车={reverse},终态={current:?}");
        }
    }
    #[test]
    fn body_terrain_planner_and_control_cross_real_stair_heights_both_directions() {
        let base = scene(false);
        let mut cfg = base.config().clone();
        cfg.min_step_tread_depth_m = 0.3;
        let mut points = Vec::new();
        for x in 0..65 {
            for y in -15..15 {
                let h = if x < 20 {
                    0.0
                } else if x < 38 {
                    ((x - 20) / 6 + 1) as f32 * 0.15
                } else {
                    0.45
                };
                points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, h + 0.0125));
            }
        }
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, cfg, 0.025)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap(),
        );
        for descending in [false, true] {
            let a = ([0.55, 0.05, 0.425], [0.0, 0.0, 0.0, 1.0]);
            let b = ([4.55, 0.05, 0.875], [0.0, 0.0, 0.0, 1.0]);
            let (start, goal) = if descending { (b, a) } else { (a, b) };
            let options = GoalOptions {
                allow_reverse: descending,
                position_tolerance_m: 0.08,
                yaw_tolerance_rad: 0.08,
            };
            let calibrated = limits();
            let mut probe=start;
            for n in 1..=80 {
                let t=n as f64/80.0;let xy=[start.0[0]*(1.0-t)+goal.0[0]*t,0.05];
                let support=map.support_position_from_body(probe.0,probe.1,0.4).unwrap();
                let next=map.body_pose_on_surface(xy,0.0,support).unwrap_or_else(|e|panic!("地形采样 {n} {xy:?} from {probe:?}: {e}"));
                assert!((next.0[0]-xy[0]).hypot(next.0[1]-xy[1])<1e-8,"姿态拟合不得改变机体水平采样位置");
                map.validate_body_motion(probe,next,&[]).unwrap_or_else(|e|panic!("地形扫掠 {n}: {e}"));probe=next;
            }
            eprintln!("踏面逐点验证通过，下阶={descending}");
            let path = plan(&map, start, goal, options, Some(&calibrated), &[], 2000).unwrap();
            eprintln!("路径完成 {} 点，下阶={descending}",path.len());
            let mut config = SurfaceFollowerConfig::new(0.4);
            config.motion_limits = Some(calibrated);
            config.controller.control_frequency = 20.0;
            let mut follower = SurfaceFollower::new(config).unwrap();
            follower
                .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
                .unwrap();
            let mut current = start;
            let mut nanos = 1_000_000_000;
            follower.update_odometry(stamped(current, nanos));
            follower
                .configure_goal(&stamped(goal, nanos), options)
                .unwrap();
            follower
                .replace_execution_path(
                    ExecutionPath::Surface(Path3D {
                        poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                    }),
                    None,
                    stamped(goal, nanos),
                )
                .unwrap();
            let mut reached = false;
            let mut replans = 0;
            for tick in 0..1500 {
                if tick%100==0 {eprintln!("控制 {tick},下阶={descending},位姿={current:?}");}
                let command = match follower.step(0.2) {
                    PlannerStep::Command(v) => v,
                    PlannerStep::Succeeded => {
                        reached = true;
                        break;
                    }
                    PlannerStep::ReplanRequired(reason) => {
                        replans += 1;
                        assert!(replans <= 8, "{reason} at {current:?}");
                        let path =
                            plan(&map, current, goal, options, Some(&limits()), &[], 2000).unwrap();
                        follower
                            .replace_execution_path(
                                ExecutionPath::Surface(Path3D {
                                    poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                                }),
                                None,
                                stamped(goal, nanos),
                            )
                            .unwrap();
                        Twist2::ZERO
                    }
                    other => panic!("{other:?}"),
                };
                let next = integrate(planar(current), command, 0.05);
                let support = map
                    .support_position_from_body(current.0, current.1, 0.4)
                    .unwrap();
                let next = map
                    .body_pose_on_surface([next.x, next.y], next.yaw, support)
                    .unwrap();
                map.validate_body_motion(current, next, &[]).unwrap();
                current = next;
                nanos += 50_000_000;
                follower.update_odometry(stamped(current, nanos));
            }
            assert!(reached, "下阶={descending},终态={current:?}");
        }
    }
}
