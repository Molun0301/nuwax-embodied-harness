//! 按连续机体位姿搜索三维路径，平面运动基元在真实支撑面上逐段核验。
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_types_messages::{GoalOptions, MotionKinematics, MotionLimits, MotionModel, Point2, Twist2};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

type BodyPose = ([f64; 3], [f64; 4]);

#[path = "surface_oriented_reference.rs"]
mod reference;
pub use reference::{plan_reference_cancellable, plan_reference_typed, with_owner_cancellation, ReferencePlanningFailure};

// 仅有墙弯道测试按单次调用启用；固定槽位，不复制地图、不在计时区间输出。
#[cfg(test)]
mod planning_diagnostic {
    use nuwax_nav_mls_planner::{SurfaceError, SurfaceSnapshot};
    use std::{cell::RefCell, time::Instant};
    #[derive(Debug, PartialEq)]
    pub(super) struct Guide {
        map: usize, start: [u64; 3], goal: [u64; 3], boundary: Vec<[u64; 2]>,
        result: Result<Vec<[u64; 3]>, SurfaceError>,
    }
    #[derive(Debug, Default)]
    pub(super) struct Report {
        // raw、初次圆角、居中候选及其圆角、朝向搜索；后者包含在raw内，不能相加。
        pub phases_ns: [u128; 4],
        // 居中入口条件、候选连接、第二次圆角、最终比较；未执行的阶段保持零。
        pub centered_parts_ns: [u128; 4], pub centered_entered: bool, pub centered_candidate_points: Option<usize>,
        pub guide_ns: [u128; 2], pub guide_calls: [usize; 2], pub guides: [Option<Guide>; 2],
        pub goal_setup_ns: u128, pub regional_targets: usize, pub search_entry_ns: u128,
        // 即时/延后两种搜索，各记录调用次数、累计展开、累计生成。
        pub search_work: [[usize; 3]; 2], pub search_budget_exhausted: usize,
        pub omni_search_work: [usize; 2],
    }
    thread_local! { static REPORT: RefCell<Option<Report>> = const { RefCell::new(None) }; }
    pub(super) fn record(f: impl FnOnce(&mut Report)) {
        REPORT.with(|slot| { if let Some(report) = slot.borrow_mut().as_mut() { f(report); } });
    }
    pub(super) struct Capture;
    impl Capture {
        pub fn begin() -> Self {
            REPORT.with(|slot| { assert!(slot.borrow().is_none()); *slot.borrow_mut() = Some(Report::default()); });
            Self
        }
        pub fn finish(self) -> Report { REPORT.with(|slot| slot.borrow_mut().take().expect("单次规划诊断")) }
    }
    impl Drop for Capture { fn drop(&mut self) { REPORT.with(|slot| { slot.borrow_mut().take(); }); } }
    pub(super) struct Timer(usize, Instant);
    pub(super) fn timer(phase: usize) -> Timer { Timer(phase, Instant::now()) }
    impl Drop for Timer {
        fn drop(&mut self) { let elapsed = self.1.elapsed().as_nanos(); record(|report| report.phases_ns[self.0] += elapsed); }
    }
    pub(super) fn guide(slot: usize, started: Instant, map: &SurfaceSnapshot, start: [f64; 3],
        goal: [f64; 3], boundary: &[[f64; 2]], result: &Result<Vec<[f64; 3]>, SurfaceError>) {
        let elapsed = started.elapsed().as_nanos();
        record(|report| {
            report.guide_calls[slot] += 1; report.guide_ns[slot] += elapsed;
            if report.guides[slot].is_none() {
                report.guides[slot] = Some(Guide { map: map as *const SurfaceSnapshot as usize,
                    start: start.map(f64::to_bits), goal: goal.map(f64::to_bits),
                    boundary: boundary.iter().map(|point| point.map(f64::to_bits)).collect(),
                    result: result.as_ref().map(|points| points.iter().map(|point| point.map(f64::to_bits)).collect())
                        .map_err(Clone::clone) });
            }
        });
    }
    impl Report {
        pub fn guide_equality(&self) -> Option<(bool, bool)> {
            let [Some(a), Some(b)] = &self.guides else { return None; };
            Some((a.map == b.map && a.start == b.start && a.goal == b.goal && a.boundary == b.boundary,
                a.result == b.result))
        }
    }
}
// 优先较快取得可执行路线；加权启发式不承诺最短路径，碰撞/支撑检查不变。
const SEARCH_HEURISTIC_WEIGHT: f64 = 10.0;
const MAX_ACTION_SAMPLES: usize = 1024;
fn angle(v: f64) -> f64 {
    (v + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
fn planar(p: BodyPose) -> PlanarPose {
    // 与完整机体几何使用同一归一化顺序，单位输入保持恒等，避免零转向变成末位微旋转。
    let [x,y,z,w]=if p.1.iter().map(|value|value*value).sum::<f64>()==1.0 {p.1} else {
        let scale=p.1.into_iter().map(f64::abs).fold(0.0,f64::max);
        let q=p.1.map(|value|value/scale);
        let norm=q.iter().map(|value|value*value).sum::<f64>().sqrt();
        q.map(|value|value/norm)
    };
    PlanarPose {
        x: p.0[0],
        y: p.0[1],
        yaw: (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (y * y + z * z)),
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

// Every primitive is checked against the immutable source; missing support
// never grants collision permission or creates a second current-body view.
fn validate_rebased_motion(map:&SurfaceSnapshot,from:BodyPose,to:BodyPose,boundary:&[[f64;2]])->bool {
    map.validate_body_motion(from,to,boundary).is_ok()
}
fn rebased_projection(map:&SurfaceSnapshot,from:BodyPose,xy:[f64;2],yaw:f64)->Option<BodyPose> {
    map.body_pose_after_motion(from,xy,yaw).ok()
}

fn action(map: &SurfaceSnapshot, from: BodyPose, velocity: Twist2, duration: f64, boundary: &[[f64; 2]]) -> Option<Vec<BodyPose>> {
    action_samples(map, from, velocity, duration, boundary, true)
}

/// 搜索可先生成真实站姿并排除已被支配的状态；接纳前仍逐段核验完整运动。
fn action_samples(
    map: &SurfaceSnapshot,
    from: BodyPose,
    velocity: Twist2,
    duration: f64,
    boundary: &[[f64; 2]],
    validate_motion: bool,
) -> Option<Vec<BodyPose>> {
    let model = map.body_envelope()?;
    let start = planar(from);
    // 固定空间/角度步长让相近连接复用完全相同的旋转前缀查询。
    // 地形推进按真实源格半格，与控制查询一致，不能只连粗规划格的三维端点。
    // 最后一步仍精确落在终点，每段保留原角度上限。
    let rate = ((velocity.linear.x.hypot(velocity.linear.y)
        + model.radius() * velocity.angular_z.abs())
        / (f64::from(map.source_voxel_size_m()) * 0.5))
        .max(velocity.angular_z.abs() / 0.08);
    let step = if rate > 0.0 { 1.0 / rate } else { duration };
    let n = (duration / step).ceil().max(1.0) as usize;
    if n > MAX_ACTION_SAMPLES {
        return None;
    }
    let mut samples = Vec::with_capacity(n);
    let mut previous = from;
    for i in 1..=n {
        let time=(step*i as f64).min(duration);
        let next = integrate(start, velocity, time);
        // 每段沿同一相对地形模型推进，法向变化连续传递；不等到失效才跳回格顶。
        let mut lifted=rebased_projection(map,previous,[next.x,next.y],next.yaw)?;
        // 原地转向没有升降通道；地形姿态可以核验，但不能借转向抬高机体避障。
        if velocity.linear.x.hypot(velocity.linear.y)<1e-9 {lifted.0[2]=previous.0[2];}
        if validate_motion {
            if !validate_rebased_motion(map,previous,lifted,boundary) {return None;}
        }
        samples.push(lifted);
        previous = lifted;
    }
    Some(samples)
}
/// 延后的扫掠沿同一组实际站姿进行，失败候选不能展开后继或返回路径。
fn action_samples_valid(map: &SurfaceSnapshot, from: BodyPose, samples: &[BodyPose], boundary: &[[f64; 2]]) -> bool {
    let mut previous = from;
    samples.iter().all(|&next| {
        let valid = validate_rebased_motion(map,previous,next,boundary);
        previous = next;
        valid
    })
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
    // 优先面向行进方向；倒退和侧移保留为转向或前进受阻时的可行后备。
    // 指定终态方向仍在平移结束后完整核验，不能因此优先整段倒走。
    for heading in headings {
        // 最终方向沿运动自然生成；终点高度来自实际站姿与相对地形，不能追加原地升降。
        let b=PlanarPose {x:b.x,y:b.y,yaw:if require_orientation {b.yaw}else{heading}};
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
            let end=out.last_mut()?;
            end.0[0]=b.x;end.0[1]=b.y;
            map.validate_body_pose(end.0,end.1).ok()?;
            Some(out)
        })();
        if candidate.is_some() {
            return candidate;
        }
        }
    }
    None
}

/// 给真实地形上生成的基元计软代价；接纳路径仍须通过完整扫掠。
fn motion_soft_multiplier(map:&SurfaceSnapshot,from:BodyPose,samples:&[BodyPose])->f64 {
    if map.config().wall_buffer_m<=0.0 || map.config().wall_buffer_weight<=0.0 {return 1.0;}
    let Some(model)=map.body_envelope() else {return 1.0;};
    let multiplier=|pose:BodyPose|map.locate_support_from_body(pose.0,pose.1,model.body_to_support_z_m)
        .ok().map_or(1.0,|location|map.soft_traversal_multiplier(location.cell));
    let mut previous=from;let mut previous_cost=multiplier(from);
    let mut weighted=0.0;let mut total=0.0;
    for &next in samples {
        let next_cost=multiplier(next);
        let span=(next.0[0]-previous.0[0]).hypot(next.0[1]-previous.0[1])
            +model.radius()*angle(planar(next).yaw-planar(previous).yaw).abs();
        weighted+=span*(previous_cost+next_cost)*0.5;total+=span;
        previous=next;previous_cost=next_cost;
    }
    if total>0.0 {weighted/total}else{previous_cost}
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
        // XY已由半体素整数坐标标识。姿态只归并亚微米/微弧度量级的闭环数值扰动，
        // 节点仍保留原三维站姿并逐边完整核验，真实地形姿态差异继续独立搜索。
        let norm=pose.1.iter().map(|v|v*v).sum::<f64>().sqrt();
        let axis=(0..4).max_by(|&a,&b|pose.1[a].abs().total_cmp(&pose.1[b].abs())).unwrap();
        let sign=if pose.1[axis]<0.0 {-1.0}else{1.0};
        [pose.0[2],pose.1[0]*sign/norm,pose.1[1]*sign/norm,pose.1[2]*sign/norm,pose.1[3]*sign/norm]
            .map(|value|(value*1e6).round() as i64)
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
    let mut queue = BinaryHeap::from([Queue(6.0*heuristic(start), 0)]);
    let mut expanded = 0;
    while let Some(Queue(priority, index)) = queue.pop() {
        if cancelled() || expanded >= 1024 {
            return None;
        }
        let mut here = nodes[index].pose;
        let (x, y, heading) = positions[index];
        let yaw = initial_yaw + f64::from(heading) * std::f64::consts::PI / 16.0;
        if let Some((velocity, duration)) = pending_motions[index].take() {
            let from = nodes[nodes[index].parent?].pose;
            let Some(samples) = action_samples(map, from, velocity, duration, boundary, false) else {
                continue;
            };
            // 必须用推进后的真实高度和姿态判重，不能用平面预估端点合并不同站姿。
            // 软代价乘子不小于一，几何长度与转角成本给出可证明的下界。
            let destination = *samples.last()?;
            let lower_cost = nodes[nodes[index].parent?].cost + position_distance(from, destination)
                + model.radius() * velocity.angular_z.abs() * duration + 0.001;
            if best.get(&(x, y, heading, pose_key(destination)))
                .is_some_and(|&(cost, _)| cost <= lower_cost) { continue; }
            if !action_samples_valid(map, from, &samples, boundary) { continue; }
            // 基元末姿态来自实际站姿的连续地形推进，不能因为另一终点也无碰撞，
            // 就把最后一个采样点强行改回旧高度/倾斜；平面指令不能执行这种原地重置。
            here=*samples.last()?;
            nodes[index].pose=here;
            let preference=motion_soft_multiplier(map,from,&samples);
            nodes[index].cost=nodes[nodes[index].parent?].cost+position_distance(from,here)*preference
                +model.radius()*velocity.angular_z.abs()*duration*preference+0.001;
            stances[index]=yaw_rotated(here.1,-f64::from(heading)*std::f64::consts::PI/16.0);
            nodes[index].samples = samples;
        }
        let key=(x,y,heading,pose_key(here));
        if best.get(&key).is_some_and(|&(cost,known)|known!=index && cost<=nodes[index].cost) {continue;}
        best.insert(key,(nodes[index].cost,index));
        let actual_priority=nodes[index].cost+6.0*heuristic(here);
        if actual_priority>priority+1e-9 {queue.push(Queue(actual_priority,index));continue;}
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
            #[cfg(test)] planning_diagnostic::record(|report| report.omni_search_work = [expanded, nodes.len()]);
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
            let cost = nodes[index].cost + position_distance(here, target) + 0.001;
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

/// Legacy observed-support planner, retained for API compatibility and diagnostics.
/// Active mission estimates use `plan_reference_cancellable` with their PCT owner.
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

/// Legacy observed-support query; cancellation does not use a wall-clock cutoff.
/// Borrows the caller's existing cancellation flag throughout search and smoothing.
#[allow(clippy::too_many_arguments)]
pub fn plan_cancellable(
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

/// 完整路径仅在交付前简化一次；执行阶段仍沿同一条路线做有限前瞻。
#[allow(clippy::too_many_arguments)]
pub fn plan_with_motion_cancellable(
    map: &SurfaceSnapshot, start: BodyPose, goal: BodyPose, options: GoalOptions,
    limits: Option<&MotionKinematics>, boundary: &[[f64; 2]], budget: usize,
    cancelled: Option<&std::sync::atomic::AtomicBool>,
) -> Result<Vec<BodyPose>, String> {
    let stopped=||cancelled.is_some_and(|flag|flag.load(std::sync::atomic::Ordering::Relaxed));
    options.validate()?;
    if let Some(motion)=limits {motion.validate()?;}
    // 自主导航只使用前进和转向，物理执行器声明仍由调用方保留。
    let options=GoalOptions {allow_reverse:false,..options};
    let motion=limits.copied().map(nuwax_nav_runtime::surface_follower::forward_navigation_kinematics);
    let limits=motion.as_ref();
    let raw=plan_complete_body_path(map,start,goal,options,limits,boundary,budget,cancelled)?;
    let goal_allowed=|path:&Vec<BodyPose>|path.last().is_some_and(|&end|
        position_distance(end,goal)<=options.position_tolerance_m
        && (!options.requires_orientation() || angle(planar(end).yaw-planar(goal).yaw).abs()<=options.yaw_tolerance_rad));
    let mut path={
        #[cfg(test)] let _smoothing_timer=planning_diagnostic::timer(1);
        round_forward_corners(map,&raw,limits,boundary,&stopped).filter(goal_allowed).unwrap_or(raw)
    };
    #[cfg(test)] let centered_timer=planning_diagnostic::timer(2);
    // 已有可行路线后再比较居中候选，软偏好不能把唯一窄路变为无路。
    #[cfg(test)] let gate_started=std::time::Instant::now();
    let try_centered=budget>0 && motion_soft_multiplier(map,start,&path)>1.0+1e-6 && !stopped();
    #[cfg(test)] {
        let elapsed=gate_started.elapsed().as_nanos();
        planning_diagnostic::record(|report| {report.centered_parts_ns[0]=elapsed;report.centered_entered=try_centered;});
    }
    if try_centered {
        let hint=(goal.0,if options.requires_orientation(){goal.1}else{path.last().expect("完整路线终点").1});
        #[cfg(test)] let candidate_started=std::time::Instant::now();
        let candidate=centered_forward_candidate(map,start,hint,options,limits,boundary,&stopped);
        #[cfg(test)] {
            let elapsed=candidate_started.elapsed().as_nanos();
            planning_diagnostic::record(|report| {report.centered_parts_ns[1]=elapsed;
                report.centered_candidate_points=candidate.as_ref().map(Vec::len);});
        }
        if let Some(candidate)=candidate {
            #[cfg(test)] let rounding_started=std::time::Instant::now();
            let candidate=round_forward_corners(map,&candidate,limits,boundary,&stopped).unwrap_or(candidate);
            #[cfg(test)] {
                let elapsed=rounding_started.elapsed().as_nanos();
                planning_diagnostic::record(|report| report.centered_parts_ns[2]=elapsed);
            }
            #[cfg(test)] let comparison_started=std::time::Instant::now();
            if candidate.last().is_some_and(|&end|position_distance(end,goal)<=options.position_tolerance_m
                && (!options.requires_orientation() || angle(planar(end).yaw-planar(goal).yaw).abs()<=options.yaw_tolerance_rad))
                && forward_path_cost(map,&candidate,limits)<forward_path_cost(map,&path,limits) {
                path=candidate;
            }
            #[cfg(test)] {
                let elapsed=comparison_started.elapsed().as_nanos();
                planning_diagnostic::record(|report| report.centered_parts_ns[3]=elapsed);
            }
        }
    }
    #[cfg(test)] drop(centered_timer);
    if stopped() {return Err("三维规划已取消".into());}
    Ok(path)
}

/// 以已声明速度折算前进与转向耗时，软墙距只参与偏好比较。
fn forward_path_cost(map:&SurfaceSnapshot,path:&[BodyPose],limits:Option<&MotionKinematics>)->f64 {
    let turn_scale=limits.map_or_else(||map.body_envelope().map_or(1.0,|model|model.radius()),
        |motion|motion.max_forward_mps/motion.max_angular_rps);
    path.windows(2).map(|pair| {
        let distance=(pair[1].0[0]-pair[0].0[0]).hypot(pair[1].0[1]-pair[0].0[1]);
        distance.max(turn_scale*angle(planar(pair[1]).yaw-planar(pair[0]).yaw).abs())
            *motion_soft_multiplier(map,pair[0],&pair[1..])
    }).sum()
}

/// 两段相切圆弧连接起终姿态；两段都按机体正X执行，反向构图不产生倒车。
fn forward_biarc_connection(
    map:&SurfaceSnapshot,from:BodyPose,to:BodyPose,limits:Option<&MotionKinematics>,
    boundary:&[[f64;2]],cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    let a=planar(from);let b=planar(to);let v=[b.x-a.x,b.y-a.y];
    let norm2=v[0]*v[0]+v[1]*v[1];if norm2<1e-12 {return None;}
    let t0=[a.yaw.cos(),a.yaw.sin()];let t1=[b.yaw.cos(),b.yaw.sin()];
    let projection=v[0]*(t0[0]+t1[0])+v[1]*(t0[1]+t1[1]);
    let turn=2.0*(1.0-(t0[0]*t1[0]+t0[1]*t1[1]));
    let tangent=if turn<1e-10 {
        if projection<=1e-10 {return None;}
        norm2/(2.0*projection)
    } else {(-projection+(projection*projection+turn*norm2).sqrt())/turn};
    if !tangent.is_finite() || tangent<0.0 {return None;}
    let joint=[(a.x+b.x+tangent*(t0[0]-t1[0]))*0.5,(a.y+b.y+tangent*(t0[1]-t1[1]))*0.5];
    let arc=|origin:[f64;2],heading:f64,target:[f64;2]|->Option<(f64,f64)> {
        let x=(target[0]-origin[0])*heading.cos()+(target[1]-origin[1])*heading.sin();
        let y=-(target[0]-origin[0])*heading.sin()+(target[1]-origin[1])*heading.cos();
        let d2=x*x+y*y;if d2<1e-14 {return Some((0.0,0.0));}
        if y.abs()<1e-10 {return (x>=0.0).then_some((0.0,x));}
        let delta=2.0*y.atan2(x);
        // 可选平滑连接不绕大圈；无此短圆弧时由原完整搜索负责。
        if delta.abs()>std::f64::consts::PI {return None;}
        let curvature=2.0*y/d2;
        let radius=curvature.abs().recip();
        if radius<limits.map_or(0.0,|m|m.minimum_turning_radius_m) {return None;}
        Some((curvature,delta/curvature))
    };
    let first=arc([a.x,a.y],a.yaw,joint)?;
    let reverse=arc([b.x,b.y],angle(b.yaw+std::f64::consts::PI),joint)?;
    let mut output=vec![from];
    for (curvature,length) in [first,(-reverse.0,reverse.1)] {
        if cancelled() {return None;}
        if length<=1e-10 {continue;}
        output.extend(action(map,*output.last()?,Twist2 {linear:Point2::new(1.0,0.0),angular_z:curvature},length,boundary)?);
    }
    let end=*output.last()?;
    if (end.0[0]-b.x).hypot(end.0[1]-b.y)>1e-7 || angle(planar(end).yaw-b.yaw).abs()>1e-7 {return None;}
    Some(output)
}

/// 前向连续连接与转向直连使用相同实际机体许可，按完整代价选择可执行结果。
fn connect_forward_motion(
    map:&SurfaceSnapshot,from:BodyPose,to:BodyPose,limits:Option<&MotionKinematics>,
    boundary:&[[f64;2]],require_orientation:bool,cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    let direct=connect(map,from,to,boundary,limits.is_none_or(|m|m.can_rotate_in_place),false,false,require_orientation);
    let distance=(to.0[0]-from.0[0]).hypot(to.0[1]-from.0[1]);
    if cancelled() || distance<1e-8 {return direct;}
    if direct.as_ref().is_some_and(|path|path.windows(2)
        .all(|p|angle(planar(p[1]).yaw-planar(p[0]).yaw).abs()<1e-6)) {return direct;}
    let heading=(to.0[1]-from.0[1]).atan2(to.0[0]-from.0[0]);
    let mut best=direct;
    let mut accept=|candidate:Option<Vec<BodyPose>>| {
        if let Some(candidate)=candidate {
            if best.as_ref().is_none_or(|path|forward_path_cost(map,&candidate,limits)<forward_path_cost(map,path,limits)) {
                best=Some(candidate);
            }
        }
    };
    if !require_orientation {
        // 位置目标的末方向自由，单圆弧可自然到达，不能为占位方向再造回绕。
        let yaw=planar(from).yaw;
        let dx=(to.0[0]-from.0[0])*yaw.cos()+(to.0[1]-from.0[1])*yaw.sin();
        let dy=-(to.0[0]-from.0[0])*yaw.sin()+(to.0[1]-from.0[1])*yaw.cos();
        let delta=2.0*dy.atan2(dx);
        if dy.abs()>1e-10 && delta.abs()<=std::f64::consts::PI {
            let curvature=2.0*dy/(dx*dx+dy*dy);
            if curvature.abs().recip()>=limits.map_or(0.0,|m|m.minimum_turning_radius_m) && !cancelled() {
                accept(action(map,from,Twist2{linear:Point2::new(1.0,0.0),angular_z:curvature},delta/curvature,boundary)
                    .map(|samples|{let mut path=vec![from];path.extend(samples);path})
                    .filter(|path|path.last().is_some_and(|p|(p.0[0]-to.0[0]).hypot(p.0[1]-to.0[1])<1e-7)));
            }
        }
    }
    let target=if require_orientation {to}
        else {(to.0,yaw_rotated(from.1,angle(heading-planar(from).yaw)))};
    if !cancelled() {accept(forward_biarc_connection(map,from,target,limits,boundary,cancelled));}
    best
}

/// 临近原到达球时检查连续前向短弧，不能只用粗格落点迫使机体继续顶向障碍。
fn forward_goal_region_connection(
    map:&SurfaceSnapshot,start:BodyPose,goal:BodyPose,options:GoalOptions,
    limits:Option<&MotionKinematics>,boundary:&[[f64;2]],cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    if options.requires_orientation() {return None;}
    let tolerance=options.position_tolerance_m;
    let fine=f64::from(map.source_voxel_size_m());
    let distance=position_distance(start,goal);
    if distance>tolerance+2.0*fine || distance<=tolerance {return None;}
    let pose=planar(start);let delta=[start.0[0]-goal.0[0],start.0[1]-goal.0[1]];
    let step=std::f64::consts::TAU/32.0;
    let mut best=None;let mut best_cost=f64::INFINITY;
    for turn in [0.0,-step,step,-2.0*step,2.0*step,-4.0*step,4.0*step] {
        if cancelled() {return None;}
        let (x,y)=if turn==0.0 {(1.0,0.0)}else{(turn.sin()/turn,2.0*(turn*0.5).sin().powi(2)/turn)};
        let direction=[x*pose.yaw.cos()-y*pose.yaw.sin(),x*pose.yaw.sin()+y*pose.yaw.cos()];
        let a=direction[0]*direction[0]+direction[1]*direction[1];
        let b=2.0*(delta[0]*direction[0]+delta[1]*direction[1]);
        let c=distance*distance-tolerance*tolerance;
        let discriminant=b*b-4.0*a*c;
        if b>=0.0 || discriminant<=0.0 {continue;}
        // 靠近球边界时使用等价稳定根式，避免两个相近数相减丢失毫米内距离。
        let entry=2.0*c/(-b+discriminant.sqrt());let exit=(-b+discriminant.sqrt())/(2.0*a);
        if entry<=0.0 || exit<=entry {continue;}
        // 球交点只生成候选；高度仍沿实际地形推进，最终用原三维球检查。
        for fraction in [0.001,0.01,0.1] {
            let length=entry+((exit-entry)*fraction).min(fine*0.5);
            if turn!=0.0 && length/turn.abs()<limits.map_or(0.0,|m|m.minimum_turning_radius_m) {continue;}
            let Some(samples)=action(map,start,Twist2 {linear:Point2::new(1.0,0.0),angular_z:turn/length},length,boundary) else {continue;};
            if samples.last().is_none_or(|&end|position_distance(end,goal)>tolerance) {continue;}
            let mut path=vec![start];path.extend(samples);
            let cost=forward_path_cost(map,&path,limits);
            if cost<best_cost {best=Some(path);best_cost=cost;}
            break;
        }
    }
    best
}

/// 从支撑图的软代价路线构造前向机体候选，不把支撑点连通当作通行许可。
fn centered_forward_candidate(
    map:&SurfaceSnapshot,start:BodyPose,goal:BodyPose,options:GoalOptions,
    limits:Option<&MotionKinematics>,boundary:&[[f64;2]],cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    let model=map.body_envelope()?;
    let target=prepare_goal_pose(map,goal,options.requires_orientation()).ok()?;
    let a=map.locate_support_from_body(start.0,start.1,model.body_to_support_z_m).ok()?;
    let b=map.locate_goal_support_from_body(target.0,target.1,model.body_to_support_z_m).ok()?;
    let coarse_guide=map.plan_in_boundary(a.support_xyz,b.support_xyz,boundary).ok()?;
    if coarse_guide.len()<2 {return None;}
    // 长直段内也保留转入道路中心的落点，不能只在紧贴拐角的端点急转。
    let step=limits.map_or(model.radius()*2.0,|m|m.max_forward_mps/m.max_angular_rps)
        .max(f64::from(map.config().voxel_size)*2.0);
    let mut guide=vec![coarse_guide[0]];
    for pair in coarse_guide.windows(2) {
        if cancelled() {return None;}
        let length=(pair[1][0]-pair[0][0]).hypot(pair[1][1]-pair[0][1]);
        let count=(length/step).ceil().max(1.0) as usize;
        for i in 1..=count {
            let t=i as f64/count as f64;
            guide.push(std::array::from_fn(|axis|pair[0][axis]+t*(pair[1][axis]-pair[0][axis])));
        }
    }
    fn continue_guide(
        map:&SurfaceSnapshot,guide:&[[f64;3]],index:usize,from:BodyPose,target:BodyPose,
        limits:Option<&MotionKinematics>,boundary:&[[f64;2]],require_orientation:bool,
        cancelled:&impl Fn()->bool,attempts:&mut usize,
    )->Option<Vec<BodyPose>> {
        if index+1>=guide.len() {return Some(vec![from]);}
        let mut candidates=vec![guide.len()-1];
        let mut offset=(guide.len()-1-index)/2;
        while offset>0 {candidates.push(index+offset);offset/=2;}
        let mut best=None;let mut best_cost=f64::INFINITY;
        for next in candidates {
            if cancelled() {return None;}
            if *attempts>=16 {break;}
            *attempts+=1;
            let last=next+1==guide.len();
            let heading=if last {planar(target).yaw} else {
                let before=guide[next-1];let after=guide[next+1];
                (after[1]-before[1]).atan2(after[0]-before[0])
            };
            let to=if last {target} else {
                let support=guide[next];
                let Ok(to)=map.body_pose_on_surface([support[0],support[1]],heading,support) else {continue;};to
            };
            let candidate=connect_forward_motion(map,from,to,limits,boundary,!last || require_orientation,cancelled);
            let Some(mut segment)=candidate else {continue;};
            if let Some(tail)=continue_guide(map,guide,next,*segment.last()?,target,limits,boundary,require_orientation,cancelled,attempts) {
                segment.extend(tail.into_iter().skip(1));
                let cost=forward_path_cost(map,&segment,limits);
                if cost<best_cost {best_cost=cost;best=Some(segment);}
            }
        }
        best
    }
    continue_guide(map,&guide,0,start,target,limits,boundary,options.requires_orientation(),cancelled,&mut 0)
}

/// 在局部真实窄窗口前完成对齐，细候选不替代接近、穿越和离开的完整检查。
fn passage_forward_candidate(
    map:&SurfaceSnapshot,start:BodyPose,goal:BodyPose,options:GoalOptions,
    limits:Option<&MotionKinematics>,boundary:&[[f64;2]],cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    let model=map.body_envelope()?;
    let a=map.locate_support_from_body(start.0,start.1,model.body_to_support_z_m).ok()?;
    let b=map.locate_goal_support_from_body(goal.0,goal.1,model.body_to_support_z_m).ok()?;
    #[cfg(test)] let guide_started=std::time::Instant::now();
    let guide=map.plan_in_boundary(a.support_xyz,b.support_xyz,boundary);
    #[cfg(test)] planning_diagnostic::guide(0,guide_started,map,a.support_xyz,b.support_xyz,boundary,&guide);
    let guide=guide.ok()?;
    let coarse=f64::from(map.config().voxel_size);let fine=f64::from(map.source_voxel_size_m());
    let longitudinal=(coarse*1.5).max(fine*2.0);let lateral=model.radius()*2.0;
    let width=model.footprint_xy_m.iter().map(|p|p[1]).fold(f64::NEG_INFINITY,f64::max)
        -model.footprint_xy_m.iter().map(|p|p[1]).fold(f64::INFINITY,f64::min);
    let mut queries=vec![(std::array::from_fn(|i|(start.0[i]+goal.0[i])*0.5),
        (goal.0[1]-start.0[1]).atan2(goal.0[0]-start.0[0]))];
    for pair in guide.windows(2) {
        let length=(pair[1][0]-pair[0][0]).hypot(pair[1][1]-pair[0][1]);
        let heading=(pair[1][1]-pair[0][1]).atan2(pair[1][0]-pair[0][0]);
        let count=(length/(longitudinal*2.0)).ceil().max(1.0) as usize;
        for index in 0..count {
            if queries.len()>=128 {break;}
            let t=(index as f64+0.5)/count as f64;
            let mut center=std::array::from_fn(|axis|pair[0][axis]+t*(pair[1][axis]-pair[0][axis]));
            center[2]+=model.body_to_support_z_m;queries.push((center,heading));
        }
    }
    let mut seen=HashSet::new();let mut attempts=0;
    for (center,heading) in queries {
        if cancelled() {return None;}
        let Ok(candidates)=map.body_passage_candidates(center,heading,[longitudinal,lateral]) else {continue;};
        for candidate in candidates {
            // 宽走廊交给一般居中引导；这里只补粗中心可能错过的连续窄窗口。
            if candidate.clear_width_m>width+2.0*coarse {continue;}
            let key=(candidate.entry_xyz.map(|x|(x/fine).round() as i64),
                candidate.exit_xyz.map(|x|(x/fine).round() as i64));
            if !seen.insert(key) {continue;}
            if attempts>=16 {return None;}attempts+=1;
            let orientation=yaw_rotated(start.1,angle(candidate.heading-planar(start).yaw));
            let Ok(entry)=prepare_goal_pose(map,(candidate.entry_xyz,orientation),true) else {continue;};
            let Ok(exit)=prepare_goal_pose(map,(candidate.exit_xyz,orientation),true) else {continue;};
            let Some(mut route)=connect_forward_motion(map,start,entry,limits,boundary,true,cancelled) else {continue;};
            let Some(through)=connect_forward_motion(map,*route.last()?,exit,limits,boundary,true,cancelled) else {continue;};
            route.extend(through.into_iter().skip(1));
            let Some(tail)=connect_forward_motion(map,*route.last()?,goal,limits,boundary,options.requires_orientation(),cancelled) else {continue;};
            route.extend(tail.into_iter().skip(1));
            return Some(route);
        }
    }
    None
}

/// 按原相邻姿态恢复前向运动，用新实际接触姿态重新推进后段。
fn replay_forward_step(map:&SurfaceSnapshot,from:BodyPose,original:[BodyPose;2],boundary:&[[f64;2]])
    ->Option<Vec<BodyPose>> {
    let a=planar(original[0]);let b=planar(original[1]);
    let turn=angle(b.yaw-a.yaw);
    let dx=(b.x-a.x)*a.yaw.cos()+(b.y-a.y)*a.yaw.sin();
    let dy=-(b.x-a.x)*a.yaw.sin()+(b.y-a.y)*a.yaw.cos();
    let (s,c)=if turn.abs()<1e-9 {(1.0,turn*0.5)} else {(turn.sin()/turn,2.0*(turn*0.5).sin().powi(2)/turn)};
    let divisor=s*s+c*c;
    let forward=(s*dx+c*dy)/divisor;
    let lateral=(-c*dx+s*dy)/divisor;
    if forward< -1e-8 || lateral.abs()>1e-8 {return None;}
    action(map,from,Twist2{linear:Point2::new(forward.max(0.0),0.0),angular_z:turn},1.0,boundary)
}

/// 只在中途原地转向两侧取连续移动窗口，用端姿态相切的前向圆弧消除折角。
fn round_forward_corners(
    map:&SurfaceSnapshot,path:&[BodyPose],limits:Option<&MotionKinematics>,boundary:&[[f64;2]],
    cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    if path.len()<4 {return None;}
    let model=map.body_envelope()?;
    let distance=|a:BodyPose,b:BodyPose|(a.0[0]-b.0[0]).hypot(a.0[1]-b.0[1]);
    let rotation=|a:BodyPose,b:BodyPose|distance(a,b)<1e-8 && angle(planar(b).yaw-planar(a).yaw).abs()>1e-6;
    let preferred_radius=limits.map_or(model.radius(),|m|m.max_forward_mps/m.max_angular_rps)
        .max(limits.map_or(0.0,|m|m.minimum_turning_radius_m));
    let mut checkpoints=vec![];let mut output=vec![path[0]];let mut cursor=0;let mut scan=1;let mut changed=false;let mut attempts=0;
    while scan+1<path.len() {
        if cancelled() {return None;}
        if !rotation(path[scan-1],path[scan]) {scan+=1;continue;}
        let corner=scan-1;let mut end=scan;
        while end+1<path.len() && distance(path[corner],path[end+1])<1e-8 {end+=1;}
        scan=end+1;
        if corner<=cursor || end+1>=path.len() {continue;}
        let turn=angle(planar(path[end]).yaw-planar(path[corner]).yaw);
        if turn.abs()<1e-4 {continue;}
        let desired=(preferred_radius*(turn.abs()*0.5).tan().abs()).min(preferred_radius*2.0);
        // 曲线也可提供相切端姿态；不能因相邻段不是直线而跳过整处折角。
        let mut before=corner;let mut before_length=0.0;
        while before>cursor && !rotation(path[before-1],path[before]) && before_length<desired*2.3 {
            before_length+=distance(path[before-1],path[before]);before-=1;
        }
        let mut after=end;let mut after_length=0.0;
        while after+1<path.len() && !rotation(path[after],path[after+1]) && after_length<desired*2.3 {
            after_length+=distance(path[after],path[after+1]);after+=1;
        }
        let available=(before_length*0.45).min(after_length*0.45).min(desired);
        let mut accepted=None;
        for scale in [1.0,0.5,0.25] {
            if attempts>=32 {break;}
            if cancelled() {return None;}
            let trim=available*scale;
            if trim<f64::from(map.source_voxel_size_m())*0.5 {continue;}
            attempts+=1;
            let mut left=corner;let mut span=0.0;
            while left>before && span<trim {span+=distance(path[left-1],path[left]);left-=1;}
            let mut right=end;span=0.0;
            while right<after && span<trim {span+=distance(path[right],path[right+1]);right+=1;}
            let attempt=(|| {
                let mut points=vec![*output.last()?];
                for index in cursor+1..=left {
                    points.extend(replay_forward_step(map,*points.last()?,[path[index-1],path[index]],boundary)?);
                }
                let joined=forward_biarc_connection(map,*points.last()?,path[right],limits,boundary,cancelled)?;
                points.extend(joined.into_iter().skip(1));
                if forward_path_cost(map,&points,limits)>forward_path_cost(map,&path[cursor..=right],limits)+1e-8 {return None;}
                Some((points,right))
            })();
            if attempt.is_some() {accepted=attempt;break;}
        }
        if let Some((points,right))=accepted {checkpoints.push((output.len(),cursor));output.extend(points.into_iter().skip(1));cursor=right;changed=true;}
    }
    if !changed {return None;}
    loop {
        let mut result=output.clone();let mut valid=true;
        for index in cursor+1..path.len() {
            if cancelled() {return None;}
            if let Some(next)=replay_forward_step(map,*result.last()?,[path[index-1],path[index]],boundary) {result.extend(next);}
            else {valid=false;break;}
        }
        if valid {return Some(result);}
        // 某处圆角改变实际接触后，后段可能不再可行；逐处撤回，保留仍可完整接续的前段。
        let (length,previous)=checkpoints.pop()?;output.truncate(length);cursor=previous;
        if checkpoints.is_empty() {return None;}
    }
}

/// 删除格网绕行中的多余折角，每条捷径及其后续都从真实末站姿连续重算。
fn simplify_omnidirectional_path(
    map:&SurfaceSnapshot,path:&[BodyPose],boundary:&[[f64;2]],reverse:bool,
    cancelled:&impl Fn()->bool,
)->Option<Vec<BodyPose>> {
    if path.len()<3 {return None;}
    let distance=|a:BodyPose,b:BodyPose|(b.0[0]-a.0[0]).hypot(b.0[1]-a.0[1]);
    let heading=|a:BodyPose,b:BodyPose|(b.0[1]-a.0[1]).atan2(b.0[0]-a.0[0]);
    // 逐段简化固定偏航折线；局部曲率不能让整条路线的其它直段放弃简化。
    let mut original_cost=vec![0.0;path.len()];
    for index in 1..path.len() {
        original_cost[index]=original_cost[index-1]+distance(path[index-1],path[index])
            *motion_soft_multiplier(map,path[index-1],&path[index..=index]);
    }
    let mut output=vec![path[0]];
    let mut index=0;
    let mut attempts=0;
    let mut changed=false;
    while index+1<path.len() {
        if cancelled() {return None;}
        let from=*output.last()?;
        let turn=angle(planar(path[index+1]).yaw-planar(path[index]).yaw);
        if turn.abs()>1e-6 {
            if distance(path[index],path[index+1])>1e-8 {
                // 曲率段保留真实采样位置与偏航；此前改过路线时只重算连续地形站姿。
                let target=path[index+1];
                let next=if changed {rebased_projection(map,from,[target.0[0],target.0[1]],planar(target).yaw)?}
                    else {target};
                if !validate_rebased_motion(map,from,next,boundary) {return None;}
                output.push(next);
            } else {
                output.extend(action(map,from,Twist2 {linear:Point2::new(0.0,0.0),angular_z:turn.signum()},
                    turn.abs(),boundary)?);
            }
            index+=1;
            continue;
        }
        let mut end=index+1;
        while end+1<path.len() && angle(planar(path[end+1]).yaw-planar(path[index]).yaw).abs()<1e-6 {end+=1;}
        let mut corners=vec![index];
        for point in index+1..end {
            if distance(path[point-1],path[point])>1e-8 && distance(path[point],path[point+1])>1e-8
                && angle(heading(path[point-1],path[point])-heading(path[point],path[point+1])).abs()>1e-6 {
                corners.push(point);
            }
        }
        corners.push(end);
        if corners.len()==2 && !changed {
            output.extend_from_slice(&path[index+1..=end]);
            index=end;
            continue;
        }
        let mut cursor=0;
        while cursor+1<corners.len() {
            if cancelled() {return None;}
            let from=*output.last()?;
            let yaw=planar(from).yaw;
            let mut accepted=None;
            // 简化只优化已有完整路线；固定查询预算耗尽后按原折角接续。
            for candidate in (cursor+2..corners.len()).rev() {
                if attempts>=64 {break;}
                if cancelled() {return None;}
                attempts+=1;
                let to=path[corners[candidate]];
                let dx=to.0[0]-from.0[0];let dy=to.0[1]-from.0[1];
                let velocity=Twist2 {linear:Point2::new(dx*yaw.cos()+dy*yaw.sin(),-dx*yaw.sin()+dy*yaw.cos()),angular_z:0.0};
                if !reverse && velocity.linear.x< -1e-8 {continue;}
                if let Some(samples)=action(map,from,velocity,1.0,boundary) {
                    let cost=dx.hypot(dy)*motion_soft_multiplier(map,from,&samples);
                    if cost<=original_cost[corners[candidate]]-original_cost[corners[cursor]]+1e-9 {
                        accepted=Some((candidate,samples));
                        changed=true;
                        break;
                    }
                }
            }
            let (next,samples)=accepted.or_else(|| {
                // 原直段保留既有地形采样位置，避免重新取样累积另一条站姿轨迹。
                let mut pose=from;let mut samples=Vec::new();
                for target in &path[corners[cursor]+1..=corners[cursor+1]] {
                    let next=rebased_projection(map,pose,[target.0[0],target.0[1]],planar(pose).yaw)?;
                    if !validate_rebased_motion(map,pose,next,boundary) {return None;}
                    samples.push(next);pose=next;
                }
                Some((cursor+1,samples))
            })?;
            output.extend(samples);cursor=next;
        }
        index=end;
    }
    changed.then_some(output)
}

/// 规划只依赖执行器运动通道，保证制动参数仍由控制器单独消费。
#[allow(clippy::too_many_arguments)]
fn plan_complete_body_path(
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
    #[cfg(test)] let _raw_timer=planning_diagnostic::timer(0);
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
    let reference=requested;
    if let Ok(centre)=prepared.as_ref() {
        goal=*centre;
        if let Some(path)=connect_forward_motion(map,start,goal,limits,boundary,options.requires_orientation(),&cancelled)
            .filter(|path|path.last().is_some_and(|end|position_distance(*end,reference)<=original_tolerance)) {return Ok(path);}
    }
    if let Some(path)=forward_goal_region_connection(map,start,reference,options,limits,boundary,&cancelled) {return Ok(path);}
    // 地形投影只产生候选机体位姿，不能平移用户原始三维到达球。
    let mut regional_targets: Vec<_> = prepared.as_ref().copied().into_iter()
        .filter(|target|position_distance(*target,reference)<=original_tolerance).collect();
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
        // 快速查询直接尝试容差落点；完整规划在搜索失败后再逐点直连，
        // 避免围绕同一远处目标先重复检查数十条几乎重合的阻塞长线。
        if budget==0 {if let Some(path)=connect_forward_motion(map,start,target,limits,boundary,options.requires_orientation(),&cancelled) {
            if path.last().is_some_and(|end|position_distance(*end,reference)<=original_tolerance) {return Ok(path);}
        }}
    }
    if regional_targets.is_empty() {
        return Err(format!("{}；目标容差区域没有已观测的可通行机体落点",
            prepared.err().unwrap_or_else(||"目标投影不在原三维到达范围内".into())));
    }
    if budget == 0 {
        return Err("当前观测中没有至最终目标的完整快速连接".into());
    }
    #[cfg(test)] {
        let elapsed=diagnostic_started.elapsed().as_nanos();
        planning_diagnostic::record(|report| {report.goal_setup_ns=elapsed;report.regional_targets=regional_targets.len();});
    }
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
    if let Some(path)=passage_forward_candidate(map,start,goal,options,limits,boundary,&cancelled)
        .filter(|path|path.last().is_some_and(|pose|endpoint_allowed(*pose))) {return Ok(path);}
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
            if let Some(tail)=connect_forward_motion(map,from,goal,limits,boundary,options.requires_orientation(),&cancelled)
                .filter(|path|path.last().is_some_and(|pose|endpoint_allowed(*pose))) {
                let mut path=vec![start];path.extend(prefix);path.extend(tail.into_iter().skip(1));return Ok(path);
            }
        }
    }
    // 后方目标先尝试连续圆弧驶离起步足迹，避免把可行的大转弯拆成
    // 数百次小角度状态扩展。圆弧仍沿同一投影和完整扫掠验证。
    let turn_radius = limits.map_or(0.0, |m| m.minimum_turning_radius_m)
        .max(model.radius()).max(0.2);
    let mut curved_candidate:Option<Vec<BodyPose>>=None;
    let mut curved_cost=f64::INFINITY;
    for radius in [turn_radius, turn_radius * 2.0] {
        for direction in [-1.0, 1.0] {
            let mut prefix=vec![start];
            for _ in 1..=4 {
                if cancelled() {return Err("三维规划已取消".into());}
                // 递增同一圆弧，已核验的前段不从起点重复生成和扫掠。
                let Some(next) = action(map, *prefix.last().expect("圆弧起点"), Twist2 {
                    linear: Point2::new(1.0, 0.0), angular_z: direction / radius,
                }, radius*std::f64::consts::FRAC_PI_4, boundary) else {break;};
                prefix.extend(next);
                let from = *prefix.last().expect("圆弧基元");
                let remaining=(from.0[0]-goal.0[0]).hypot(from.0[1]-goal.0[1]);
                if forward_path_cost(map,&prefix,limits)+(remaining-original_tolerance).max(0.0)>=curved_cost {break;}
                let tail = if endpoint_allowed(from) && (!options.requires_orientation()
                    || angle(planar(from).yaw-planar(goal).yaw).abs() <= options.yaw_tolerance_rad) {
                    Some(vec![from])
                } else {
                    connect_forward_motion(map,from,goal,limits,boundary,options.requires_orientation(),&cancelled)
                };
                if let Some(tail) = tail.filter(|path|path.last().is_some_and(|pose|endpoint_allowed(*pose))) {
                    let mut path=prefix.clone();path.extend(tail.into_iter().skip(1));
                    let cost=forward_path_cost(map,&path,limits);
                    if cost<curved_cost {curved_cost=cost;curved_candidate=Some(path);}
                }
            }
        }
    }
    if let Some(path)=curved_candidate {return Ok(path);}
    // 先利用已维护的支撑图生成引导线，再逐段核对真实机体及转向。
    // 开阔地和普通绕行无需展开完整的32方向状态空间；引导线不通过
    // 机体扫掠时仍进入下方完整搜索，不把点可达当作机体可达。
    if let (Ok(a), Ok(b)) = (
        map.locate_support_from_body(start.0, start.1, model.body_to_support_z_m),
        map.locate_goal_support_from_body(goal.0, goal.1, model.body_to_support_z_m),
    ) {
        #[cfg(test)] let guide_started=std::time::Instant::now();
        let guide=map.plan_in_boundary(a.support_xyz, b.support_xyz, boundary);
        #[cfg(test)] planning_diagnostic::guide(1,guide_started,map,a.support_xyz,b.support_xyz,boundary,&guide);
        if let Ok(guide) = guide {
            let mut path = vec![start];
            let mut valid = true;
            for (index, support) in guide.iter().enumerate().skip(1) {
                if cancelled() { return Err("三维规划已取消".into()); }
                let last = *path.last().expect("起点");
                let final_segment = index + 1 == guide.len();
                let yaw = if final_segment && options.requires_orientation() { planar(goal).yaw }
                    else { (support[1]-last.0[1]).atan2(support[0]-last.0[0]) };
                let Ok(target) = map.body_pose_on_surface([support[0],support[1]],yaw,*support) else { valid=false;break; };
                if let Some(segment) = connect_forward_motion(map,last,target,limits,boundary,
                    final_segment && options.requires_orientation(),&cancelled) {
                    path.extend(segment.into_iter().skip(1));
                } else { valid=false;break; }
            }
            if valid && path.len()>1 && path.last().is_some_and(|pose|endpoint_allowed(*pose)) { return Ok(path); }
        }
    }
    // 按规划格和朝向搜索完整前向路线，所有后继保留真实连续站姿。
    // 位置目标先按平面候选排序，出队后生成真实站姿并按真实代价重新排队，
    // 再检查完整扫掠；只有已核验状态参与支配判重，候选不计实际展开预算。
    // 需要最终朝向时保留原展开方式，位置目标也保留原方式作为回退。
    let search = |deferred:bool| -> Result<Vec<BodyPose>,String> {
    let resolution = f64::from(map.config().voxel_size);
    #[cfg(test)] let _search_timer=planning_diagnostic::timer(3);
    #[cfg(test)] {
        let elapsed=diagnostic_started.elapsed().as_nanos();
        planning_diagnostic::record(|report| {report.search_entry_ns=elapsed;report.search_work[usize::from(deferred)][0]+=1;});
    }
    let yaw_step = std::f64::consts::TAU / 32.0;
    let radius = limits
        .map_or(0.0, |m| m.minimum_turning_radius_m)
        .max(model.radius())
        .max(0.2);
    let length = resolution * 2.0;
    let action_length=|velocity:Twist2,duration:f64| {
        if let Some(motion)=limits {
            let forward=motion.max_forward_mps;
            let longitudinal=if velocity.linear.x>=0.0 {velocity.linear.x}
                else {-velocity.linear.x*forward/motion.max_reverse_mps};
            let lateral=if velocity.linear.y==0.0 {0.0}
                else {velocity.linear.y.abs()*forward/motion.max_lateral_mps};
            let angular=velocity.angular_z.abs()*forward/motion.max_angular_rps;
            // 同时满足平移模长、各通道与角速度；折算为米，与支撑距离场同单位。
            duration*velocity.linear.x.hypot(velocity.linear.y).max(longitudinal).max(lateral).max(angular)
        } else {
            duration*(velocity.linear.x.hypot(velocity.linear.y)+model.radius()*velocity.angular_z.abs())
        }
    };
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
        let turn=if options.requires_orientation() {angle(planar(goal).yaw-planar(p).yaw).abs()}
            else if omni {0.0} else if reverse {forward.min(angle(heading+std::f64::consts::PI-planar(p).yaw).abs())} else {forward};
        // 所有目标落点共享请求的末方向，距离场覆盖同一到达域；明确朝向也需要绕墙引导。
        let graph_distance=map.locate_support_from_body(p.0,p.1,model.body_to_support_z_m)
            .ok().and_then(|location|goal_costs.cost(location.cell)).unwrap_or(d);
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
    let mut pending_actions:HashMap<usize,(Twist2,f64)>=HashMap::new();
    while let Some(Queue(_, index)) = queue.pop() {
        if cancelled() { return Err("三维规划已取消".into()); }
        if let Some((velocity,duration))=pending_actions.remove(&index) {
            let parent=nodes[index].parent.expect("候选有父状态");
            let from=nodes[parent].pose;
            let Some(samples)=action_samples(map,from,velocity,duration,boundary,false) else {continue;};
            let Some(&end)=samples.last() else {continue;};
            let Some(state)=key(end) else {continue;};
            if Some(state)==key(from) {continue;}
            let preference=motion_soft_multiplier(map,from,&samples);
            let cost=nodes[parent].cost+action_length(velocity,duration)*preference+0.001;
            if best.get(&state).is_some_and(|&(old,_)|old<=cost) {continue;}
            nodes[index].pose=end;nodes[index].cost=cost;nodes[index].samples=samples;
            queue.push(Queue(cost+estimate(end)*SEARCH_HEURISTIC_WEIGHT,index));
            continue;
        }
        let here = nodes[index].pose;
        let cost = nodes[index].cost;
        // 固定偏航的全向折线在搜索内尝试祖先直连，新站姿继续生成后继。
        if deferred && omni {
            if let Some(parent)=nodes[index].parent {
                if let Some(ancestor)=nodes[parent].parent {
                    let from=nodes[ancestor].pose;
                    let heading=planar(from).yaw;
                    let fixed=nodes[parent].samples.iter().chain(&nodes[index].samples)
                        .all(|&pose|angle(planar(pose).yaw-heading).abs()<1e-6);
                    if fixed {
                        let dx=here.0[0]-from.0[0];let dy=here.0[1]-from.0[1];
                        let velocity=Twist2 {linear:Point2::new(dx*heading.cos()+dy*heading.sin(),
                            -dx*heading.sin()+dy*heading.cos()),angular_z:0.0};
                        let lower=nodes[ancestor].cost+action_length(velocity,1.0)+0.001;
                        let corner_length=(nodes[parent].pose.0[0]-from.0[0]).hypot(nodes[parent].pose.0[1]-from.0[1])
                            +(here.0[0]-nodes[parent].pose.0[0]).hypot(here.0[1]-nodes[parent].pose.0[1]);
                        if dx.hypot(dy)<corner_length-1e-6
                            && (reverse || velocity.linear.x>=-1e-8) && lower<cost-1e-8 {
                            if let Some(samples)=action(map,from,velocity,1.0,boundary) {
                                let replacement=nodes[ancestor].cost+action_length(velocity,1.0)
                                    *motion_soft_multiplier(map,from,&samples)+0.001;
                                if replacement<cost-1e-8 {
                                    let end=*samples.last().expect("祖先直连终点");
                                    nodes[index].parent=Some(ancestor);nodes[index].pose=end;
                                    nodes[index].cost=replacement;nodes[index].samples=samples;
                                    queue.push(Queue(replacement+estimate(end)*SEARCH_HEURISTIC_WEIGHT,index));
                                    continue;
                                }
                            }
                        }
                    }
                }
            }
        }
        let state=key(here).ok_or("搜索状态支撑缺失")?;
        if best.get(&state).is_some_and(|&(old,known)|known!=index && old<=cost) {continue;}
        if deferred {if let Some(parent)=nodes[index].parent {
            if !action_samples_valid(map,nodes[parent].pose,&nodes[index].samples,boundary) {continue;}
        }}
        best.insert(state,(cost,index));
        expanded += 1;
        if expanded > budget || (!deferred && nodes.len()>budget*4) {
            #[cfg(test)] planning_diagnostic::record(|report| {
                report.search_work[usize::from(deferred)][1]+=expanded;
                report.search_work[usize::from(deferred)][2]+=nodes.len();report.search_budget_exhausted+=1;
            });
            return Err("三维朝向搜索预算耗尽".into());
        }
        let close = endpoint_allowed(here)
            && (!options.requires_orientation() || angle(planar(here).yaw - planar(goal).yaw).abs()
                <= options.yaw_tolerance_rad.min(yaw_step).max(1e-6));
        let tail = if close {
            Some(vec![here])
        } else if distance(here) < length * 3.0 || expanded % 8 == 0 {
            connect_forward_motion(map,here,goal,limits,boundary,options.requires_orientation(),&cancelled)
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
            #[cfg(test)] planning_diagnostic::record(|report| {
                report.search_work[usize::from(deferred)][1]+=expanded;
                report.search_work[usize::from(deferred)][2]+=nodes.len();
            });
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
        // 紧弯保留可达性，巡航半径减少宽路中反复小紧弯导致的降速和摆头。
        let cruising_radius=limits.map_or(radius,|motion|motion.max_forward_mps/motion.max_angular_rps).max(radius);
        for (index,candidate_radius) in [radius,cruising_radius].into_iter().enumerate() {
            if index==1 && cruising_radius<=radius+1e-8 {continue;}
            let turns:&[f64]=if candidate_radius==radius {&[std::f64::consts::FRAC_PI_4,std::f64::consts::FRAC_PI_2]}
                else {&[yaw_step,std::f64::consts::FRAC_PI_4,std::f64::consts::FRAC_PI_2]};
            for &turn in turns {for sign in [-1.0,1.0] {
                actions.push((Twist2 {linear:Point2::new(1.0,0.0),angular_z:sign/candidate_radius},candidate_radius*turn));
            }}
        }
        if rotate {
            // 大转向与微调共享完整扫掠，减少为了对准走廊逐格展开的小角度状态。
            for steps in [1.0,4.0,8.0] {for sign in [-1.0, 1.0] {
                actions.push((Twist2 {linear:Point2::new(0.0,0.0),angular_z:sign},yaw_step*steps));
            }}
        }
        if omni {
            for forward in [1.0,-1.0] {
                if forward<0.0 && !reverse {continue;}
                for lateral in [-1.0,1.0] {
                    actions.push((Twist2 {linear:Point2::new(
                        forward*std::f64::consts::FRAC_1_SQRT_2,lateral*std::f64::consts::FRAC_1_SQRT_2),
                        angular_z:0.0},length));
                }
            }
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
            if deferred {
                // 这里只排序尚未拟合的动作；真实支撑、代价和扫掠在出队后完成。
                let flat=integrate(planar(here),velocity,duration);
                let nominal=([flat.x,flat.y,here.0[2]],yaw_rotated(here.1,angle(flat.yaw-planar(here).yaw)));
                let cost=cost+action_length(velocity,duration)+0.001;
                let id=nodes.len();
                nodes.push(Node {pose:nominal,cost,parent:Some(index),samples:vec![]});
                pending_actions.insert(id,(velocity,duration));
                queue.push(Queue(cost+estimate(nominal)*SEARCH_HEURISTIC_WEIGHT,id));
                continue;
            }
            let Some(samples) = action_samples(map, here, velocity, duration, boundary, false) else {
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
            let preference=motion_soft_multiplier(map,here,&samples);
            let next = cost+action_length(velocity,duration)*preference+0.001;
            if best.get(&k).is_some_and(|&(old, _)| old <= next) {
                continue;
            }
            if !deferred && !action_samples_valid(map,here,&samples,boundary) {continue;}
            let id = nodes.len();
            nodes.push(Node {
                pose: end,
                cost: next,
                parent: Some(index),
                samples,
            });
            if !deferred {best.insert(k,(next,id));}
            queue.push(Queue(next + estimate(end) * SEARCH_HEURISTIC_WEIGHT, id));
        }
    }
    #[cfg(test)] planning_diagnostic::record(|report| {
        report.search_work[usize::from(deferred)][1]+=expanded;
        report.search_work[usize::from(deferred)][2]+=nodes.len();
    });
    Err("不存在满足完整机体与运动约束的三维路线".into())
    };
    let mut complete=search(!options.requires_orientation());
    if complete.is_err() && !cancelled() && !options.requires_orientation() {complete=search(false);}
    if complete.is_ok() || cancelled() {return complete;}
    if omni && !options.requires_orientation() {
        if let Some(path)=omnidirectional_connection(map,start,reference,original_tolerance,boundary,reverse,rotate,&goal_costs,&cancelled) {
            return Ok(path);
        }
    }
    for target in regional_targets {
        if cancelled() {return Err("三维规划已取消".into());}
        if let Some(path)=connect_forward_motion(map,start,target,limits,boundary,options.requires_orientation(),&cancelled) {
            if path.last().is_some_and(|end|position_distance(*end,reference)<=original_tolerance) {return Ok(path);}
        }
    }
    complete
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
                fixed_collision: None,
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
    fn body_search_heading_does_not_depend_on_quaternion_scale() {
        let initial=pose(0.0,0.0,1.2);
        for scale in [0.01,3.0,1e200] {
            let scaled=(initial.0,initial.1.map(|value|value*scale));
            assert!((planar(scaled).yaw-1.2).abs()<1e-12,
                "相同物理姿态的四元数缩放不能改变搜索方向");
        }
    }

    #[test]
    fn rear_omnidirectional_goal_turns_before_forward_translation() {
        let map=scene(false);
        let mut motion=limits().kinematics();
        motion.model=MotionModel::Omnidirectional;motion.max_lateral_mps=0.5;
        let start=pose(-1.0,0.0,0.0);
        let goal=pose(-1.5,0.0,2.0);
        let options=GoalOptions {allow_reverse:true,position_tolerance_m:0.1,
            yaw_tolerance_rad:std::f64::consts::PI};
        let path=plan_with_motion_cancellable(&map,start,goal,options,Some(&motion),&[],100,None).unwrap();
        let first_translation=path.windows(2).find(|p|position_distance(p[0],p[1])>1e-6).unwrap();
        assert!(angle(planar(first_translation[0]).yaw-std::f64::consts::PI).abs()<1e-6,
            "后方目标应先完成转身再前进");
        for pair in path.windows(2) {
            let yaw=planar(pair[0]).yaw;
            let forward=(pair[1].0[0]-pair[0].0[0])*yaw.cos()+(pair[1].0[1]-pair[0].0[1])*yaw.sin();
            assert!(forward>=-1e-8,"开阔后方目标不能直接倒走");
            map.validate_body_motion(pair[0],pair[1],&[]).unwrap();
        }
        assert!(position_distance(*path.last().unwrap(),goal)<options.position_tolerance_m);
    }

    #[test]
    fn omnidirectional_shortcuts_remove_grid_corners_without_cutting_obstacles_or_final_turns() {
        let map=scene(true);
        let start=pose(-1.2,0.0,0.0);
        let mut path=vec![start];
        for xy in [[-1.2,0.3],[-0.8,0.3],[-0.8,0.7],[-0.4,0.7],[-0.4,1.0],
            [1.0,1.0],[1.0,0.7],[1.4,0.7],[1.4,0.0]] {
            let from=*path.last().unwrap();
            path.extend(action(&map,from,Twist2 {linear:Point2::new(xy[0]-from.0[0],xy[1]-from.0[1]),angular_z:0.0},1.0,&[]).unwrap());
        }
        let before_turn=*path.last().unwrap();
        path.extend(action(&map,before_turn,Twist2 {linear:Point2::new(0.0,0.0),angular_z:1.0},0.8,&[]).unwrap());
        assert!(action(&map,start,Twist2 {linear:Point2::new(2.6,0.0),angular_z:0.0},1.0,&[]).is_none(),
            "直线到达会穿过中间的真实机体障碍");
        let shortened=simplify_omnidirectional_path(&map,&path,&[],true,&||false).unwrap();
        let length=|points:&[BodyPose]|points.windows(2).map(|p|position_distance(p[0],p[1])).sum::<f64>();
        assert!(length(&shortened)<length(&path)-0.3,"一次简化应消除多余格网折线");
        assert!(position_distance(*shortened.last().unwrap(),*path.last().unwrap())<1e-6);
        assert!((planar(*shortened.last().unwrap()).yaw-0.8).abs()<1e-6,"末端真实转向仍保留");
        for pair in shortened.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        assert!(simplify_omnidirectional_path(&map,&path,&[],true,&||true).is_none(),"取消应中断可选简化");
    }

    #[test]
    fn curved_prefix_does_not_disable_straight_span_corner_simplification() {
        let map=scene(false);
        let start=pose(-1.2,-0.6,0.0);
        let mut path=vec![start];
        path.extend(action(&map,start,Twist2{linear:Point2::new(1.0,0.0),angular_z:0.5},0.3,&[]).unwrap());
        let curve_end=path.len();
        for delta in [[0.2,0.0],[0.0,0.2],[0.2,0.0],[0.0,0.2],[0.2,0.0]] {
            let from=*path.last().unwrap();
            path.extend(action(&map,from,Twist2{linear:Point2::new(delta[0],delta[1]),angular_z:0.0},1.0,&[]).unwrap());
        }
        let shortened=simplify_omnidirectional_path(&map,&path,&[],true,&||false).unwrap();
        let length=|points:&[BodyPose]|points.windows(2).map(|p|position_distance(p[0],p[1])).sum::<f64>();
        assert_eq!(&shortened[..curve_end],&path[..curve_end],"曲率前缀保持原采样");
        assert!(length(&shortened)<length(&path)-0.2,"同路线直段的格网锯齿应可删除");
        assert!(position_distance(*shortened.last().unwrap(),*path.last().unwrap())<1e-8);
        for pair in shortened.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    }

    fn assert_forward_path(path:&[BodyPose]) {
        for pair in path.windows(2) {
            let dx=pair[1].0[0]-pair[0].0[0];let dy=pair[1].0[1]-pair[0].0[1];
            if dx.hypot(dy)<1e-9 {continue;}
            // 连续圆弧的位移弦沿中间朝向；不能把合法转弯误计成侧移。
            let heading=planar(pair[0]).yaw+angle(planar(pair[1]).yaw-planar(pair[0]).yaw)*0.5;
            assert!(dx*heading.cos()+dy*heading.sin()>=-1e-8,"路径不能要求倒走");
            assert!((-dx*heading.sin()+dy*heading.cos()).abs()<1e-7,"路径不能要求横移");
        }
    }

    fn wall_bounded_turn_scene() -> SurfaceSnapshot {
        let mut config = crate::surface_config::SurfaceNavigationConfig::new(
            "map".into(), 0.4, 0.32, 0.3, 0.05, 0.2, 20.0, 0.2,
        ).unwrap();
        config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
            self_collision: None,
            footprint_xy_m: vec![[-0.30, -0.16], [0.30, -0.16], [0.30, 0.16], [-0.30, 0.16]],
            z_min_m: -0.05, z_max_m: 0.20, source: "静态有墙直角弯验收".into(),
        }), None).unwrap();
        // 使用统一完整机体配置；墙距是有限软偏好，不另加圆盘硬膨胀。
        assert_eq!(config.geometry.wall_clearance_m, 0.0);
        assert!(config.geometry.wall_buffer_m.is_finite() && config.geometry.wall_buffer_m > 0.0);
        assert!(config.geometry.wall_buffer_weight.is_finite() && config.geometry.wall_buffer_weight > 0.0);
        let mut free = HashSet::new();
        // 体素边界定义精确净空：横臂[-2.10,0]×[-.25,.25]、
        // 竖臂[-.25,.25]×[0,2.10]、居中转向口袋[-.40,.40]²。
        for x in -42..0 { for y in -5..5 { free.insert((x, y)); } }
        for x in -5..5 { for y in 0..42 { free.insert((x, y)); } }
        for x in -8..8 { for y in -8..8 { free.insert((x, y)); } }
        assert_eq!(free.len(), 936, "固定净空体素并集不得扩大或缩小");
        let mut walls = HashSet::new();
        let mut points = Vec::new();
        for x in -43..=8 { for y in -9..=42 {
            let xy = ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05);
            if free.contains(&(x, y)) {
                points.push((xy.0, xy.1, 0.025));
            } else if (-1..=1).any(|dx| (-1..=1).any(|dy| free.contains(&(x + dx, y + dy)))) {
                walls.insert((x, y));
                // 真实占据墙柱从地面封到.80米；包括两臂端盖和对角封口。
                for z in 0..16 { points.push((xy.0, xy.1, (z as f32 + 0.5) * 0.05)); }
            }
        } }
        for cap in [(-43, 0), (0, 42), (-9, -9), (8, 8)] {
            assert!(walls.contains(&cap), "墙柱端盖或对角封口缺失: {cap:?}");
        }
        assert_eq!(points.len(), free.len() + walls.len() * 16);
        SurfaceSnapshot::from_points(&points, config.geometry, 0.05).unwrap()
            .with_body_envelope(config.body_envelope.unwrap()).unwrap()
    }

    #[test]
    fn calibrated_body_plans_and_closes_wall_bounded_ninety_degree_turn() {
        // 静态几何、合成50毫秒执行反馈；不覆盖观察TTL、Scan、后台worker、
        // MotionGuard或硬件验收，Succeeded也不代表物理停止证据。
        const DT: f64 = 0.05;
        const WITNESS_LENGTH_M: f64 = 3.20;
        const MAX_PLANNED_LENGTH_M: f64 = 4.80;
        const MAX_ACTUAL_TRAVEL_M: f64 = 5.0;
        let map = Arc::new(wall_bounded_turn_scene());
        let envelope = map.body_envelope().unwrap().clone();
        assert_eq!(envelope.footprint_xy_m,
            vec![[-0.30, -0.16], [0.30, -0.16], [0.30, 0.16], [-0.30, 0.16]]);
        assert_eq!(envelope.z_bounds_m, [-0.05, 0.20]);
        assert_eq!(envelope.body_to_support_z_m, 0.4);
        assert!((envelope.radius() - 0.34).abs() < 1e-12);
        assert!((0.40 - envelope.radius() - 0.06).abs() < 1e-12);
        assert!((0.25_f64 - 0.16 - 0.09).abs() < 1e-12);
        let actual_pose = |x: f64, y: f64, yaw: f64| -> BodyPose {
            ([x, y, 0.45], [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()])
        };
        let start = actual_pose(-1.60, 0.0, 0.0);
        let goal = actual_pose(0.0, 1.60, std::f64::consts::FRAC_PI_2);
        // 独立通行见证仅证明原场景可过，绝不交给规划器或跟随器作为路线。
        let witness = [start, actual_pose(0.0, 0.0, 0.0),
            actual_pose(0.0, 0.0, std::f64::consts::FRAC_PI_2), goal];
        for edge in witness.windows(2) {
            map.validate_body_motion(edge[0], edge[1], &[])
                .expect("原地图必须容许直行、口袋居中旋转90度、再直行");
        }
        let witness_length: f64 = witness.windows(2).map(|p| position_distance(p[0], p[1])).sum();
        assert!((witness_length - WITNESS_LENGTH_M).abs() < 1e-12);
        for p in [start, goal] {
            let support = map.locate_support_from_body(p.0, p.1, 0.4).unwrap();
            assert!((support.support_xyz[2] - 0.05).abs() < 1e-6);
            map.validate_body_pose(p.0, p.1).unwrap();
        }
        assert!(map.validate_body_motion(start, goal, &[]).is_err(),
            "同图起终点对角扫掠必须撞墙，不能被开阔地面替代");
        assert!(map.validate_body_motion(start,
            actual_pose(-1.60, 0.0, std::f64::consts::FRAC_PI_2), &[]).is_err(),
            "窄臂内90度旋转必须受墙阻挡，不能提前完成转弯");
        let motion = MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 0.20, max_reverse_mps: 0.0, max_lateral_mps: 0.0,
            max_angular_rps: 0.55, linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.05,
            minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
        };
        let options = GoalOptions {
            allow_reverse: false, position_tolerance_m: 0.12, yaw_tolerance_rad: 0.15,
        };
        let planning_capture=planning_diagnostic::Capture::begin();
        let planning_started = std::time::Instant::now();
        let route = plan(&map, start, goal, options, Some(&motion), &[], 8000)
            .unwrap_or_else(|error| panic!("已证明可通行的有墙直角弯规划失败: {error}; plan_wall={:?}",
                planning_started.elapsed()));
        let plan_wall = planning_started.elapsed();
        let planning_report=planning_capture.finish();
        let route_bits:Vec<[u64;7]>=route.iter().map(|&(p,q)|
            [p[0].to_bits(),p[1].to_bits(),p[2].to_bits(),q[0].to_bits(),q[1].to_bits(),q[2].to_bits(),q[3].to_bits()]).collect();
        assert!(route.len() >= 2);
        assert_eq!(route[0], start);
        assert_forward_path(&route);
        for edge in route.windows(2) {
            map.validate_body_motion(edge[0], edge[1], &[]).unwrap();
        }
        let planned_length_m: f64 = route.windows(2).map(|p| position_distance(p[0], p[1])).sum();
        assert!(planned_length_m <= MAX_PLANNED_LENGTH_M,
            "规划路线不得以绕圈换成功: {planned_length_m}米，独立见证{WITNESS_LENGTH_M}米");
        let end = *route.last().unwrap();
        assert!(position_distance(end, goal) < options.position_tolerance_m);
        assert!(angle(planar(end).yaw - planar(goal).yaw).abs() < options.yaw_tolerance_rad);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.speed = 0.20;
        config.controller.control_frequency = 1.0 / DT;
        config.controller.goal_tolerance = options.position_tolerance_m;
        config.controller.orientation_tolerance = options.yaw_tolerance_rad;
        config.controller.simulation = true;
        config.motion_limits = Some(motion.clone());
        assert_eq!(config.motion_limits.as_ref(), Some(&motion));
        assert_eq!(config.kinematics(), Some(motion.kinematics()));
        assert_eq!(config.body_to_support_z_m, envelope.body_to_support_z_m);
        assert!(config.max_observation_age_sec.is_none());
        // 跨crate测试只核验公开入参和实际命令；不声称读取跟随器私有字段。
        let mut follower = SurfaceFollower::new(config).unwrap();
        let mut current = start;
        let mut now = 1_000_000_000;
        follower.configure_goal(&stamped(goal, now), options).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(stamped(current, now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: route.into_iter().map(|p| stamped(p, now)).collect(),
        }), None, stamped(goal, now)).unwrap();
        let mut applied = Twist2::ZERO;
        follower.observe_applied_command(applied);
        let mut reached = false;
        let mut failure = None;
        let mut executed_steps = 0;
        let mut actual_travel_m = 0.0;
        let mut total_abs_yaw_rad = 0.0;
        let mut peak_control_wall = std::time::Duration::ZERO;
        let mut total_control_wall = std::time::Duration::ZERO;
        for tick in 0..1200 {
            follower.update_odometry(stamped(current, now));
            assert_eq!(map.body_envelope(), Some(&envelope));
            let control_started = std::time::Instant::now();
            let result = follower.step(0.32);
            let control_wall = control_started.elapsed();
            peak_control_wall = peak_control_wall.max(control_wall);
            total_control_wall += control_wall;
            let command = match result {
                PlannerStep::Succeeded => { reached = true; break; }
                PlannerStep::Command(command) => command,
                other => {
                    failure = Some(format!("tick={tick} pose={current:?} result={other:?}"));
                    break; // Pending、重规划和失败均失败，禁止重试或替换路线。
                }
            };
            // 原始请求先验约束，再模拟执行；绝不夹紧非法命令来制造成功。
            assert!([command.linear.x, command.linear.y, command.angular_z].into_iter().all(f64::is_finite));
            assert!(command.linear.x >= 0.0 && command.linear.x <= motion.max_forward_mps + 1e-12,
                "tick={tick} 前进命令越界: {command:?}");
            assert_eq!(command.linear.y, 0.0, "差速机体不能横移");
            assert!(command.angular_z.abs() <= motion.max_angular_rps + 1e-12);
            let linear_rate = if command.linear.x < applied.linear.x {
                motion.guaranteed_braking_mps2.min(motion.linear_acceleration_mps2)
            } else { motion.linear_acceleration_mps2 };
            let angular_rate = if command.angular_z.abs() < applied.angular_z.abs() {
                motion.guaranteed_angular_braking_rps2
            } else { motion.angular_acceleration_rps2 };
            assert!((command.linear.x - applied.linear.x).abs() <= linear_rate * DT + 1e-12,
                "tick={tick} 线加减速不可达: previous={applied:?}, requested={command:?}");
            assert!((command.angular_z - applied.angular_z).abs() <= angular_rate * DT + 1e-12,
                "tick={tick} 角加减速不可达: previous={applied:?}, requested={command:?}");
            let speed = (applied.linear.x + command.linear.x) * 0.5;
            let angular = (applied.angular_z + command.angular_z) * 0.5;
            assert!(speed.is_finite() && speed >= 0.0 && speed <= motion.max_forward_mps + 1e-12);
            assert!(angular.is_finite() && angular.abs() <= motion.max_angular_rps + 1e-12);
            let yaw = planar(current).yaw;
            let next_yaw = yaw + angular * DT;
            // 独立SE(2)平均实际速度圆弧积分；高度恒为地面.05+标定.40。
            // 不调用生产integrate/地形投影、不吸附路径，也不改写原地图。
            let (dx, dy) = if angular.abs() < 1e-12 {
                (speed * yaw.cos() * DT, speed * yaw.sin() * DT)
            } else {
                (speed * (next_yaw.sin() - yaw.sin()) / angular,
                 speed * (yaw.cos() - next_yaw.cos()) / angular)
            };
            let next = actual_pose(current.0[0] + dx, current.0[1] + dy, next_yaw);
            map.validate_body_motion(current, next, &[]).unwrap_or_else(|error|
                panic!("原地图拒绝实际完整机体区间 tick={tick}: from={current:?}, to={next:?}, error={error:?}"));
            actual_travel_m += speed * DT;
            total_abs_yaw_rad += angular.abs() * DT;
            assert!(actual_travel_m <= MAX_ACTUAL_TRAVEL_M, "实际行程超过预定5米上限");
            assert!(total_abs_yaw_rad <= std::f64::consts::PI, "累计绝对转角超过预定π上限");
            current = next;
            applied = command;
            follower.observe_applied_command(applied);
            executed_steps += 1;
            now += 50_000_000;
        }
        let error_m = position_distance(current, goal);
        let yaw_error_rad = angle(planar(current).yaw - planar(goal).yaw).abs();
        eprintln!("WALL_TURN_PLAN_DIAGNOSTIC phase_order=raw,smoothing,centered,search centered_subphase_order=gate,candidate,rounding,comparison nested_not_additive=true guide_order=passage,fallback search_order=immediate,deferred search_columns=calls,expanded,generated guide_equal_query_result={:?} report={planning_report:?} route_pose_bits={route_bits:?}",
            planning_report.guide_equality());
        eprintln!("静态有墙90度弯合成验收: arrived={reached}, planned_m={planned_length_m:.6}, witness_m={WITNESS_LENGTH_M:.2}, actual_m={actual_travel_m:.6}, abs_yaw_rad={total_abs_yaw_rad:.6}, error_m={error_m:.6}, yaw_error_rad={yaw_error_rad:.6}, steps={executed_steps}, plan_wall_ms={:.3}, total_control_wall_ms={:.3}, peak_control_wall_ms={:.3}, failure={failure:?}",
            plan_wall.as_secs_f64() * 1000.0, total_control_wall.as_secs_f64() * 1000.0,
            peak_control_wall.as_secs_f64() * 1000.0);
        assert!(failure.is_none(), "有墙直角弯不许等待、失败或重规划: {failure:?}");
        assert!(reached, "必须在1200拍内返回实际Succeeded");
        assert!(error_m < 0.12 && yaw_error_rad < 0.15, "必须到达原始三维目标及方向");
        assert!(executed_steps > 0 && executed_steps <= 1200);
        assert_eq!(map.body_envelope(), Some(&envelope));
    }

    #[test]
    fn forward_only_navigation_turns_to_lateral_and_rear_goals() {
        let map=scene(false);let start=pose(0.0,0.0,0.0);
        let mut motion=limits().kinematics();motion.model=MotionModel::Omnidirectional;motion.max_lateral_mps=0.5;
        for goal in [pose(0.0,1.0,0.0),pose(-1.0,0.0,0.0),pose(-0.8,0.7,0.0)] {
            let path=plan_with_motion_cancellable(&map,start,goal,GoalOptions {allow_reverse:true,
                position_tolerance_m:0.1,yaw_tolerance_rad:std::f64::consts::PI},Some(&motion),&[],100,None).unwrap();
            assert_forward_path(&path);
            assert!(position_distance(*path.last().unwrap(),goal)<0.1);
            for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        }
    }

    #[test]
    fn close_position_goal_uses_a_real_short_forward_arc_inside_the_original_sphere() {
        let map=scene(false);let start=pose(0.0,0.0,0.0);let goal=pose(0.0,0.351,0.0);
        let motion=limits().kinematics();let options=GoalOptions {position_tolerance_m:0.35,
            yaw_tolerance_rad:std::f64::consts::PI,allow_reverse:false};
        let path=forward_goal_region_connection(&map,start,goal,options,Some(&motion),&[],&||false).unwrap();
        assert_forward_path(&path);assert!(path.len()>1 && position_distance(start,*path.last().unwrap())>0.001);
        assert!(position_distance(*path.last().unwrap(),goal)<=0.35);
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        let car=nuwax_nav_types_messages::MotionKinematics{minimum_turning_radius_m:10.0,..motion};
        assert!(forward_goal_region_connection(&map,start,goal,options,Some(&car),&[],&||false).is_none());
        let boundary=[[-0.0001,-0.0001],[0.0001,-0.0001],[0.0001,0.0001],[-0.0001,0.0001]];
        assert!(forward_goal_region_connection(&map,start,goal,options,Some(&motion),&boundary,&||false).is_none());
        let oriented=GoalOptions{yaw_tolerance_rad:0.1,..options};
        assert!(forward_goal_region_connection(&map,start,goal,oriented,Some(&motion),&[],&||false).is_none());
    }

    #[test]
    fn forward_fillet_connects_curves_on_both_sides_of_a_middle_rotation() {
        let map=scene(false);let mut route=vec![pose(-1.1,-0.8,0.0)];
        for (velocity,duration) in [
            (Twist2{linear:Point2::new(1.0,0.0),angular_z:0.3},0.8),
            (Twist2{linear:Point2::new(0.0,0.0),angular_z:1.0},0.8),
            (Twist2{linear:Point2::new(1.0,0.0),angular_z:-0.2},0.8),
        ] {route.extend(action(&map,*route.last().unwrap(),velocity,duration,&[]).unwrap());}
        let motion=limits().kinematics();
        let smooth=round_forward_corners(&map,&route,Some(&motion),&[],&||false).expect("相邻曲线应可相切接续");
        assert_forward_path(&smooth);
        assert_eq!(smooth[0],route[0]);
        assert!(position_distance(*smooth.last().unwrap(),*route.last().unwrap())<1e-7);
        assert!(angle(planar(*smooth.last().unwrap()).yaw-planar(*route.last().unwrap()).yaw).abs()<1e-7);
        assert!(smooth.windows(2).all(|p|position_distance(p[0],p[1])>1e-8
            || angle(planar(p[1]).yaw-planar(p[0]).yaw).abs()<1e-6));
        assert!(forward_path_cost(&map,&smooth,Some(&motion))<forward_path_cost(&map,&route,Some(&motion)));
        for pair in smooth.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    }

    #[test]
    fn forward_fillet_replaces_middle_right_angles_but_keeps_endpoint_turns() {
        let map=scene(false);let start=pose(-1.2,-0.7,std::f64::consts::PI);
        let mut route=vec![start];
        for (i,to) in [pose(-0.2,-0.7,0.0),pose(-0.2,0.7,std::f64::consts::FRAC_PI_2),pose(1.2,0.7,0.4)]
            .into_iter().enumerate() {
            route.extend(connect(&map,*route.last().unwrap(),to,&[],true,false,false,i==2).unwrap().into_iter().skip(1));
        }
        let motion=limits().kinematics();
        let smooth=round_forward_corners(&map,&route,Some(&motion),&[],&||false).expect("开阔直角应可圆滑");
        assert_forward_path(&smooth);
        assert_eq!(smooth[0],route[0]);
        assert!(position_distance(*smooth.last().unwrap(),*route.last().unwrap())<1e-7);
        assert!(angle(planar(*smooth.last().unwrap()).yaw-0.4).abs()<1e-7);
        let middle_turns=|path:&[BodyPose]|path.windows(2).filter(|pair| {
            position_distance(pair[0],pair[1])<1e-8
                && angle(planar(pair[1]).yaw-planar(pair[0]).yaw).abs()>1e-5
                && position_distance(pair[0],start)>1e-6
                && position_distance(pair[0],*route.last().unwrap())>1e-6
        }).count();
        assert!(middle_turns(&route)>0);assert_eq!(middle_turns(&smooth),0,"中间直角应成为相切圆弧");
        assert!(forward_path_cost(&map,&smooth,Some(&motion))<forward_path_cost(&map,&route,Some(&motion)));
        for pair in smooth.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        assert!(round_forward_corners(&map,&route,Some(&motion),&[],&||true).is_none());
    }

    #[test]
    fn free_goal_heading_uses_a_natural_forward_arc_and_keeps_the_actual_start() {
        let map=scene(false);let start=pose(-0.8,-0.8,0.0);let goal=pose(0.2,0.2,0.0);
        let motion=limits().kinematics();
        let path=connect_forward_motion(&map,start,goal,Some(&motion),&[],false,&||false).unwrap();
        assert_eq!(path[0],start);assert_forward_path(&path);
        assert!(position_distance(*path.last().unwrap(),goal)<1e-7);
        assert!(path.windows(2).all(|p|position_distance(p[0],p[1])>1e-8),"自然圆弧无需先原地折转");
        assert!((planar(*path.last().unwrap()).yaw-std::f64::consts::FRAC_PI_2).abs()<1e-7,
            "位置目标不应被占位零方向牵引回绕");
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    }

    #[test]
    fn forward_biarc_enters_corridor_center_without_lateral_motion_or_stationary_turns() {
        let map=scene(false);let start=pose(-1.2,-0.5,0.0);let end=pose(0.5,0.0,0.0);
        let path=forward_biarc_connection(&map,start,end,Some(&limits().kinematics()),&[],&||false).unwrap();
        assert_forward_path(&path);
        assert!(position_distance(*path.last().unwrap(),end)<1e-7);
        assert!(path.windows(2).all(|p|position_distance(p[0],p[1])>1e-8),"向中心转入不需要原地折角");
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    }

    #[test]
    fn forward_center_preference_compares_a_valid_wall_hugging_direct_route() {
        let base=scene(false);let mut config=base.config().clone();
        config.wall_clearance_m=0.0;config.wall_buffer_m=0.6;config.wall_buffer_weight=3.0;
        let mut points=Vec::new();
        for ix in -22..23 {for iy in -6..=5 {
            let zs=if iy==-6 || iy==5 {0..10}else{0..1};
            for iz in zs {points.push(((ix as f32+0.5)*0.1,(iy as f32+0.5)*0.1,(iz as f32+0.5)*0.1));}
        }}
        let map=SurfaceSnapshot::from_points(&points,config,0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
        let start=pose(-1.5,-0.3,0.0);let goal=pose(1.5,-0.3,0.0);
        let direct=connect(&map,start,goal,&[],true,false,false,false).expect("靠墙直达本来可行");
        let motion=limits().kinematics();
        let options=GoalOptions {allow_reverse:true,position_tolerance_m:0.1,yaw_tolerance_rad:std::f64::consts::PI};
        let a=map.locate_support_from_body(start.0,start.1,0.4).unwrap();
        let b=map.locate_goal_support_from_body(goal.0,goal.1,0.4).unwrap();
        eprintln!("居中支撑引导: {:?}",map.plan_in_boundary(a.support_xyz,b.support_xyz,&[]));
        eprintln!("直接软代价: {}，居中候选: {:?}",forward_path_cost(&map,&direct,Some(&motion)),
            centered_forward_candidate(&map,start,goal,options,Some(&motion),&[],&||false)
                .map(|p|(p.len(),forward_path_cost(&map,&p,Some(&motion)))));
        let path=plan_with_motion_cancellable(&map,start,goal,options,Some(&motion),&[],2000,None).unwrap();
        assert_forward_path(&path);
        let middle:Vec<_>=path.iter().filter(|p|p.0[0].abs()<0.5).collect();
        assert!(!middle.is_empty());
        assert!(middle.iter().map(|p|p.0[1].abs()).sum::<f64>()/(middle.len() as f64)<0.2,
            "存在有效直连时也应比较居中路线，不能始终贴墙：{:?}",middle.iter().map(|p|p.0).collect::<Vec<_>>());
        assert!(forward_path_cost(&map,&path,Some(&motion))<forward_path_cost(&map,&direct,Some(&motion)));
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    }

    fn soft_preference_scene(y_min:i32,y_max:i32,obstacle:bool,weight:f32)->SurfaceSnapshot {
        let mut config=crate::surface_config::SurfaceNavigationConfig::new(
            "map".into(),0.4,0.32,0.3,0.1,0.2,20.0,0.2).unwrap();
        config.geometry.wall_buffer_weight=weight;
        let geometry=nuwax_nav_types_messages::RobotGeometry {
            self_collision: None,
            footprint_xy_m:vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
            z_min_m:-0.05,z_max_m:0.2,source:"软偏好回归".into(),
        };
        config.apply_calibration(Some(geometry.clone()),None).unwrap();
        let buffer=config.geometry.wall_buffer_m;
        config.apply_calibration(Some(geometry),None).unwrap();
        assert!((config.geometry.wall_buffer_m-buffer).abs()<1e-6,"重复装配不能累加软余量");
        assert_eq!(config.geometry.wall_clearance_m,0.0);
        let mut points=vec![];
        for x in -30..45 {for y in y_min..y_max {
            if obstacle && (-2..14).contains(&x) && (-4..4).contains(&y) {
                for z in 0..12 {points.push(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,(z as f32+0.5)*0.1));}
            } else {points.push(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025));}
        }}
        SurfaceSnapshot::from_points(&points,config.geometry,0.1).unwrap()
            .with_body_envelope(config.body_envelope.unwrap()).unwrap()
    }

    #[test]
    fn body_soft_preference_keeps_a_narrow_corridor_traversable() {
        let map=soft_preference_scene(-2,2,false,2.0);
        let start=pose(-1.05,0.0,0.0);let goal=pose(2.05,0.0,0.0);
        let support=map.locate_support_from_body(start.0,start.1,0.4).unwrap();
        assert!(map.soft_traversal_multiplier(support.cell)>1.0,"窄路应有有限软代价");
        let path=plan(&map,start,goal,GoalOptions{position_tolerance_m:0.12,..Default::default()},Some(&limits()),&[],2000).unwrap();
        for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        assert!(position_distance(*path.last().unwrap(),goal)<0.12);
    }

    #[test]
    fn body_soft_preference_uses_the_wider_side_of_an_obstacle() {
        let mut clearances=vec![];let mut sides=vec![];
        for weight in [0.0,2.0] {
            // 两侧均可前向转入，较窄侧也可执行，才能检验软偏好的路线选择。
            let map=soft_preference_scene(-11,14,true,weight);
            let start=pose(-1.25,0.0,0.0);let goal=pose(2.45,0.0,0.0);
            let mut motion=limits();motion.model=MotionModel::Omnidirectional;motion.max_lateral_mps=0.5;
            let began=std::time::Instant::now();
            let path=plan(&map,start,goal,GoalOptions{position_tolerance_m:0.12,
                yaw_tolerance_rad:std::f64::consts::PI,..Default::default()},Some(&motion),&[],5000).unwrap();
            for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            sides.push(path.iter().filter(|p|p.0[0]>0.2 && p.0[0]<1.0).map(|p|p.0[1]).sum::<f64>());
            let clearance=path.iter().filter(|p|p.0[0]>0.2 && p.0[0]<1.0).map(|p| {
                let yaw=planar(*p).yaw;let half=0.3*yaw.sin().abs()+0.16*yaw.cos().abs();
                let obstacle_gap=p.0[1].abs()-0.4-half;
                let outer_gap=if p.0[1]>0.0 {1.4-p.0[1]-half}else{p.0[1]+1.1-half};
                obstacle_gap.min(outer_gap)
            }).fold(f64::INFINITY,f64::min);
            assert!(clearance.is_finite() && clearance>0.0);
            clearances.push(clearance);
            eprintln!("软偏好权重{weight}，核心段净空{clearance}米，规划与完整复核{:?}",began.elapsed());
        }
        assert!(sides[0]<0.0 && sides[1]>0.0,"两侧都可前向通行，软偏好应切换到较宽侧：{sides:?}");
        assert!(clearances[1]>clearances[0],"较宽侧的完整机体净空也必须增加：{clearances:?}");
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
    fn physical_reverse_capability_does_not_enable_forbidden_navigation() {
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
        // 平地上不应凭空生成法向残差，再把恰在Y=0的支撑点跳到另一行。
        let mut probe=current;
        for i in 1..=20 {
            let xy=[0.5-0.05*i as f64,0.0];
            let next=map.body_pose_after_motion(probe,xy,0.0).unwrap();
            map.validate_body_motion(probe,next,&[])
                .unwrap_or_else(|error|panic!("平地倒退样本{i}不应丢失连续支撑：{error}, {probe:?}->{next:?}"));
            probe=next;
        }
        assert!(plan_with_motion_cancellable(&map,current,goal,options,Some(&motion),&[],0,None).is_err(),
            "没有转身空间时，物理倒车能力不能变成自主导航的倒退后备");
        let mut config=SurfaceFollowerConfig::new(0.4);config.basic_kinematics=Some(motion);
        let policy=config.kinematics().unwrap();
        assert_eq!(policy.max_reverse_mps,0.0);assert_eq!(policy.max_lateral_mps,0.0);

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
        let ground_before = original.query_ground([turn.0[0], turn.0[1]], 0.2).height.unwrap();
        let ground_after = corrected.query_ground([turn.0[0], turn.0[1]], 0.2).height.unwrap();
        assert!(ground_after < ground_before,
            "本例必须包含真实地图高度校正；同层接触关联允许保留原站姿");
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
        for calibrated in [false,true] {
        let clear=Arc::new(scene(false));let blocked=Arc::new(scene(true));
        let start=pose(if calibrated {-0.5001}else{-0.51},0.05,0.0);let goal=pose(0.55,0.05,0.001);
        let motion=MotionKinematics {model:MotionModel::Omnidirectional,max_forward_mps:0.4,
            max_reverse_mps:0.3,max_lateral_mps:0.3,max_angular_rps:0.6,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true};
        let mut config=SurfaceFollowerConfig::new(0.4);config.basic_kinematics=Some(motion);
        if calibrated {let mut declared=limits();declared.model=MotionModel::Omnidirectional;declared.max_lateral_mps=0.3;config.motion_limits=Some(declared);}
        config.controller.speed=0.4;config.controller.control_frequency=20.0;
        let mut follower=SurfaceFollower::new(config).unwrap();let now=1_000_000_000;
        follower.configure_goal(&stamped(goal,now),GoalOptions {position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI,..Default::default()}).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(clear)).unwrap();follower.update_odometry(stamped(start,now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:vec![stamped(start,now),stamped(goal,now)]}),None,stamped(goal,now)).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
        assert!(matches!(follower.step(0.3),PlannerStep::ReplanRequired(_)),
            "朝向已对齐，所有平移都被新障碍阻挡时必须重规划，完整标定={calibrated}");
        }
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
                    follower.observe_applied_command(command);
                    if command==Twist2::ZERO {
                        // 过零属于原路线控制接续，模拟也须经过控制器采用的真实制动时钟。
                        std::thread::sleep(std::time::Duration::from_secs_f64(motion.maximum_stopping_time_sec()));
                    }
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
                    // 位姿时间推进不能替代拥有方的实际制动时钟，零命令须经过原标定窗口。
                    if command==Twist2::ZERO {
                        std::thread::sleep(std::time::Duration::from_secs_f64(limits().maximum_stopping_time_sec()));
                    }
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
            assert!(connect(&map,pair[0],pair[1],&[],true,false,false,true).is_some());
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
    fn small_required_turn_precedes_translation_into_a_narrow_corridor() {
        for calibrated in [false,true] {
        let base=scene(false);let mut geometry=base.config().clone();
        geometry.voxel_size=0.025;geometry.wall_clearance_m=0.025;
        let mut points:Vec<_>=(-60..100).flat_map(|x|(-20..20).map(move |y|
            ((x as f32+0.5)*0.025,(y as f32+0.5)*0.025,0.0125))).collect();
        for x in -8..100 {for y in [-6,5] {points.push(((x as f32+0.5)*0.025,(y as f32+0.5)*0.025,0.55));}}
        let map=Arc::new(SurfaceSnapshot::from_points(&points,geometry,0.025).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        let mut current=([-0.498,0.0,0.425],[0.0,0.0,0.013_f64.sin(),0.013_f64.cos()]);
        let aligned=(current.0,[0.0,0.0,0.0,1.0]);let goal=([0.8,0.0,0.425],aligned.1);
        let mut route=vec![current,aligned];
        for i in 1..=27 {route.push(([(-0.498+0.05*i as f64).min(0.8),0.0,0.425],aligned.1));}
        for edge in route.windows(2) {map.validate_body_motion(edge[0],edge[1],&[]).unwrap();}
        let mut config=SurfaceFollowerConfig::new(0.4);config.controller.speed=0.35;config.controller.control_frequency=20.0;
        let mut motion=limits().kinematics();motion.model=MotionModel::Omnidirectional;motion.max_lateral_mps=0.35;
        config.basic_kinematics=Some(motion);
        if calibrated {let mut declared=limits();declared.model=MotionModel::Omnidirectional;declared.max_lateral_mps=0.35;config.motion_limits=Some(declared);}
        config.controller.simulation=true;
        let mut follower=SurfaceFollower::new(config).unwrap();let mut now=1_000_000_000;
        follower.configure_goal(&stamped(goal,now),GoalOptions {position_tolerance_m:0.12,
            yaw_tolerance_rad:std::f64::consts::PI,..Default::default()}).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(stamped(current,now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:route.into_iter().map(|p|stamped(p,now)).collect()}),None,stamped(goal,now)).unwrap();
        let mut reached=false;let mut first=true;
        for _ in 0..600 {
            match follower.step(0.3) {
                PlannerStep::Command(command)=>{
                    if first && !calibrated {assert!(command.angular_z<0.0 && command.linear.x.hypot(command.linear.y)<1e-9,
                        "窄口所需小角度必须先在原地完成: {command:?}");first=false;}
                    follower.observe_applied_command(command);
                    let next=integrate(planar(current),command,0.05);
                    let actual=([next.x,next.y,current.0[2]],[0.0,0.0,(next.yaw*0.5).sin(),(next.yaw*0.5).cos()]);
                    map.validate_body_motion(current,actual,&[]).unwrap();current=actual;
                }
                PlannerStep::Succeeded=>{reached=true;break;}
                other=>panic!("已规划的小转向不能被角度容差消耗后反复重规划: {other:?}"),
            }
            now+=50_000_000;follower.update_odometry(stamped(current,now));
        }
        assert!(reached);
        }
    }

    #[test]
    fn calibrated_rotation_keeps_actual_height_under_low_ceiling() {
        let base=scene(false);
        let points:Vec<_>=(-20..20).flat_map(|x|(-15..15).flat_map(move |y|
            [0.025,0.075,0.125,0.725].into_iter().map(move |z|
                ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,z)))).collect();
        let map=Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.05).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        let mut current=([0.05,0.05,0.46],[0.0,0.0,0.0,1.0]);
        let goal=(current.0,[0.0,0.0,0.5_f64.sin(),0.5_f64.cos()]);
        map.validate_body_pose(current.0,current.1).unwrap();
        let support=map.locate_support_from_body(current.0,current.1,0.4).unwrap();
        assert!(map.body_pose_on_surface([0.05,0.05],0.0,support.support_xyz).is_err(),
            "格顶投影会抬高实际机体撞到低顶棚");
        let options=GoalOptions {position_tolerance_m:0.1,yaw_tolerance_rad:0.05,..Default::default()};
        let path=plan(&map,current,goal,options,Some(&limits()),&[],2000).unwrap();
        let mut config=SurfaceFollowerConfig::new(0.4);config.motion_limits=Some(limits());
        config.controller.control_frequency=20.0;
        let mut follower=SurfaceFollower::new(config).unwrap();let mut now=1_000_000_000;
        follower.configure_goal(&stamped(goal,now),options).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(stamped(current,now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses:path.into_iter().map(|p|stamped(p,now)).collect()
        }),None,stamped(goal,now)).unwrap();
        let mut reached=false;
        for _ in 0..400 {
            match follower.step(0.3) {
                PlannerStep::Succeeded=>{reached=true;break;}
                PlannerStep::Command(command)=>{
                    follower.observe_applied_command(command);
                    let next=integrate(planar(current),command,0.05);
                    let actual=([next.x,next.y,current.0[2]],[0.0,0.0,(next.yaw*0.5).sin(),(next.yaw*0.5).cos()]);
                    map.validate_body_motion(current,actual,&[]).unwrap();current=actual;
                }
                other=>panic!("完整制动轨迹不得先改变实际机体高度: {other:?}"),
            }
            now+=50_000_000;follower.update_odometry(stamped(current,now));
        }
        assert!(reached,"低顶棚下原地转向应保持实际高度完成");
    }

    #[test]
    fn projected_goal_does_not_move_the_requested_three_dimensional_ball() {
        let map=scene(false);let start=pose(-0.55,0.05,0.0);
        let mut goal=pose(0.55,0.05,0.0);goal.0[2]-=0.1;
        let options=GoalOptions {position_tolerance_m:0.04,
            yaw_tolerance_rad:std::f64::consts::PI,..Default::default()};
        let prepared=prepare_goal_pose(&map,goal,false).unwrap();
        let direct=connect(&map,start,prepared,&[],true,false,false,false).unwrap();
        assert!(position_distance(*direct.last().unwrap(),goal)>options.position_tolerance_m,
            "该场景的地形投影确实超出原目标球");
        assert!(plan(&map,start,goal,options,Some(&limits()),&[],0).is_err(),
            "快速连接不能把投影后的另一个三维终点当作原目标到达");
    }

    #[test]
    fn same_surface_goal_still_requires_original_three_dimensional_tolerance() {
        let map = Arc::new(scene(false));
        let mut current = pose(0.05,0.05,0.0);
        let endpoint = pose(0.35,0.05,0.0);
        let mut goal = endpoint; goal.0[2] += 0.2;
        let options = GoalOptions { position_tolerance_m:0.35,
            yaw_tolerance_rad:std::f64::consts::PI, ..Default::default() };
        assert!(position_distance(current,goal)>options.position_tolerance_m);
        let mut follower = SurfaceFollower::new(SurfaceFollowerConfig::new(0.4)).unwrap();
        let mut now=1_000_000_000;
        follower.configure_goal(&stamped(goal,now),options).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(stamped(current,now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses:vec![stamped(current,now),stamped(endpoint,now)]
        }),None,stamped(goal,now)).unwrap();
        let mut reached=false;
        for tick in 0..100 {
            match follower.step(0.3) {
                PlannerStep::Succeeded=>{
                    assert!(tick>0,"同层仅满足水平容差不能提前成功");
                    assert!(position_distance(current,goal)<options.position_tolerance_m);reached=true;break;
                }
                PlannerStep::Command(command)=>{
                    let next=integrate(planar(current),command,0.05);
                    current=pose(next.x,next.y,next.yaw);
                }
                other=>panic!("同层三维目标应能持续接近: {other:?}"),
            }
            now+=50_000_000;follower.update_odometry(stamped(current,now));
        }
        assert!(reached);
    }

    #[test]
    fn forward_paths_replace_old_sideways_escape_routes() {
        for obstacle in [false, true] {
            let base = scene(false);
            let mut points: Vec<_> = (-35..35).flat_map(|x| (-20..20).map(move |y|
                ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))).collect();
            let mut envelope = base.body_envelope().unwrap().clone();
            envelope.footprint_xy_m = vec![[-0.31,-0.12],[0.31,-0.12],[0.31,0.12],[-0.31,0.12]];
            if obstacle { points.push((-0.35,0.25,0.55)); }
            let map = Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
                .with_body_envelope(envelope).unwrap());
            let mut current = pose(0.0,0.0,0.0);
            let goal = if obstacle { pose(0.06,1.0,0.0) } else { pose(-1.6,-0.2,0.0) };
            if obstacle {assert!(map.validate_body_motion(current,goal,&[]).is_err(),"不能跳过为转身腾出净空的前进段");}
            let options=GoalOptions {allow_reverse:true,position_tolerance_m:0.12,
                yaw_tolerance_rad:std::f64::consts::PI};
            let mut config=SurfaceFollowerConfig::new(0.4);
            config.controller.speed=0.35;config.controller.control_frequency=20.0;
            let mut motion=limits().kinematics();motion.model=MotionModel::Omnidirectional;motion.max_lateral_mps=0.35;
            config.basic_kinematics=Some(motion);
            let route=plan_with_motion_cancellable(&map,current,goal,options,Some(&motion),&[],2000,None).unwrap();
            assert_forward_path(&route);
            for pair in route.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            let mut follower=SurfaceFollower::new(config).unwrap();let mut now=1_000_000_000;
            follower.configure_goal(&stamped(goal,now),options).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
            follower.update_odometry(stamped(current,now));
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                poses:route.into_iter().map(|p|stamped(p,now)).collect()
            }),None,stamped(goal,now)).unwrap();
            let mut arrived=false;
            for tick in 0..600 {
                match follower.step(0.3) {
                    PlannerStep::Command(command) => {
                        assert!(command.linear.x>=0.0 && command.linear.y==0.0);
                        follower.observe_applied_command(command);
                        let next=integrate(planar(current),command,0.05);
                        let actual=pose(next.x,next.y,next.yaw);
                        map.validate_body_motion(current,actual,&[]).unwrap();current=actual;
                    }
                    PlannerStep::Succeeded=>{arrived=true;break;}
                    other=>panic!("前向完整路线不应反复重规划: {other:?}, obstacle={obstacle}, current={current:?}"),
                }
                now+=50_000_000;follower.update_odometry(stamped(current,now));
            }
            assert!(arrived,"前向路线必须到达: obstacle={obstacle},current={current:?}");
        }
    }

    #[test]
    fn seven_meter_global_route_is_followed_without_segment_replanning() {
        let base = scene(false);
        let points: Vec<_> = (-10..90).flat_map(|x| (-12..12).map(move |y|
            ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))).collect();
        let map = Arc::new(SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        let mut current = pose(0.05, 0.05, 0.0);
        let goal = pose(7.05, 0.05, 0.0);
        let options = GoalOptions { position_tolerance_m: 0.12,
            yaw_tolerance_rad: std::f64::consts::PI, ..Default::default() };
        let started = std::time::Instant::now();
        let route = plan(&map, current, goal, options, Some(&limits()), &[], 30_000).unwrap();
        let planning = started.elapsed();
        assert!(position_distance(*route.last().unwrap(), goal) < options.position_tolerance_m);
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.controller.speed = 0.3;
        config.controller.control_frequency = 20.0;
        config.basic_kinematics = Some(limits().kinematics());
        let mut follower = SurfaceFollower::new(config).unwrap();
        let mut now = 1_000_000_000;
        follower.configure_goal(&stamped(goal, now), options).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        follower.update_odometry(stamped(current, now));
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: route.into_iter().map(|p| stamped(p, now)).collect()
        }), None, stamped(goal, now)).unwrap();
        let mut arrived_at = None;
        let started = std::time::Instant::now();
        for tick in 0..700 {
            match follower.step(0.3) {
                PlannerStep::Command(command) => {
                    assert!(command.linear.x > 0.1, "直线路线不能在离散路径点反复停车: {command:?}");
                    follower.observe_applied_command(command);
                    let next = integrate(planar(current), command, 0.05);
                    let actual = pose(next.x, next.y, next.yaw);
                    map.validate_body_motion(current, actual, &[]).unwrap();
                    current = actual;
                }
                PlannerStep::Succeeded => { arrived_at = Some(tick); break; }
                other => panic!("完整路线不得短段接续: {other:?}"),
            }
            now += 50_000_000;
            follower.update_odometry(stamped(current, now));
        }
        let ticks = arrived_at.expect("七米路线应在35秒模拟时间内完成");
        eprintln!("七米完整路线：规划{planning:?}，跟随{}拍/{}秒，计算{:?}，误差{}米", ticks,
            ticks as f64 * 0.05, started.elapsed(), position_distance(current, goal));
        assert!(position_distance(current, goal) < options.position_tolerance_m);
    }

    #[test]
    fn one_meter_per_second_slows_for_translation_corner_without_replanning() {
        let base=scene(false);
        let mut points:Vec<_>=(-10..40).flat_map(|x|(-10..60).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        for y in -5..50 {points.push((3.55,(y as f32+0.5)*0.1,0.55));}
        let map=Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        for calibrated in [false,true] {
            let mut current=pose(0.05,0.05,0.0);let goal=pose(3.05,4.05,0.0);
            let mut route=vec![current];
            for i in 1..=60 {route.push(pose(0.05+0.05*i as f64,0.05,0.0));}
            for i in 1..=80 {route.push(pose(3.05,0.05+0.05*i as f64,0.0));}
            for pair in route.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            let mut motion=limits();motion.model=MotionModel::Omnidirectional;
            motion.max_forward_mps=1.0;motion.max_reverse_mps=1.0;motion.max_lateral_mps=1.0;
            let mut config=SurfaceFollowerConfig::new(0.4);config.controller.speed=1.0;config.controller.control_frequency=20.0;
            if calibrated {config.motion_limits=Some(motion.clone());}else{config.basic_kinematics=Some(motion.kinematics());}
            let mut follower=SurfaceFollower::new(config).unwrap();let mut now=1_000_000_000;
            follower.configure_goal(&stamped(goal,now),GoalOptions{allow_reverse:true,
                position_tolerance_m:0.12,yaw_tolerance_rad:std::f64::consts::PI}).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();follower.update_odometry(stamped(current,now));
            follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:route.into_iter().map(|p|stamped(p,now)).collect()}),None,stamped(goal,now)).unwrap();
            let mut applied=Twist2::ZERO;let mut peak:f64=0.0;let mut corner_speed:f64=1.0;let mut reached=None;
            for tick in 0..400 {
                let command=match follower.step(0.3) {
                    PlannerStep::Succeeded=>{reached=Some(tick as f64*0.05);break;}
                    PlannerStep::Command(command)=>command,
                    other=>panic!("高速弯道应保持原路线: {other:?}, calibrated={calibrated},pose={current:?}"),
                };
                if command==Twist2::ZERO {std::thread::sleep(std::time::Duration::from_millis(50));}
                let delta=Point2::new(command.linear.x-applied.linear.x,command.linear.y-applied.linear.y);
                let ratio=(0.05/delta.x.hypot(delta.y).max(1e-12)).min(1.0);
                let next_applied=Twist2 {linear:Point2::new(applied.linear.x+delta.x*ratio,applied.linear.y+delta.y*ratio),angular_z:command.angular_z};
                let mean=Twist2{linear:Point2::new((applied.linear.x+next_applied.linear.x)*0.5,(applied.linear.y+next_applied.linear.y)*0.5),angular_z:command.angular_z};
                let next=integrate(planar(current),mean,0.05);let actual=pose(next.x,next.y,next.yaw);
                map.validate_body_motion(current,actual,&[]).expect("高速拐弯不能越过真实墙体");current=actual;applied=next_applied;
                let speed=applied.linear.x.hypot(applied.linear.y);peak=peak.max(speed);
                if (current.0[0]-3.05).hypot(current.0[1]-0.05)<0.2 {corner_speed=corner_speed.min(speed);}
                follower.observe_applied_command(applied);now+=50_000_000;follower.update_odometry(stamped(current,now));
            }
            let elapsed=reached.expect("一米每秒七米折线应在20秒内完成");
            assert!(peak>0.99 && corner_speed<0.6,"直段应达巡航且转角应减速：peak={peak},corner={corner_speed}");
            assert!(position_distance(current,goal)<0.12);
            eprintln!("一米每秒七米折线：完整标定{calibrated}，到达{elapsed}s，最高{peak}，转角最低{corner_speed}");
        }
    }

    #[test]
    fn one_meter_per_second_follows_seven_meter_global_route_with_real_acceleration() {
        let base=scene(false);
        let points:Vec<_>=(-10..90).flat_map(|x|(-10..90).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        let map=Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        for calibrated in [false,true] { for lateral in [false,true] { for dt in [0.05,0.1] { for pose_stride in [1,4] { for jointed in [false,true] {
            // 两个世界方向都让机体正面沿路线，保持原巡航时间/精度要求。
            let mut current=pose(0.05,0.05,if lateral {std::f64::consts::FRAC_PI_2}else{0.0});
            let goal=if lateral {pose(0.05,7.05,std::f64::consts::FRAC_PI_2)} else {pose(7.05,0.05,0.0)};
            let options=GoalOptions{position_tolerance_m:if pose_stride==1 {0.12}else{0.35},yaw_tolerance_rad:0.05,..Default::default()};
            let mut motion=limits();motion.model=MotionModel::Omnidirectional;
            motion.max_forward_mps=1.0;motion.max_reverse_mps=1.0;motion.max_lateral_mps=1.0;
            let mut route=plan(&map,current,goal,options,Some(&motion),&[],30_000).unwrap();
            assert_forward_path(&route);
            if jointed {
                // 真实路线每20厘米连接不同地形动作时含同位置接点；接点不能成为隐性停点。
                route=route.into_iter().enumerate().flat_map(|(index,p)|if index%4==0 {vec![p,p]}else{vec![p]}).collect();
            }
            assert!(position_distance(*route.last().unwrap(),goal)<options.position_tolerance_m);
            let mut config=SurfaceFollowerConfig::new(0.4);
            config.controller.speed=1.0;config.controller.control_frequency=1.0/dt;
            if calibrated {config.motion_limits=Some(motion.clone());} else {config.basic_kinematics=Some(motion.kinematics());}
            let mut follower=SurfaceFollower::new(config).unwrap();let mut now=1_000_000_000;
            follower.configure_goal(&stamped(goal,now),options).unwrap();
            follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();follower.update_odometry(stamped(current,now));
            follower.replace_execution_path(ExecutionPath::Surface(Path3D{poses:route.into_iter().map(|p|stamped(p,now)).collect()}),None,stamped(goal,now)).unwrap();
            let mut applied=Twist2::ZERO;let mut peak:f64=0.0;let mut reached=None;let began=std::time::Instant::now();
            for tick in 0..(12.0/dt) as usize {
                let command=match follower.step(0.3) {
                    PlannerStep::Succeeded=>{reached=Some(tick as f64*dt);break;}
                    PlannerStep::Command(command)=>command,
                    other=>panic!("七米完整路线不得短段接续: {other:?}, calibrated={calibrated},lateral={lateral},dt={dt}"),
                };
                assert!(command.linear.x>=0.0 && command.linear.y==0.0,"巡航只能前进");
                // 独立执行器模型限制矢量加速度1米/秒²；只积分实际速度，不把位置吸到路径。
                let delta=Point2::new(command.linear.x-applied.linear.x,command.linear.y-applied.linear.y);
                let ratio=(dt/delta.x.hypot(delta.y).max(1e-12)).min(1.0);
                let next_applied=Twist2 {linear:Point2::new(applied.linear.x+delta.x*ratio,applied.linear.y+delta.y*ratio),angular_z:command.angular_z};
                let average=Twist2 {linear:Point2::new((applied.linear.x+next_applied.linear.x)*0.5,(applied.linear.y+next_applied.linear.y)*0.5),angular_z:command.angular_z};
                let next=integrate(planar(current),average,dt);let actual=pose(next.x,next.y,next.yaw);
                map.validate_body_motion(current,actual,&[]).unwrap();current=actual;applied=next_applied;
                peak=peak.max(applied.linear.x.hypot(applied.linear.y));
                follower.observe_applied_command(applied);now+=(dt*1e9)as i64;
                if (tick+1)%pose_stride==0 {follower.update_odometry(stamped(current,now));}
            }
            let elapsed=reached.expect("七米一米每秒路线须在12秒内完成");
            assert!(elapsed<=10.0,"巡航速度不能被默认值或前瞻截断：{elapsed}s, calibrated={calibrated},lateral={lateral},dt={dt}");
            assert!(peak>0.99,"真实积分应达到声明的一米每秒，实测{peak}");
            assert!(position_distance(current,goal)<options.position_tolerance_m);
            eprintln!("一米每秒七米：完整标定{calibrated}，沿Y轴{lateral}，周期{dt}s，位姿每{pose_stride}拍，含地形接点{jointed}，到达{elapsed}s，最高速度{peak}，计算{:?}",began.elapsed());
        }}}}}
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
        assert!(matches!(map.query_ground([start.0[0], start.0[1]], 0.1).state,
            nuwax_nav_mls_planner::surface::GroundQueryState::Unknown
                | nuwax_nav_mls_planner::surface::GroundQueryState::Estimated),
            "邻域估计可以提供坐标参考，原始盲区仍没有实测地面");
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
        assert!(matches!(map.query_ground([start.0[0], start.0[1]], 0.1).state,
            nuwax_nav_mls_planner::surface::GroundQueryState::Unknown
                | nuwax_nav_mls_planner::surface::GroundQueryState::Estimated),
            "执行不能给原始盲区补造实测地面");
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
        assert!(matches!(map.query_ground([start.0[0], start.0[1]], 0.1).state,
            nuwax_nav_mls_planner::surface::GroundQueryState::Unknown
                | nuwax_nav_mls_planner::surface::GroundQueryState::Estimated),
            "当前位置的缺测事实必须保留");
        let options = GoalOptions { allow_reverse: false, position_tolerance_m: 0.08,
            yaw_tolerance_rad: 0.08 };
        // 按当前生产PCT入口验证；旧plan接口明确要求目标有观测支撑。
        let planner = nuwax_nav_pct::PctPlanner::default();
        let path = plan_reference_cancellable(&view, &planner, start, start, options,
            Some(&limits()), &[], 2000, None)
            .expect("当前已验证姿态满足目标时，不能把缺测误当成碰撞");
        assert_eq!(path, vec![start]);
        let blocked = view.with_recent_obstacle_points(1, 100, 0, &[],
            [(0, [start.0[0], start.0[1], start.0[2]])]).unwrap();
        assert!(plan_reference_cancellable(&blocked, &planner, start, start, options,
            Some(&limits()), &[], 2000, None).is_err(),
            "目标就在当前位置也不能越过实际占据检查");
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
        // 到达请求使用真实机体高度；0.24米的目标高度差不能被0.1米容差静默抹掉。
        let goal = ([0.55, 0.05, 0.46], [0.0, 0.0, 0.3_f64.sin(), 0.3_f64.cos()]);
        let options = GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.1,
            yaw_tolerance_rad: 0.08,
        };
        let motion = limits();
        let outside = ([goal.0[0],goal.0[1],0.70],goal.1);
        assert!(plan(&map,start,outside,options,Some(&motion),&[],3000).is_err(),
            "地面投影不能替换原始三维目标球");
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
            // 与相邻动态障碍闭环一样，让实际制动窗口完成后再继续推进测试。
            if command==Twist2::ZERO {
                std::thread::sleep(std::time::Duration::from_secs_f64(limits().maximum_stopping_time_sec()));
            }
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
                assert!(path.iter().any(|p| angle(planar(*p).yaw-std::f64::consts::PI).abs()<0.1),
                    "后方目标优先转身前进，末端再恢复指定朝向");
            }
            let mut config = SurfaceFollowerConfig::new(0.4);
            config.motion_limits = Some(limits.clone());
            config.controller.control_frequency = 20.0;
            config.controller.simulation = true;
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
                follower.observe_applied_command(command);
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
                            plan(&map, current, goal, options, Some(&limits()), &[], 2000)
                                .unwrap_or_else(|error| panic!("台阶重规划失败 tick={tick}, from={current:?}, 原因={reason}, 规划={error}"));
                        follower
                            .replace_execution_path(
                                ExecutionPath::Surface(Path3D {
                                    poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                                }),
                                None,
                                stamped(goal, nanos),
                            )
                            .unwrap();
                        // 模拟回执真实零速度；制动器使用单调钟，位姿时间不能替代它的物理等待。
                        follower.observe_applied_command(Twist2::ZERO);
                        Twist2::ZERO
                    }
                    other => panic!("{other:?}"),
                };
                follower.observe_applied_command(command);
                if command == Twist2::ZERO {
                    std::thread::sleep(std::time::Duration::from_secs_f64(limits().maximum_stopping_time_sec()));
                }
                let next = integrate(planar(current), command, 0.05);
                // 实际站姿按同一连续地形变化推进，不能每拍重设到单列格顶，
                // 否则足端越过台阶边缘时会制造平面命令未执行的瞬间升降。
                let next = map
                    .body_pose_after_motion(current, [next.x, next.y], next.yaw)
                    .unwrap();
                map.validate_body_motion(current, next, &[]).unwrap();
                current = next;
                nanos += 50_000_000;
                follower.update_odometry(stamped(current, nanos));
            }
            assert!(reached, "下阶={descending},终态={current:?}");
        }
    }
    #[test]
    fn mission_reference_preserves_known_wall_bounded_turn() {
        let map = wall_bounded_turn_scene();
        let body = |x, y, yaw: f64| ([x,y,0.45], [0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()]);
        let start = body(-1.6, 0.0, 0.0);
        let goal = body(0.0, 1.6, std::f64::consts::FRAC_PI_2);
        let options = GoalOptions {allow_reverse:false,position_tolerance_m:0.12,yaw_tolerance_rad:0.15};
        let motion = limits();
        let legacy = plan(&map,start,goal,options,Some(&motion),&[],8000).unwrap();
        assert!(legacy.windows(2).all(|p|map.validate_body_motion(p[0],p[1],&[]).is_ok()));
        let planner = nuwax_nav_pct::PctPlanner::default();
        let path = plan_reference_cancellable(&map,&planner,start,goal,options,Some(&motion),&[],30_000,None)
            .expect("a previously usable wall-bounded turn must remain mission-reachable");
        assert!(path.windows(2).all(|p|map.validate_body_motion(p[0],p[1],&[]).is_ok()));
        let end = *path.last().unwrap();
        assert!(position_distance(end,goal)<=options.position_tolerance_m);
        assert!(angle(planar(end).yaw-planar(goal).yaw).abs()<=options.yaw_tolerance_rad);
    }

}
