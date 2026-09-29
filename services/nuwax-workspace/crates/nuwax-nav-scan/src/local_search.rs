//! 仅处理近场样条受阻的前向姿态接续；不承担长距离全局规划。
use super::{angle, yaw, BodyPose, LocalPlanError};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_pct::Tomogram;
use nuwax_nav_types_messages::MotionKinematics;
use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};
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
fn key(p: BodyPose, quantum: f64, angular: f64) -> (i32, i32, i32, i32) {
    (
        (p.0[0] / quantum).floor() as i32,
        (p.0[1] / quantum).floor() as i32,
        (p.0[2] / quantum).floor() as i32,
        (yaw(p.1).rem_euclid(std::f64::consts::TAU) / angular).floor() as i32,
    )
}

pub fn probe_motion(
    map: &SurfaceSnapshot,
    from: BodyPose,
    length: f64,
    turn: f64,
    boundary: &[[f64; 2]],
) -> Option<Vec<BodyPose>> {
    probe_motion_until(map,from,length,turn,boundary,|_|false)
}
fn probe_motion_until(
    map: &SurfaceSnapshot,
    from: BodyPose,
    length: f64,
    turn: f64,
    boundary: &[[f64;2]],
    reached: impl Fn(BodyPose)->bool,
) -> Option<Vec<BodyPose>> {
    let (poses,views)=project_motion_until(map,from,length,turn,reached)?;
    validate_projected(from,&poses,&views,boundary)?;
    Some(poses)
}
fn project_motion_until(
    map:&SurfaceSnapshot,from:BodyPose,length:f64,turn:f64,reached:impl Fn(BodyPose)->bool,
) -> Option<ProjectedMotion> {
    let radius = map.body_envelope()?.radius();
    let n = ((length + turn.abs() * radius) / 0.025).ceil().max(1.0) as usize;
    let heading = yaw(from.1);
    let mut previous = from;
    let mut poses = Vec::with_capacity(n);
    let mut views = Vec::with_capacity(n);
    for i in 1..=n {
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
        let (next,view)=super::project_endpoint(map,previous,xy,heading+theta).ok()?;
        previous=next;
        poses.push(previous);
        views.push(view);
    }
    // 端点几何已拒绝的候选不再支付沿途扫掠；任何返回的候选仍逐段核验全部运动。
    // 保留生成端点时的同一只读查询视图，不把尚未验证的未来站姿写回地图。
    Some((poses,views))
}
fn validate_projected(
    from:BodyPose,poses:&[BodyPose],views:&[nuwax_nav_mls_planner::SurfaceBodyQueryView],boundary:&[[f64;2]],
)->Option<()> {
    let mut previous=from;
    for (&next,view) in poses.iter().zip(views) {
        super::check_projected_motion(view,previous,next,boundary).ok()?;
        previous=next;
    }
    Some(())
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
        .fold(f64::INFINITY, f64::min)
}
fn reference_progress(p: [f64; 3], reference: &[[f64; 3]]) -> (f64, f64, f64) {
    let mut best = (f64::INFINITY, 0.0, reference[0][2]);
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
    let target = *reference.last().ok_or("近场搜索缺少目标")?;
    let offset = map
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
    let tangent = reference.windows(2).last().map_or(yaw(start.1), |pair| {
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
            reference_progress([p.0[0], p.0[1], p.0[2] - offset], reference);
        if distance(p) > corridor
            || deviation > corridor
            || progress < reference_length - 1e-6
            || (p.0[2] - offset - height).abs() > layer_tolerance
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
            reference_progress([p.0[0], p.0[1], p.0[2] - offset], reference);
        (reference_length - progress).max(0.0) + deviation
    };
    // 搜索状态按源精度合并，实际候选仍保留完整站姿并逐段核对，不量化机体或障碍。
    let quantum = f64::from(map.source_voxel_size_m()).clamp(0.025, 0.1);
    let angular = 0.1;
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        samples: Vec::new(),
    }];
    let mut open = BinaryHeap::from([Entry(remaining(start) * 4.0, 0)]);
    let mut best = HashMap::from([(key(start, quantum, angular), 0.0)]);
    let mut expanded = 0;
    let mut closest = start;
    while let Some(Entry(_, id)) = open.pop() {
        let from = nodes[id].pose;
        let cost = nodes[id].cost;
        if best
            .get(&key(from, quantum, angular))
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
            let xy=[target[0]-tangent.sin()*offset,target[1]+tangent.cos()*offset];
            let dx=xy[0]-from.0[0];let dy=xy[1]-from.0[1];
            let heading=dy.atan2(dx);let distance=dx.hypot(dy);
            let error=angle(heading-yaw(from.1));
            let line_time=distance/motion.max_forward_mps+error.abs()/motion.max_angular_rps;
            proposals.push((line_time,heading,distance,0.0));
            if error.abs()>1e-6 && error.abs()<std::f64::consts::FRAC_PI_2 {
                let length=distance*error/error.sin();let turn=2.0*error;
                if motion.minimum_turning_radius_m==0.0 || length/turn.abs()>=motion.minimum_turning_radius_m {
                    let time=(length/motion.max_forward_mps).max(turn.abs()/motion.max_angular_rps);
                    proposals.push((time,yaw(from.1),length,turn));
                }
            }
        }
        // 按声明线速、角速算接续时间，优先核验可执行的直线或单一圆弧。
        // 不再按固定离散转角拼出波浪，也不为省一段几何检查放弃真实净空。
        proposals.sort_by(|a,b|a.0.total_cmp(&b.0));
        for (_,heading,length,turn) in proposals {
            // 解析长接续先用层析中心路线排除穿墙提案；未通过仍可由原姿态动作搜索绕行。
            // 这一步不授予机体许可，入选提案随后保留完整地形、旋转和扫掠检查。
            let samples=(length/(tomogram.resolution*0.5)).ceil().max(1.0) as usize;
            let mut previous=[from.0[0],from.0[1],from.0[2]-offset];
            let mut visible=true;
            for i in 1..=samples {
                let t=i as f64/samples as f64;
                let theta=turn*t;
                let (dx,dy)=if turn.abs()<1e-8 {(length*t,0.0)}
                    else {(length/turn*theta.sin(),length/turn*(1.0-theta.cos()))};
                let point=[from.0[0]+dx*heading.cos()-dy*heading.sin(),
                    from.0[1]+dx*heading.sin()+dy*heading.cos(),
                    (from.0[2]-offset)*(1.0-t)+target[2]*t];
                if arrival.is_some_and(|(goal,tolerance)|(point[0]-goal[0]).hypot(point[1]-goal[1])<tolerance) {break;}
                if !tomogram.segment_clear(previous,point,boundary) {visible=false;break;}
                previous=point;
            }
            if !visible {continue;}
            let mut prefix=vec![from];
            if super::rotate(map,&mut prefix,heading,motion,boundary).is_err() {continue;}
            let in_goal=|pose:BodyPose|arrival.is_some_and(|(goal,tolerance)|super::distance3(pose.0,goal)<tolerance);
            let Some(line)=probe_motion_until(map,*prefix.last().unwrap(),length,turn,boundary,in_goal)else{continue;};
            prefix.extend(line);
            if let Some(heading)=final_yaw {
                if super::rotate(map,&mut prefix,heading,motion,boundary).is_err(){continue;}
            }
            if reach(*prefix.last().unwrap()) {
                connection=Some(prefix);break;
            }
        }
        if reach(from) || connection.is_some() {
            let mut chain = Vec::new();
            let mut current = id;
            while let Some(parent) = nodes[current].parent {
                chain.push(current);
                current = parent;
            }
            chain.reverse();
            let mut path = vec![start];
            for id in chain {
                path.extend_from_slice(&nodes[id].samples);
            }
            if let Some(connection)=connection {path.extend(connection.into_iter().skip(1));}
            return Ok(path);
        }
        expanded += 1;
        if expanded > 256 {
            break;
        }
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
        let mut forward_available=false;
        let mut dominated_coarse:Vec<ProjectedMotion>=Vec::new();
        for (index,(length, turn)) in actions.into_iter().enumerate() {
            if index==9 && !forward_available {
                // 被搜索代价淘汰的粗动作，只在决定是否需要细动作时核验一条可行前进边。
                // 保持原粗细动作选择语义，避免延后检查改变路径与转弯形状。
                forward_available=dominated_coarse.iter().any(|(poses,views)|
                    validate_projected(from,poses,views,boundary).is_some());
            }
            if index>=9 && forward_available {break;}
            if length == 0.0 && !motion.can_rotate_in_place {
                continue;
            }
            if length > 0.0 && turn != 0.0 && length / turn.abs() < motion.minimum_turning_radius_m
            {
                continue;
            }
            let Some((samples,views)) = project_motion_until(map, from, length, turn, |_|false) else {
                continue;
            };
            let next = *samples.last().unwrap();
            if (next.0[0] - start.0[0]).hypot(next.0[1] - start.0[1]) > distance(start) + 1.0 {
                if index<9 && length>0.0 && !forward_available {dominated_coarse.push((samples,views));}
                continue;
            }
            let support = [next.0[0], next.0[1], next.0[2] - offset];
            let terrain_cost = tomogram
                .cell_at(support)
                .map_or(2.0, |id| tomogram.cells[id].cost);
            let next_cost = cost
                + length.max(0.04) * (1.0 + 0.15 * terrain_cost)
                + turn.abs() * 0.12
                + reference_distance(next.0, reference) * 0.1;
            let k = key(next, quantum, angular);
            if best.get(&k).is_some_and(|&old| next_cost >= old) {
                if index<9 && length>0.0 && !forward_available {dominated_coarse.push((samples,views));}
                continue;
            }
            // 已有合法路径代价更低时不重复做扫掠；真正入队的边仍完整核验。
            if validate_projected(from,&samples,&views,boundary).is_none() {continue;}
            if index<9 && length>0.0 {forward_available=true;}
            best.insert(k,next_cost);
            let next_id = nodes.len();
            nodes.push(Node {
                pose: next,
                cost: next_cost,
                parent: Some(id),
                samples,
            });
            let heading_cost = final_yaw.map_or(0.0, |y| angle(y - yaw(next.1)).abs() * 0.05);
            open.push(Entry(
                next_cost + 4.0 * remaining(next) + heading_cost,
                next_id,
            ));
        }
    }
    // 搜索已经绕过早期样条失败处时，反馈实际接续困难的位置。
    // 它只影响当前目标的有限路线代价，不将搜索预算耗尽当作永久障碍。
    Err(LocalPlanError {
        reason: format!(
            "近场前向搜索未连通（{expanded}次展开，最近{:.3}米，姿态{closest:?}）",
            distance(closest)
        ),
        blocked: Some(closest.0),
    })
}
