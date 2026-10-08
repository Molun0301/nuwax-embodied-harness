//! Scan式路线引导近场规划：优化三次样条，按真实地形生成可执行机体轨迹。
//! 全局层析路线只作参考，局部轨迹在有限视距内处理转向、机体碰撞与接触。
mod cancellation;
mod local_search;
mod spline;
#[cfg(feature="profile")]
mod profile;
pub use local_search::probe_motion;
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_pct::Tomogram;
pub use nuwax_nav_pct::{PlanningCancellation, planning_cancelled, with_planning_cancellation};
use nuwax_nav_types_messages::MotionKinematics;
pub type BodyPose = ([f64; 3], [f64; 4]);
#[derive(Clone, Debug)]
pub struct ScanConfig {
    /// 局部参考长度，单位米。
    pub horizon_m: f64,
    /// 重新生成近场轨迹前的最小进度，单位米。
    pub refresh_distance_m: f64,
    /// 样条控制点优化迭代次数。
    pub optimization_iterations: usize,
    /// 中间参考走廊，生产拥有方与既有路线偏差范围同步。
    pub reference_corridor_m: f64,
}
impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            horizon_m: 5.0,
            refresh_distance_m: 1.0,
            optimization_iterations: 12,
            reference_corridor_m: 0.9,
        }
    }
}
impl ScanConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.horizon_m.is_finite()
            || self.horizon_m <= 0.0
            || !self.refresh_distance_m.is_finite()
            || self.refresh_distance_m <= 0.0
            || !self.reference_corridor_m.is_finite()
            || self.reference_corridor_m <= 0.0
            || self.optimization_iterations > 100
        {
            return Err("近场视距、更新距离或样条迭代次数无效".into());
        }
        Ok(())
    }
}
fn admit_candidate<T>(result: Result<T, LocalPlanError>) -> Result<Option<T>, LocalPlanError> {
    match result {
        Ok(value) => Ok(Some(value)),
        Err(error) if matches!(error.kind, LocalPlanFailureKind::Cancelled) => Err(error),
        Err(_) => Ok(None),
    }
}
/// A finite local search is not a proof of physical infeasibility.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocalPlanFailureKind {
    /// Superseded work is neither a search failure nor physical obstruction.
    Cancelled,
    BudgetExhausted,
    SearchExhausted,
    Unclassified,
}
#[derive(Clone, Debug, PartialEq)]
pub struct LocalPlanError {
    pub kind: LocalPlanFailureKind,
    /// Closest explored pose, diagnostic only; never an obstacle measurement.
    pub closest_pose: Option<BodyPose>,
    pub reason: String,
    pub blocked: Option<[f64; 3]>,
}
impl std::fmt::Display for LocalPlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.reason)
    }
}
impl From<String> for LocalPlanError {
    fn from(reason: String) -> Self {
        Self {
            kind: if planning_cancelled() { LocalPlanFailureKind::Cancelled }
                else { LocalPlanFailureKind::Unclassified },
            reason,
            closest_pose: None,
            blocked: None,
        }
    }
}
impl From<&str> for LocalPlanError {
    fn from(reason: &str) -> Self {
        reason.to_owned().into()
    }
}
#[derive(Debug)]
pub struct LocalTrajectory {
    pub poses: Vec<BodyPose>,
    pub smoothed: bool,
    pub elapsed_ms: f64,
}
fn distance3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
fn yaw(q: [f64; 4]) -> f64 {
    (2.0 * (q[3] * q[2] + q[0] * q[1])).atan2(1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
}
fn angle(v: f64) -> f64 {
    v.sin().atan2(v.cos())
}
fn advance(
    map: &SurfaceSnapshot,
    from: BodyPose,
    xy: [f64; 2],
    heading: f64,
    boundary: &[[f64; 2]],
) -> Result<BodyPose, LocalPlanError> {
    cancellation::checkpoint()?;
    let (next,view)=project_endpoint(map,from,xy,heading)?;
    check_projected_motion(&view,from,next,boundary)?;
    Ok(next)
}
/// 候选先生成完整端点站姿；中间扫掠在候选可能采用时单独核验。
fn project_endpoint(
    map:&SurfaceSnapshot,from:BodyPose,xy:[f64;2],heading:f64,
) -> Result<(BodyPose,nuwax_nav_mls_planner::SurfaceBodyQueryView),LocalPlanError> {
    #[cfg(feature="profile")]
    let began=std::time::Instant::now();
    cancellation::checkpoint()?;
    let view=map.body_query_with_current_support(from.0,from.1);
    #[cfg(feature="profile")]
    profile::record(0,began,view.is_err());
    cancellation::checkpoint()?;
    let view=view.map_err(|e|e.to_string())?;
    #[cfg(feature="profile")]
    let began=std::time::Instant::now();
    let next=view.body_pose_after_motion(from,xy,heading);
    #[cfg(feature="profile")]
    profile::record(1,began,next.is_err());
    cancellation::checkpoint()?;
    let mut next=next.map_err(|e|e.to_string())?;
    if (xy[0] - from.0[0]).hypot(xy[1] - from.0[1]) < 1e-9 {
        next.0[2] = from.0[2];
    }
    Ok((next,view))
}
fn check_projected_motion(
    view:&nuwax_nav_mls_planner::SurfaceBodyQueryView,from:BodyPose,next:BodyPose,boundary:&[[f64;2]],
) -> Result<(),LocalPlanError> {
    #[cfg(feature="profile")]
    let began=std::time::Instant::now();
    cancellation::checkpoint()?;
    let checked=view.validate_body_motion(from,next,boundary);
    #[cfg(feature="profile")]
    profile::record(2,began,checked.is_err());
    cancellation::checkpoint()?;
    checked.map_err(|e|e.to_string())?;
    Ok(())
}
fn rotate(
    map: &SurfaceSnapshot,
    poses: &mut Vec<BodyPose>,
    heading: f64,
    kinematics: &MotionKinematics,
    boundary: &[[f64; 2]],
) -> Result<(), LocalPlanError> {
    cancellation::checkpoint()?;
    let from = *poses.last().unwrap();
    let change = angle(heading - yaw(from.1));
    if change.abs() < 1e-4 {
        return Ok(());
    }
    if !kinematics.can_rotate_in_place {
        return Err("当前机型不能原地完成局部轨迹转向".into());
    }
    let n = (change.abs() / 0.08).ceil().max(1.0) as usize;
    // 对后方目标试两个实际旋转方向，左右机体扫掠空间可能不同。
    for delta in [change, change - change.signum() * std::f64::consts::TAU] {
        cancellation::checkpoint()?;
        let n = if delta == change {
            n
        } else {
            (delta.abs() / 0.08).ceil() as usize
        };
        let mut candidate = Vec::with_capacity(n);
        let mut previous = from;
        let mut failed = false;
        for i in 1..=n {
            cancellation::checkpoint()?;
            // 原地转向保持实际中心和相对地面倾斜；不能把地面法向绕世界Z轴转走。
            let result = advance(
                map,
                previous,
                [from.0[0], from.0[1]],
                yaw(from.1) + delta * i as f64 / n as f64,
                boundary,
            );
            let next = match result {
                Ok(next) => next,
                Err(error) if matches!(error.kind, LocalPlanFailureKind::Cancelled) => return Err(error),
                Err(_) => { failed = true; break; }
            };
            candidate.push(next);
            previous = next;
        }
        if !failed {
            poses.extend(candidate);
            return Ok(());
        }
    }
    Err("近场初始转向被实际机体障碍阻断".into())
}
fn project(
    map: &SurfaceSnapshot,
    start: BodyPose,
    points: &[[f64; 3]],
    kinematics: &MotionKinematics,
    smooth: bool,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    boundary: &[[f64; 2]],
    blocked: &std::cell::Cell<Option<[f64; 3]>>,
) -> Result<Vec<BodyPose>, LocalPlanError> {
    cancellation::checkpoint()?;
    let mut poses = vec![start];
    let mut valid = Vec::new();
    for &point in points {
        cancellation::checkpoint()?;
        if (point[0] - start.0[0]).hypot(point[1] - start.0[1]) > 1e-5 { valid.push(point); }
    }
    if valid.is_empty() {
        if arrival.is_some_and(|(goal, tolerance)| distance3(start.0, goal) >= tolerance) {
            return Err("近场参考尚未到达原三维目标区域".into());
        }
        if let Some(yaw) = final_yaw {
            rotate(map, &mut poses, yaw, kinematics, boundary)?;
        }
        return Ok(poses);
    }
    let first = valid[0];
    rotate(
        map,
        &mut poses,
        (first[1] - start.0[1]).atan2(first[0] - start.0[0]),
        kinematics,
        boundary,
    )?;
    for (index, point) in valid.iter().enumerate() {
        cancellation::checkpoint()?;
        let from = *poses.last().unwrap();
        let dx = point[0] - from.0[0];
        let dy = point[1] - from.0[1];
        let length = dx.hypot(dy);
        if length < 1e-6 {
            continue;
        }
        let tangent = if smooth {
            valid.get(index + 1).map_or(dy.atan2(dx), |next| {
                (next[1] - from.0[1]).atan2(next[0] - from.0[0])
            })
        } else {
            dy.atan2(dx)
        };
        let change = angle(tangent - yaw(from.1));
        if smooth && change.abs() > std::f64::consts::FRAC_PI_2 {
            return Err("样条存在切向反折，需要显式转向接续".into());
        }
        if !smooth && change.abs() > 0.03 {
            rotate(map, &mut poses, tangent, kinematics, boundary)?;
        }
        if smooth
            && kinematics.minimum_turning_radius_m > 0.0
            && change.abs() / length > 1.0 / kinematics.minimum_turning_radius_m + 1e-6
        {
            return Err("局部样条曲率超过当前机型能力".into());
        }
        let previous = *poses.last().unwrap();
        let mut xy = [point[0], point[1]];
        if let Some((goal, tolerance)) = arrival {
            // 在进入原三维到达球的位置截取末段，不强迫机体走到不可站立的点击中心。
            let delta = [xy[0] - previous.0[0], xy[1] - previous.0[1]];
            let dz = previous.0[2] - goal[2];
            let endpoint_distance = (xy[0] - goal[0]).hypot(xy[1] - goal[1]).hypot(dz);
            if endpoint_distance < tolerance && delta[0] * delta[0] + delta[1] * delta[1] > 1e-12 {
                let mut low = 0.0;
                let mut high = 1.0;
                for _ in 0..24 {
                    cancellation::checkpoint()?;
                    let t = (low + high) * 0.5;
                    let distance = (previous.0[0] + delta[0] * t - goal[0])
                        .hypot(previous.0[1] + delta[1] * t - goal[1])
                        .hypot(dz);
                    if distance < tolerance * (1.0 - 1e-5) {
                        high = t;
                    } else {
                        low = t;
                    }
                }
                xy = [
                    previous.0[0] + delta[0] * high,
                    previous.0[1] + delta[1] * high,
                ];
            }
        }
        let next = advance(map, previous, xy, tangent, boundary).map_err(|mut error| {
            if !matches!(error.kind, LocalPlanFailureKind::Cancelled)
                && blocked.get().is_none() {
                blocked.set(Some([xy[0], xy[1], previous.0[2]]));
            }
            error.reason = format!("轨迹点{index}({:.3},{:.3})：{}", xy[0], xy[1], error.reason);
            error
        })?;
        // 地形投影不得跳至层析参考的另一楼层。
        let offset = map.body_envelope().unwrap().body_to_support_z_m;
        if (next.0[2] - offset - point[2]).abs()
            > f64::from(map.config().step_threshold_m + map.config().voxel_size) * 2.0
        {
            return Err("局部地形与参考路线不在同一高度层".into());
        }
        poses.push(next);
        if arrival.is_some_and(|(goal, tolerance)| {
            ((next.0[0] - goal[0]).powi(2)
                + (next.0[1] - goal[1]).powi(2)
                + (next.0[2] - goal[2]).powi(2))
            .sqrt()
                < tolerance
        }) {
            break;
        }
    }
    if arrival
        .is_some_and(|(goal, tolerance)| distance3(poses.last().unwrap().0, goal) >= tolerance)
    {
        return Err("地形投影后的末点尚未到达原三维目标区域".into());
    }
    if let Some(heading) = final_yaw {
        rotate(map, &mut poses, heading, kinematics, boundary)?;
    }
    cancellation::checkpoint()?;
    Ok(poses)
}
/// 参考坐标为支撑面；起点保留实际机体完整站姿，必要时执行完整近场搜索。
pub fn plan(
    map: &SurfaceSnapshot,
    tomogram: &Tomogram,
    start: BodyPose,
    reference: &[[f64; 3]],
    kinematics: &MotionKinematics,
    config: &ScanConfig,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    boundary: &[[f64; 2]],
    follow_reference: bool,
    prefer_straight: bool,
) -> Result<LocalTrajectory, LocalPlanError> {
    cancellation::checkpoint()?;
    let result = plan_impl(map, tomogram, start, reference, kinematics, config, final_yaw,
        arrival, boundary, follow_reference, prefer_straight);
    cancellation::checkpoint()?;
    result
}

fn plan_impl(
    map: &SurfaceSnapshot,
    tomogram: &Tomogram,
    start: BodyPose,
    reference: &[[f64; 3]],
    kinematics: &MotionKinematics,
    config: &ScanConfig,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    boundary: &[[f64; 2]],
    follow_reference: bool,
    prefer_straight: bool,
) -> Result<LocalTrajectory, LocalPlanError> {
    #[cfg(feature="profile")]
    let _profile=profile::Scope::begin(start,reference,kinematics,config,final_yaw,arrival,follow_reference,prefer_straight);
    cancellation::checkpoint()?;
    local_search::validate_caps(kinematics)?;
    let began = std::time::Instant::now();
    let blocked = std::cell::Cell::new(None);
    if reference.is_empty() {
        return Err("近场参考路线为空".into());
    }
    let spacing = (f64::from(map.source_voxel_size_m()) * 0.75).clamp(0.01, 0.05);
    let radius = map
        .body_envelope()
        .ok_or("近场规划缺少机体尺寸")?
        .footprint_xy_m
        .iter()
        .map(|p| p[1].abs())
        .fold(0.0, f64::max);
    let model = map.body_envelope().unwrap();
    let xmin = model
        .footprint_xy_m
        .iter()
        .map(|p| p[0])
        .fold(f64::INFINITY, f64::min);
    let xmax = model
        .footprint_xy_m
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max);
    let centers = [(xmin + radius).min(0.0), (xmax - radius).max(0.0)];
    let mut failures = Vec::new();
    let interior = arrival.map(|(goal, tolerance)| (goal, tolerance * 0.8));
    // 可执行直线优先保留，不用初始偏航或墙距梯度把它改成蛇形。
    if let Some(&end)=reference.last() {
        let origin=reference[0];
        let dx=end[0]-origin[0];let dy=end[1]-origin[1];let length=dx.hypot(dy);
        if length>1e-6 {
            let mut previous=0.0;
            let straight=reference.iter().all(|p| {
                let along=((p[0]-origin[0])*dx+(p[1]-origin[1])*dy)/length;
                let deviation=((p[0]-origin[0])*dy-(p[1]-origin[1])*dx).abs()/length;
                let valid=deviation<=f64::from(map.source_voxel_size_m())*0.5
                    && along+1e-9>=previous;
                previous=along;valid
            });
            // 全局折点只表达可通行走廊，不代表每个点都需要转向。
            // 直弦仍逐段检查真实三维地形和完整机体；允许在参考走廊内省去无必要折点。
            let in_corridor=reference.iter().all(|p| {
                let along=((p[0]-origin[0])*dx+(p[1]-origin[1])*dy)/length;
                let deviation=((p[0]-origin[0])*dy-(p[1]-origin[1])*dx).abs()/length;
                along>=-1e-9 && along<=length+1e-9 && deviation<=config.reference_corridor_m
            });
            // 起点到首个地面格的短接头不应强迫走折线；后续参考必须仍是同一直段。
            // 窗口内存在真实转角时交给下方圆滑候选，避免直弦提前切掉转弯接续。
            let tail=reference.get(1).copied().unwrap_or(origin);
            let tx=end[0]-tail[0];let ty=end[1]-tail[1];let tl=tx.hypot(ty);
            let shortcut=in_corridor && (tail[0]-origin[0]).hypot(tail[1]-origin[1])<=model.radius()*2.0
                && (tl<1e-9 || reference[1..].iter().all(|p| {
                    let along=((p[0]-tail[0])*tx+(p[1]-tail[1])*ty)/tl;
                    let deviation=((p[0]-tail[0])*ty-(p[1]-tail[1])*tx).abs()/tl;
                    along>=-1e-9 && along<=tl+1e-9 && deviation<=f64::from(map.source_voxel_size_m())
                }));
            if straight || prefer_straight || shortcut {
                let samples=spline::resample(&[origin,end],spacing)?;
                for region in [interior,arrival] {
                    cancellation::checkpoint()?;
                    if let Some(poses)=admit_candidate(project(map,start,&samples,kinematics,false,final_yaw,region,boundary,&blocked))? {
                        #[cfg(feature="profile")]
                        profile::success();
                        return Ok(LocalTrajectory {poses,smoothed:false,
                            elapsed_ms:began.elapsed().as_secs_f64()*1000.0});
                    }
                    if arrival.is_none(){break;}
                }
            }
        }
    }
    // 已沿局部走廊前进后，粗参考的近处折点可能落在侧后方。
    // 尝试从实际位置直达较远的参考折点，保留末段切向供下一窗口接续。
    // 完整候选仍核验地形、机体和转向，不能用仅中心可见或整窗直弦切掉真正转角。
    for join in (2..reference.len().saturating_sub(1)).rev() {
        cancellation::checkpoint()?;
        let origin = reference[0];
        let heading = yaw(start.1);
        let first = reference[1];
        if (first[0] - origin[0]) * heading.cos() + (first[1] - origin[1]) * heading.sin() >= 0.0
            || reference[1..join].iter().any(|p| (p[0] - origin[0]).hypot(p[1] - origin[1]) > model.radius() * 2.0)
        { continue; }
        let end = reference[join];
        let dx = end[0] - origin[0];
        let dy = end[1] - origin[1];
        let length2 = dx * dx + dy * dy;
        if length2 < 1e-12 { continue; }
        let inside = reference[1..join].iter().all(|p| {
            let t = (((p[0] - origin[0]) * dx + (p[1] - origin[1]) * dy) / length2).clamp(0.0, 1.0);
            (p[0] - origin[0] - dx * t).hypot(p[1] - origin[1] - dy * t) <= config.reference_corridor_m
        });
        if !inside { continue; }
        let mut guide = vec![origin];
        guide.extend_from_slice(&reference[join..]);
        let controls = spline::resample(&guide, 0.25_f64.max(tomogram.resolution))?;
        let samples = spline::sample(&controls, spacing)?;
        // 捷径受阻只否决该候选，不能把它的碰撞点当作原参考走廊受阻的证据。
        let shortcut_blocked = std::cell::Cell::new(None);
        if let Some(poses) = admit_candidate(project(map, start, &samples, kinematics, true, final_yaw, interior, boundary, &shortcut_blocked))? {
            #[cfg(feature="profile")]
            profile::success();
            return Ok(LocalTrajectory { poses, smoothed: true, elapsed_ms: began.elapsed().as_secs_f64()*1000.0 });
        }
    }
    // 首段方向属于全局路线的接续意图。固定该切向，先完成实际转向，
    // 不把未完成的偏航旋转混进平移曲线，也不让平滑提前切掉首个拐点。
    let departure=reference.iter().find(|p|(p[0]-start.0[0]).hypot(p[1]-start.0[1])>1e-5)
        .map_or(yaw(start.1),|p|(p[1]-start.0[1]).atan2(p[0]-start.0[0]));
    // 先沿参考直段生成仅在拐点圆滑的样条；碰撞距离优化只用于受阻候选。
    // 无条件推动全部控制点会把可通行的直段也推向离散墙距梯度，产生无必要的蛇形。
    for optimize_obstacles in [false, true] {
        cancellation::checkpoint()?;
        let controls = if optimize_obstacles { spline::optimize(
            reference,
            tomogram,
            radius,
            centers,
            config.optimization_iterations,
            Some(departure),
        ) } else { spline::resample(reference, 0.25_f64.max(tomogram.resolution)) }?;
        let sampled = spline::sample(&controls, spacing)?;
        for goal_region in [interior, arrival] {
            cancellation::checkpoint()?;
            match project(
                map,
                start,
                &sampled,
                kinematics,
                true,
                final_yaw,
                goal_region,
                boundary,
                &blocked,
            ) {
                Ok(poses) => {
                    #[cfg(feature="profile")]
                        profile::success();
                        return Ok(LocalTrajectory {
                        poses,
                        smoothed: true,
                        elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
                    })
                }
                Err(reason) => {
                    if matches!(reason.kind, LocalPlanFailureKind::Cancelled) { return Err(reason); }
                    tracing::debug!(target:"navigation.scan",optimize_obstacles,?reason,
                        "近场参考候选未通过机体检查");
                    failures.push(reason)
                },
            }
            if arrival.is_none() {
                break;
            }
        }
    }
    // 极窄空间可能需要显式转向；原折线也经过同一真实地形及机体检查。
    let sampled = spline::resample(reference, spacing)?;
    if let Some(poses) = admit_candidate(project(
        map, start, &sampled, kinematics, false, final_yaw, interior, boundary, &blocked,
    ))? {
        #[cfg(feature="profile")]
                        profile::success();
                        return Ok(LocalTrajectory {
            poses,
            smoothed: false,
            elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
        });
    }
    // 细门缝的中线可能落在全局粗格之间，直接使用近场真实占据给出的连续空隙。
    let mut attempted = 0;
    for pair in reference.windows(2) {
        cancellation::checkpoint()?;
        let dx = pair[1][0] - pair[0][0];
        let dy = pair[1][1] - pair[0][1];
        let length = dx.hypot(dy);
        if length < 1e-6 {
            continue;
        }
        let heading = dy.atan2(dx);
        let count = (length / 0.3).ceil().max(1.0) as usize;
        for i in 0..count {
            cancellation::checkpoint()?;
            if attempted >= 12 {
                break;
            }
            attempted += 1;
            let t = (i as f64 + 0.5) / count as f64;
            let center = [
                pair[0][0] + t * dx,
                pair[0][1] + t * dy,
                pair[0][2] + model.body_to_support_z_m,
            ];
            let Ok(candidates) =
                map.body_passage_candidates(center, heading, [0.15, model.radius() * 2.0])
            else {
                continue;
            };
            for candidate in candidates.into_iter().take(2) {
                cancellation::checkpoint()?;
                if candidate.clear_width_m > radius * 2.0 + 2.0 * f64::from(map.config().voxel_size)
                {
                    continue;
                }
                let support = |p: [f64; 3]| [p[0], p[1], p[2] - model.body_to_support_z_m];
                let mut guide = vec![reference[0]];
                let entry = candidate.entry_xyz;
                if (entry[0] - start.0[0]) * heading.cos() + (entry[1] - start.0[1]) * heading.sin()
                    > 0.0
                {
                    guide.push(support(entry));
                }
                guide.push(support(candidate.exit_xyz));
                guide.push(*reference.last().unwrap());
                let samples = spline::resample(&guide, spacing)?;
                if let Some(poses) = admit_candidate(project(
                    map, start, &samples, kinematics, false, final_yaw, arrival, boundary, &blocked,
                ))? {
                    #[cfg(feature="profile")]
                        profile::success();
                        return Ok(LocalTrajectory {
                        poses,
                        smoothed: false,
                        elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
                    });
                }
            }
        }
    }
    let poses = match project(
        map, start, &sampled, kinematics, false, final_yaw, arrival, boundary, &blocked,
    ) {
        Ok(poses) => poses,
        Err(polyline) => {
            if matches!(polyline.kind, LocalPlanFailureKind::Cancelled) { return Err(polyline); }
            cancellation::checkpoint()?;
            // 后备搜索也遵守起步方向：可原地完成后方转向时先对齐，
            // 不能因样条失败又把尚未完成的转向混进前进动作。
            let mut prefix=vec![start];
            let departure_distance=reference.iter().find(|p|(p[0]-start.0[0]).hypot(p[1]-start.0[1])>1e-5)
                .map_or(0.0,|p|(p[0]-start.0[0]).hypot(p[1]-start.0[1]));
            // 首个拐点位于机体尺度内时，不能用起步圆弧吞掉这段方向。
            // 更长的段已经尝试过直线和样条；它们失败后允许搜索必要的绕障曲线。
            let search_start=if departure_distance<=model.radius()*2.0
                && angle(departure-yaw(start.1)).abs()>std::f64::consts::FRAC_PI_2
                && admit_candidate(rotate(map,&mut prefix,departure,kinematics,boundary))?.is_some() {
                *prefix.last().unwrap()
            }else{start};
            local_search::plan(
            map,
            tomogram,
            search_start,
            reference,
            kinematics,
            final_yaw,
            arrival,
            boundary,
            config.horizon_m,
            config.reference_corridor_m,
            follow_reference,
        )
        .map(|poses| {
            prefix.extend(poses.into_iter().skip(1));
            prefix
        })
        .map_err(|local| LocalPlanError {
            reason: format!(
                "样条：{}；折线：{polyline}；{local}",
                failures.first().map_or("未生成", |failure| failure.reason.as_str())
            ),
            ..local
        })?
        },
    };
    #[cfg(feature="profile")]
    profile::success();
    Ok(LocalTrajectory {
        poses,
        smoothed: false,
        elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_mls_planner::{
        mls_planner::Config, SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations,
    };
    use nuwax_nav_types_messages::MotionModel;
    use std::{collections::BTreeMap, sync::Arc};
    pub(super) fn motion() -> MotionKinematics {
        MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 1.0,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        }
    }
    fn scene(door: Option<i32>) -> SurfaceSnapshot {
        scene_with_slope(door, 0.0)
    }
    fn scene_with_slope(door: Option<i32>, slope: f64) -> SurfaceSnapshot {
        scene_with_extra_cells(door, slope, &[])
    }
    fn scene_with_extra_cells(door: Option<i32>, slope: f64, extra: &[[i32; 3]]) -> SurfaceSnapshot {
        let size = 0.01_f32;
        let max_step = if slope == 0.0 { 0.0 } else { 0.15 };
        let mut cells: Vec<[i32; 3]> = (-130..131)
            .flat_map(|x| {
                (-70..100).map(move |y| [x, y, ((f64::from(x) + 0.5) * slope - 0.5).floor() as i32])
            })
            .collect();
        if let Some(width) = door {
            cells.extend((0..4).flat_map(|x| {
                (-70..100)
                    .filter(move |&y| y < 0 || y >= width)
                    .flat_map(move |y| (0..65).map(move |z| [x, y, z]))
            }));
        }
        cells.extend_from_slice(extra);
        let points: Vec<_> = cells
            .iter()
            .map(|p| {
                (
                    (p[0] as f32 + 0.5) * size,
                    (p[1] as f32 + 0.5) * size,
                    (p[2] as f32 + 0.5) * size,
                )
            })
            .collect();
        let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
        for key in cells {
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 1,
                        hits: 2,
                    },
                );
        }
        let mut observations = SurfaceObservations::new(size, 32).unwrap();
        for (key, values) in blocks {
            observations.replace_block(key, values).unwrap();
        }
        observations.mark_complete_geometry_source();
        let cfg = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.55,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            max_slope_deg: 18.0,
            step_threshold_m: max_step,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        SurfaceSnapshot::from_points(&points, cfg, size)
            .unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]],
                z_bounds_m: [-0.15, 0.15],
                body_to_support_z_m: 0.4,
                max_step_up_m: f64::from(max_step),
                max_step_down_m: f64::from(max_step),
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap()
            .with_observations(Some(Arc::new(observations)))
    }
    fn fine_escape_scene(mirror: bool, shift: [i32; 2], coarse_dead_end: bool, half_width_cells: i32) -> (SurfaceSnapshot, BodyPose) {
        let mut obstacles = vec![];
        for z in 25..55 {
            obstacles.push([-29, half_width_cells, z]);
            obstacles.push([-29, -half_width_cells-1, z]);
            if coarse_dead_end {
                for x in 34..61 { for y in [-half_width_cells-1,half_width_cells+1] { obstacles.push([x,y,z]); } }
                for y in -half_width_cells-1..half_width_cells+2 { obstacles.push([60,y,z]); }
            } else {
                for y in -50..20 { obstacles.push([37,y,z]); }
            }
        }
        for p in &mut obstacles {
            p[0] += shift[0];
            p[1] = (if mirror { -p[1]-1 } else { p[1] }) + shift[1];
        }
        // Extend the independent flat support for the mirrored swept body.
        obstacles.extend((-130..131).flat_map(|x|(-100..-70).map(move |y|[x,y,-1])));
        let half_width=half_width_cells as f64*0.01-0.0001;
        let map = scene_with_extra_cells(None, 0.0, &obstacles).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.3,-half_width],[0.3,-half_width],[0.3,half_width],[-0.3,half_width]],
            z_bounds_m: [-0.15,0.15], body_to_support_z_m: 0.4, max_step_up_m: 0.0, max_step_down_m: 0.0,
            max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
        }).unwrap();
        let start = ([0.011+shift[0] as f64*0.01,
            shift[1] as f64*0.01,0.4], [0.0,0.0,0.0,1.0]);
        (map,start)
    }
    fn report_path_metrics(name: &str, path: &[BodyPose]) {
        let (mut length,mut turning,mut time)=(0.0_f64,0.0_f64,0.0_f64);
        for pair in path.windows(2) {
            let d=distance3(pair[0].0,pair[1].0);
            let a=angle(yaw(pair[1].1)-yaw(pair[0].1)).abs();
            length+=d; turning+=a; time+=(d/motion().max_forward_mps).max(a/motion().max_angular_rps);
        }
        eprintln!("path_metrics name={name} length={length:.9} turning={turning:.9} nominal_time={time:.9}");
    }
    fn check_fine_escape(mirror: bool, shift: [i32; 2], coarse_dead_end: bool, half_width_cells: i32) {
        let (map,start)=fine_escape_scene(mirror,shift,coarse_dead_end,half_width_cells);
        let sign=if mirror {-1.0} else {1.0};
        map.validate_body_pose(start.0,start.1).unwrap();
        for turn in [0.08,-0.08,0.3,-0.3] {
            assert!(probe_motion(&map,start,0.0,turn,&[]).is_none(), "premature turn {turn}");
        }
        for turn in [0.1,-0.1] {
            assert!(probe_motion(&map,start,0.01,turn,&[]).is_none(), "curved fine departure must collide before clearing rear pins");
        }
        assert_eq!(probe_motion(&map,start,0.25,0.0,&[]).is_some(),coarse_dead_end);
        for (length,turn) in [(0.2,0.25),(0.2,-0.25),(0.12,0.45),(0.12,-0.45)] {
            assert!(probe_motion(&map,start,length,turn,&[]).is_none(), "coarse arc must collide");
        }
        // This witness is constructed before invoking search. Every segment goes through
        // the full map body-motion validator, not just endpoint or planner assertions.
        let mut witness=vec![start];
        witness.extend(probe_motion(&map,start,0.01,0.0,&[]).expect("one centimetre departure"));
        let depart=*witness.last().unwrap();
        witness.extend(probe_motion(&map,depart,0.0,sign*std::f64::consts::FRAC_PI_2,&[]).expect("turn after departure"));
        let turned=*witness.last().unwrap();
        witness.extend(probe_motion(&map,turned,0.5,0.0,&[]).expect("escape"));
        for pair in witness.windows(2) { map.validate_body_motion(pair[0],pair[1],&[]).unwrap(); }
        if coarse_dead_end {
            let coarse=*probe_motion(&map,start,0.25,0.0,&[]).unwrap().last().unwrap();
            assert!(probe_motion(&map,coarse,0.25,0.0,&[]).is_none(),"coarse forward terminates at the cap");
            for turn in [0.3,-0.3,sign*std::f64::consts::FRAC_PI_2] {
                assert!(probe_motion(&map,coarse,0.0,turn,&[]).is_none(),"coarse endpoint cannot turn out");
            }
        }
        let goal=witness.last().unwrap().0;
        let tomo=Tomogram::from_surface(&map).unwrap();
        let began=std::time::Instant::now();
        let result=local_search::plan(&map,&tomo,start,&[[start.0[0],start.0[1],0.0],[goal[0],goal[1],0.0]],
            &motion(),None,Some((goal,0.015)),&[],3.0,0.9,false);
        eprintln!("fine_escape mirror={mirror} shift={shift:?} coarse_dead_end={coarse_dead_end} half_width_cells={half_width_cells} result={:?} elapsed_ms={:.3}",
            result.as_ref().map(|p|p.len()),began.elapsed().as_secs_f64()*1000.0);
        let path=result.expect("the independently swept fine escape must be searchable");
        report_path_metrics("fine_escape",&path);
        assert!(distance3(path.last().unwrap().0,goal)<0.015);
        assert!(path.iter().any(|p| (p.0[0]-depart.0[0]).abs()<1e-8 && (p.0[1]-depart.0[1]).abs()<1e-8));
        for pair in path.windows(2) { map.validate_body_motion(pair[0],pair[1],&[]).unwrap(); }
    }
    #[test]
    fn fine_actions_admit_both_small_turns_even_with_legal_coarse_forward() {
        let map=scene(None);
        let start=([0.0101,0.005,0.4],[0.0,0.0,0.0,1.0]);
        let tomo=Tomogram::from_surface(&map).unwrap();
        let failure=local_search::with_expansion_limit(1,||local_search::plan(&map,&tomo,start,
            &[[0.0101,0.005,0.0],[0.3,0.005,0.0]],&motion(),None,
            Some(([0.3,0.005,2.0],0.01)),&[],3.0,0.9,false)).unwrap_err();
        assert_eq!(failure.kind,LocalPlanFailureKind::BudgetExhausted);
        assert!(failure.blocked.is_none());
        let admitted=local_search::initial_admissions();
        assert_eq!(admitted.len(),14,"all legal existing actions, including both fine turn signs");
        for (length,turn) in [(0.01,0.0),(0.01,0.1),(0.01,-0.1),(0.0,0.08),(0.0,-0.08)] {
            let pose=admitted.iter().find(|(l,t,_)|*l==length && *t==turn).expect("fine action admitted").2;
            assert_eq!(pose,*probe_motion(&map,start,length,turn,&[]).unwrap().last().unwrap());
        }
    }

    #[test]
    fn fine_escape_one_centimetre_then_turn_with_mirrors_and_translations() {
        let results:Vec<_>=[(false,[0,0]),(true,[0,0]),(false,[5,10]),(true,[-5,10])].into_iter()
            .map(|(mirror,shift)|std::panic::catch_unwind(||check_fine_escape(mirror,shift,false,2))).collect();
        assert!(results.iter().all(Result::is_ok),"every transformed witness must pass");
    }
    #[test]
    fn matched_existing_search_success_across_old_quantization_boundary() {
        // Same swept geometry, shifted so the old 2.5 cm key admitted its 1 cm departure.
        // This is an actual nonzero-search baseline success, not an analytic fast path.
        check_fine_escape(false,[1,0],false,2);
        assert_eq!(local_search::last_work().0,1);
    }
    #[test]
    fn fine_escape_wide_body_geometry() {
        let results:Vec<_>=[false,true].into_iter().map(|mirror|
            std::panic::catch_unwind(||check_fine_escape(mirror,[0,0],false,16))).collect();
        assert!(results.iter().all(Result::is_ok),"both wide-body witnesses must pass");
    }
    #[test]
    fn fine_escape_when_legal_coarse_forward_enters_a_dead_end() {
        let results:Vec<_>=[false,true].into_iter().map(|mirror|
            std::panic::catch_unwind(||check_fine_escape(mirror,[0,0],true,2))).collect();
        assert!(results.iter().all(Result::is_ok),"both coarse-dead-end witnesses must pass");
    }

    #[test]
    fn nearby_reference_behind_departure_rejoins_forward_without_erasing_exit_tangent() {
        let map = scene(None);
        let tomo = Tomogram::from_surface(&map).unwrap();
        let heading = std::f64::consts::PI;
        let start = ([0.5, 0.0, 0.4], [0.0, 0.0, (heading * 0.5).sin(), (heading * 0.5).cos()]);
        let reference = [[0.5, 0.0, 0.0], [0.52, 0.3, 0.0], [-0.1, 0.3, 0.0], [-0.5, 0.5, 0.0]];
        let trajectory = plan(&map, &tomo, start, &reference, &motion(), &ScanConfig::default(),
            None, None, &[], false, false).unwrap();
        let departure = trajectory.poses.iter().find(|p| (p.0[0]-start.0[0]).hypot(p.0[1]-start.0[1])>1e-5).unwrap();
        assert!(angle(yaw(departure.1)-heading).abs()<0.6,
            "近处侧后方粗参考不应要求先转过直角，已有完整前向接续应优先");
        let end = trajectory.poses.last().unwrap();
        assert!((end.0[0]+0.5).hypot(end.0[1]-0.5)<1e-8);
        assert!(angle(yaw(end.1)-0.2_f64.atan2(-0.4)).abs()<0.01,
            "下一窗口的末段切向须保留，不能整窗拉直提前切掉出口转弯");
    }

    #[test]
    fn cancelled_scan_stages_are_typed_and_never_report_obstruction() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let reference = [[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]];
        let run = || plan(&map, &tomo, start, &reference, &motion(), &ScanConfig::default(),
            None, None, &[], false, false);
        for checkpoints in [0, 8, 80] {
            let error = cancellation::after_checkpoints(checkpoints, run).unwrap_err();
            assert_eq!(error.kind, LocalPlanFailureKind::Cancelled);
            assert_eq!(error.blocked, None);
            assert_eq!(error.closest_pose, None);
        }
        let baseline = run().expect("cancelled work must not poison a later valid plan");
        let token = PlanningCancellation::new(Arc::new(std::sync::atomic::AtomicU64::new(9)), 9);
        let scoped = with_planning_cancellation(&token, run).unwrap();
        assert_eq!(baseline.poses, scoped.poses);
        assert_eq!(baseline.smoothed, scoped.smoothed);
    }
    #[test]
    fn cancelled_search_exits_connector_and_expansion_work_without_negative_evidence() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        for checkpoints in [3, 40, 400] {
            let error = cancellation::after_checkpoints(checkpoints, || local_search::plan(
                &map, &tomo, start, &[[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]], &motion(),
                None, Some(([0.1, 0.2, 2.0], 0.1)), &[], 3.0, 0.9, true)).unwrap_err();
            assert_eq!(error.kind, LocalPlanFailureKind::Cancelled);
            assert_eq!(error.blocked, None);
            assert_eq!(error.closest_pose, None);
        }
    }
    #[test]
    fn executable_straight_reference_does_not_bend_to_initial_yaw() {
        let map=scene(None);let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([-0.8,0.2,0.4],[0.0,0.0,(0.3_f64/2.0).sin(),(0.3_f64/2.0).cos()]);
        let trajectory=plan(&map,&tomo,start,&[[-0.8,0.2,0.0],[0.0,0.2,0.0],[0.8,0.2,0.0]],
            &motion(),&ScanConfig::default(),None,None,&[],false,false).unwrap();
        assert!(trajectory.poses.iter().all(|p|(p.0[1]-0.2).abs()<1e-8),
            "可执行直线不能为贴合初始偏航变成波浪");
        let first=trajectory.poses.iter().find(|p|(p.0[0]-start.0[0]).abs()>1e-6).unwrap();
        assert!(yaw(first.1).abs()<1e-8,"应先对齐直线再平移");
    }
    #[test]
    fn rear_departure_finishes_alignment_to_the_selected_clear_straight() {
        let map=scene(None);let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([0.8,0.0,0.4],[0.0,0.0,0.0,1.0]);
        let trajectory=plan(&map,&tomo,start,&[[0.8,0.0,0.0],[0.3,0.0,0.0],[-0.8,0.5,0.0]],
            &motion(),&ScanConfig::default(),None,None,&[],false,false).unwrap();
        let first=trajectory.poses.iter().find(|p|distance3(p.0,start.0)>1e-6).unwrap();
        let heading=(0.5_f64).atan2(-1.6);
        assert!(angle(yaw(first.1)-heading).abs()<1e-8,
            "可通直线省去短接头后，仍须完成整段转向才能前进：{first:?}");
        assert!(trajectory.poses.iter().all(|p| ((p.0[0]-start.0[0])*0.5+(p.0[1]-start.0[1])*1.6).abs()<1e-8),
            "明确可通的后方直线不能弯曲");
    }

    #[test]
    fn analytic_connection_enters_goal_region_before_blocked_click_center() {
        let map=scene(Some(0));
        let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([-0.8,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let goal=[-0.1,0.2,0.4];
        assert!(map.validate_body_pose(goal,start.1).is_err(),"点击中心的机体确实与墙相交");
        let path=local_search::plan(&map,&tomo,start,&[[-0.8,0.2,0.0],[-0.1,0.2,0.0]],
            &motion(),None,Some((goal,0.35)),&[],3.0,0.9,false).unwrap();
        assert!(distance3(path.last().unwrap().0,goal)<0.35);
        assert!(path.iter().all(|p|(p.0[1]-0.2).abs()<1e-9),"可直入目标球时不应绕圈寻找点击中心");
        for pair in path.windows(2) {
            map.validate_body_motion(pair[0],pair[1],&[]).unwrap();
        }
    }

    #[test]
    fn reversed_departure_through_offset_doorway() {
        let map=scene(Some(40));let tomo=Tomogram::from_surface(&map).unwrap();
        for sign in [1.0_f64,-1.0] {
            let heading=if sign>0.0 {0.0}else{std::f64::consts::PI};
            let start=([sign*0.8,-0.3,0.4],[0.0,0.0,(heading*0.5).sin(),(heading*0.5).cos()]);
            let reference=[[sign*0.8,-0.3,0.0],[sign*0.5,0.2,0.0],[-sign*0.5,0.2,0.0],[-sign*0.8,-0.3,0.0]];
            // Independently establish a swept feasible route before asking the search.
            let mut witness=vec![start];
            for target in &reference[1..] {
                let from=*witness.last().unwrap();let dx=target[0]-from.0[0];let dy=target[1]-from.0[1];
                rotate(&map,&mut witness,dy.atan2(dx),&motion(),&[]).unwrap();
                witness.extend(probe_motion(&map,*witness.last().unwrap(),dx.hypot(dy),0.0,&[]).unwrap());
            }
            for pair in witness.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            let goal=[-sign*0.8,-0.3,0.4];
            let path=local_search::plan(&map,&tomo,start,&reference,&motion(),None,Some((goal,0.1)),&[],3.0,0.9,true).unwrap();
            assert_eq!(path[0],start);
            assert!(distance3(path.last().unwrap().0,goal)<0.1);
            for pair in path.windows(2) {
                map.validate_body_motion(pair[0],pair[1],&[]).unwrap();
                let dx=pair[1].0[0]-pair[0].0[0];let dy=pair[1].0[1]-pair[0].0[1];
                assert!(dx*yaw(pair[0].1).cos()+dy*yaw(pair[0].1).sin()>=-1e-9);
            }
        }
    }

    #[test]
    fn deferred_candidate_sweeps_still_reject_concave_boundary_crossing() {
        let map=scene(None);
        let start=([-0.8,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let path=probe_motion(&map,start,1.6,0.0,&[]).unwrap();
        assert!(path.last().unwrap().0[0]>0.79);
        // 端点站姿全部位于无障碍平面，但同一路径会穿出凹多边形的中间缺口。
        let boundary=[[-1.2,-0.8],[1.2,-0.8],[1.2,0.8],[0.1,0.8],
            [0.1,-0.2],[-0.1,-0.2],[-0.1,0.8],[-1.2,0.8]];
        assert!(nuwax_nav_pct::inside_boundary([start.0[0],start.0[1]],&boundary));
        assert!(nuwax_nav_pct::inside_boundary([path.last().unwrap().0[0],0.2],&boundary));
        assert!(probe_motion(&map,start,1.6,0.0,&boundary).is_none(),
            "检查顺序优化不能以端点有效替代全部扫掠许可");
    }
    #[test]
    fn future_pose_cannot_clear_an_obstacle_as_if_already_occupied() {
        let source=scene(Some(0));
        let actual=([-0.8,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let map=source.with_current_body_support(actual.0,actual.1).unwrap();
        let hypothetical=([0.01,0.2,0.4],[0.0,0.0,0.0,1.0]);
        assert!(map.validate_body_pose(hypothetical.0,hypothetical.1).is_err());
        assert!(advance(&map,hypothetical,[0.035,0.2],0.0,&[]).is_err(),
            "未到达的候选姿态不能把真实墙体当作当前自体占据清除");
    }

    #[test]
    fn close_goal_turns_in_place_and_respects_boundary() {
        let map = scene(None);
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([0.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let route = plan(
            &map,
            &tomo,
            start,
            &[[0.0, 0.2, 0.0]],
            &motion(),
            &ScanConfig::default(),
            Some(1.0),
            Some((start.0, 0.1)),
            &[],
            false,
            false,
        )
        .unwrap();
        assert!((yaw(route.poses.last().unwrap().1) - 1.0).abs() < 1e-6);
        assert!(route.poses.iter().all(|p| p.0 == start.0));
        let boundary = [[-0.6, -0.3], [0.3, -0.3], [0.3, 0.7], [-0.6, 0.7]];
        assert!(plan(
            &map,
            &tomo,
            start,
            &[[0.0, 0.2, 0.0], [0.9, 0.2, 0.0]],
            &motion(),
            &ScanConfig::default(),
            None,
            None,
            &boundary,
            false,
            false,
        )
        .is_err());
    }
    pub(super) fn coarse_open_floor() -> SurfaceSnapshot {
        let points: Vec<_> = (-15..20).flat_map(|x| (-8..15).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,-0.05))).collect();
        let config = Config { world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.55,
            surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: 0.0,
            wall_buffer_m: 0.1, wall_buffer_weight: 2.0, max_slope_deg: 18.0,
            step_threshold_m: 0.2, min_step_tread_depth_m: 0.2, step_penalty_weight: 1.0,
            goal_tolerance: 0.1, viz_publish_hz: 0.0 };
        SurfaceSnapshot::from_points(&points,config,0.1).unwrap().with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m: [-0.1,0.1], body_to_support_z_m: 0.4,
            max_step_up_m: 0.2, max_step_down_m: 0.2, max_cross_slope_deg: 18.0,
            support_uncertainty_m: 0.0,
        }).unwrap()
    }
    #[test]
    fn search_objective_handles_point_references_and_invalid_speed_caps() {
        let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([0.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        for reference in [vec![[0.0,0.2,0.0]],vec![[0.0,0.2,0.0];3]] {
            let path=local_search::plan(&map,&tomo,start,&reference,&motion(),Some(0.5),
                Some((start.0,0.01)),&[],3.0,0.9,true).unwrap();
            assert!(angle(yaw(path.last().unwrap().1)-0.5).abs()<1e-6);
            for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            for invalid in [0.0,-1.0,f64::NAN,f64::INFINITY,f64::NEG_INFINITY] {
                for angular in [true,false] {
                    let mut caps=motion();
                    if angular {caps.max_angular_rps=invalid;}else{caps.max_forward_mps=invalid;}
                    let failure=local_search::plan(&map,&tomo,start,&reference,&caps,None,None,&[],3.0,0.9,true).unwrap_err();
                    assert!(failure.blocked.is_none());
                    let failure=plan(&map,&tomo,start,&reference,&caps,&ScanConfig::default(),None,None,&[],true,true).unwrap_err();
                    assert!(failure.blocked.is_none());
                }
            }
        }
    }
    #[test]
    fn objective_degenerate_reference_reach_without_arrival() {
        let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([0.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        for reference in [vec![[0.0,0.2,0.0]],vec![[0.0,0.2,0.0];3],
            vec![[0.0,0.2,0.0],[0.5,0.7,0.0],[0.5,0.7,0.0]]] {
            let path=local_search::plan(&map,&tomo,start,&reference,&motion(),None,None,&[],3.0,0.9,true).unwrap();
            for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            let end=path.last().unwrap();let target=reference.last().unwrap();
            let tangent=if target[0]>0.0 {std::f64::consts::FRAC_PI_4}else{0.0};
            let dx=end.0[0]-target[0];let dy=end.0[1]-target[1];
            assert!((dx*tangent.cos()+dy*tangent.sin()).abs()<1e-6);
            assert!(dx.hypot(dy)<=0.81+1e-9);
            assert!(angle(yaw(end.1)-tangent).cos()>0.0);
        }
    }
    #[test]
    fn final_yaw_connector_ranking_uses_complete_validated_motion() {
        let results: Vec<_>=[(1.0,[0.0,0.0]),(-1.0,[0.0,0.0]),(1.0,[0.3,0.2]),(-1.0,[0.3,0.2])]
            .into_iter().map(|(sign,shift)| std::panic::catch_unwind(|| {
                let map=coarse_open_floor(); let tomo=Tomogram::from_surface(&map).unwrap();
                let start=([shift[0],shift[1],0.4],[0.0,0.0,0.0,1.0]);
                let heading=sign*0.5_f64;
                let goal=[shift[0]+heading.cos(),shift[1]+heading.sin(),0.4];
                let time=|path: &[BodyPose]|path.windows(2).map(|p|
                    (distance3(p[0].0,p[1].0)/motion().max_forward_mps)
                    .max(angle(yaw(p[1].1)-yaw(p[0].1)).abs()/motion().max_angular_rps)).sum::<f64>();
                let mut line=vec![start]; rotate(&map,&mut line,heading,&motion(),&[]).unwrap();
                line.extend(probe_motion(&map,*line.last().unwrap(),1.0,0.0,&[]).unwrap());
                let mut arc=vec![start]; arc.extend(probe_motion(&map,start,heading/heading.sin(),2.0*heading,&[]).unwrap());
                rotate(&map,&mut arc,heading,&motion(),&[]).unwrap();
                for path in [&line,&arc] {
                    assert!(distance3(path.last().unwrap().0,goal)<1e-6);
                    for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
                }
                assert!(time(&line)+0.2<time(&arc));
                let began=std::time::Instant::now();
                let path=local_search::plan(&map,&tomo,start,&[[shift[0],shift[1],0.0],[goal[0],goal[1],0.0]],
                    &motion(),Some(heading),Some((goal,0.001)),&[],3.0,0.9,false).unwrap();
                eprintln!("final_yaw_case sign={sign} shift={shift:?} elapsed_ms={:.3} line_time={} arc_time={}",
                    began.elapsed().as_secs_f64()*1000.0,time(&line),time(&arc));
                report_path_metrics("final_yaw",&path);
                for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
                assert!(distance3(path.last().unwrap().0,goal)<0.001);
                assert!(angle(yaw(path.last().unwrap().1)-heading).abs()<1e-6);
                assert!(time(&path)<=time(&line)+1e-6,"required final yaw must participate before ranking");
            })).collect();
        assert!(results.iter().all(Result::is_ok));
    }
    #[test]
    fn matched_search_workloads() {
        let map=coarse_open_floor();
        let tomo=Tomogram::from_surface(&map).unwrap();
        let start=([-1.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
        let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];
        let excluded=[[2.0,2.0],[3.0,2.0],[3.0,3.0],[2.0,3.0]];
        for repeat in 0..3 {
            for name in ["open_connection","budget16","drained_boundary"] {
                let boundary=if name=="drained_boundary" {&excluded[..]} else {&[]};
                let arrival=if name=="budget16" {Some(([0.1,0.2,2.0],0.1))} else {None};
                let began=std::time::Instant::now();
                let result=local_search::with_expansion_limit(16,||local_search::plan(&map,&tomo,start,
                    &reference,&motion(),None,arrival,boundary,3.0,0.9,true));
                eprintln!("matched_workload name={name} repeat={repeat} result={:?} elapsed_ms={:.3}",
                    result.as_ref().map(|p|p.len()),began.elapsed().as_secs_f64()*1000.0);
                if name=="open_connection" {report_path_metrics(name,result.as_ref().unwrap());assert!(result.is_ok());} else {
                    let failure=result.unwrap_err();
                    assert_eq!(failure.kind,if name=="budget16" {LocalPlanFailureKind::BudgetExhausted}
                        else {LocalPlanFailureKind::SearchExhausted});
                    assert!(failure.blocked.is_none());
                }
            }
        }
    }

    #[test]
    fn reduced_budget_open_floor_search_has_diagnostics_not_obstruction() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let run = || local_search::plan(&map, &tomo, start,
            &[[-1.0,0.2,0.0], [0.1,0.2,0.0]], &motion(), None, None, &[], 3.0, 0.9, true);
        assert!(run().is_ok(), "the unbounded fixture has an open-floor connection");
        // Force a finite attempt using the production branch with a reduced test budget.
        // The elevated arrival ball is unconnected, but is not an obstacle measurement.
        let failure = local_search::with_expansion_limit(0, || local_search::plan(&map, &tomo, start,
            &[[-1.0,0.2,0.0], [0.1,0.2,0.0]], &motion(), None,
            Some(([0.1,0.2,2.0],0.1)), &[], 3.0, 0.9, true)).unwrap_err();
        assert_eq!(failure.kind, LocalPlanFailureKind::BudgetExhausted);
        assert_eq!(failure.closest_pose, Some(start));
        assert_eq!(failure.blocked, None);
    }

    #[test]
    fn production_expansion_limit_is_budget_exhaustion_without_physical_evidence() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0,0.2,0.4], [0.0,0.0,0.0,1.0]);
        // A goal ball above the only floor prevents arrival without blocking the open terrain.
        let failure = local_search::plan(&map, &tomo, start,
            &[[-1.0,0.2,0.0],[0.1,0.2,0.0]], &motion(), None,
            Some(([0.1,0.2,2.0],0.1)), &[], 3.0, 0.9, true).unwrap_err();
        assert_eq!(failure.kind, LocalPlanFailureKind::BudgetExhausted);
        assert!(failure.closest_pose.is_some());
        let (expanded,nodes,frontier)=local_search::last_work();
        assert_eq!(expanded,256);
        assert!(nodes<=1+14*256 && frontier<=14*256);
        assert!(failure.blocked.is_none());
    }

    #[test]
    fn exhausted_search_is_distinct_and_outer_projection_cannot_supply_obstruction() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        // Open terrain with a boundary excluding every outgoing motion drains the frontier.
        let boundary = [[2.0,2.0], [3.0,2.0], [3.0,3.0], [2.0,3.0]];
        let reference = [[-1.0,0.2,0.0], [0.1,0.2,0.0]];
        let drained = local_search::plan(&map, &tomo, start, &reference,
            &motion(), None, None, &boundary, 3.0, 0.9, true).unwrap_err();
        assert_eq!(drained.kind, LocalPlanFailureKind::SearchExhausted);
        assert!(drained.blocked.is_none());
        let wrapped = local_search::with_expansion_limit(0, || plan(&map, &tomo, start, &reference,
            &motion(), &ScanConfig::default(), None, None, &boundary, true, false)).unwrap_err();
        let wrapped_drained = plan(&map, &tomo, start, &reference,
            &motion(), &ScanConfig::default(), None, None, &boundary, true, false).unwrap_err();
        assert_eq!(wrapped_drained.kind, LocalPlanFailureKind::SearchExhausted);
        assert!(wrapped_drained.blocked.is_none());
        assert_eq!(wrapped.kind, LocalPlanFailureKind::BudgetExhausted);
        assert!(wrapped.closest_pose.is_some());
        assert!(wrapped.blocked.is_none(), "old projection rejection must not leak into a search failure");
        assert_eq!(LocalPlanError::from("input unavailable").kind, LocalPlanFailureKind::Unclassified);
    }

    #[test]
    fn local_window_connects_to_the_forward_cross_section_on_open_ground() {
        let map = scene(None);
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let path = local_search::plan(
            &map, &tomo, start, &[[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]],
            &motion(), None, None, &[], 3.0, 0.9, true,
        ).unwrap();
        assert!(path.last().unwrap().0[0] >= 0.1 - 1e-6);
        for pair in path.windows(2) {
            map.body_query_with_current_support(pair[0].0, pair[0].1).unwrap()
                .validate_body_motion(pair[0], pair[1], &[]).unwrap();
        }
    }

    #[test]
    fn local_window_must_connect_beyond_a_blocking_wall() {
        let map = scene(Some(0));
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        // 墙位于x=0；走到墙前虽已覆盖六成参考，仍没有连接到窗口交接点。
        let result = local_search::plan(
            &map, &tomo, start, &[[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]],
            &motion(), None, None, &[], 3.0, 0.9, true,
        );
        let failure = result.expect_err("不能把墙前的可行前缀当作完整近场接续");
        assert!(failure.blocked.is_none(), "finite search cannot report an obstacle");
        let reached = failure.closest_pose.expect("closest pose is diagnostic").0;
        assert!(reached[0] > -0.5 && reached[0] < 0.0, "反馈应在墙前而非原起点：{reached:?}");
    }

    #[test]
    fn in_place_turn_on_a_slope_keeps_the_world_ground_normal() {
        let map = scene_with_slope(None, 0.08);
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = map
            .body_pose_on_surface([0.0, 0.2], 0.0, [0.0, 0.2, 0.0])
            .unwrap();
        let reference = [[start.0[0], start.0[1], start.0[2] - 0.4]];
        let route = plan(
            &map,
            &tomo,
            start,
            &reference,
            &motion(),
            &ScanConfig::default(),
            Some(1.2),
            Some((start.0, 0.1)),
            &[],
            false,
            false,
        )
        .unwrap();
        let normal = |q: [f64; 4]| {
            [
                2.0 * (q[0] * q[2] + q[3] * q[1]),
                2.0 * (q[1] * q[2] - q[3] * q[0]),
                1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]),
            ]
        };
        let initial = normal(start.1);
        assert!(initial[0].abs() > 0.03, "输入必须是真实斜坡姿态");
        for pose in route.poses {
            assert_eq!(pose.0, start.0);
            let current = normal(pose.1);
            assert!(
                (0..3).all(|axis| (current[axis] - initial[axis]).abs() < 1e-5),
                "转向不能旋转地面法向"
            );
        }
    }
    fn assert_thirty_three_centimeter_passage() -> (BodyPose, [[f64;3];4]) {
        let map = scene(Some(33));
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-0.8, 0.165, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let reference = [
            [-0.8, 0.165, 0.0],
            [-0.35, 0.15, 0.0],
            [0.4, 0.15, 0.0],
            [0.8, 0.165, 0.0],
        ];
        let route = plan(
            &map,
            &tomo,
            start,
            &reference,
            &motion(),
            &ScanConfig::default(),
            None,
            Some(([0.8, 0.165, 0.4], 0.1)),
            &[],
            false,
            false,
        )
        .unwrap();
        assert!(route.poses.last().unwrap().0[0] > 0.65);
        for pair in route.poses.windows(2) {
            map.body_query_with_current_support(pair[0].0, pair[0].1)
                .unwrap()
                .validate_body_motion(pair[0], pair[1], &[])
                .unwrap();
        }
        (start,reference)
    }
    #[test]
    fn matched_narrow_passage_success() {
        assert_thirty_three_centimeter_passage();
    }
    #[test]
    fn actual_thirty_three_centimeter_passage_survives_coarse_reference() {
        let (start,reference)=assert_thirty_three_centimeter_passage();
        let blocked = scene(Some(30));
        let tomo = Tomogram::from_surface(&blocked).unwrap();
        assert!(plan(
            &blocked,
            &tomo,
            start,
            &reference,
            &motion(),
            &ScanConfig::default(),
            None,
            None,
            &[],
            false,
            false,
        )
        .is_err());
    }
}

#[cfg(all(test, feature="profile"))]
#[path="current_on_diagnostic.rs"]
mod current_on_diagnostic;
