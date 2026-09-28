//! 两条导航链共享的有限速度窗口与制动包络，无运动端或后台任务。
#![deny(unsafe_code)]
#![warn(missing_docs)]
use nuwax_nav_types_messages::{MotionLimits, MotionModel, Point2, Twist2};

/// 连续机体平面位姿，所有坐标保持地图参考。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarPose {
    /// 世界坐标横向位置。
    pub x: f64,
    /// 世界坐标纵向位置。
    pub y: f64,
    /// 机体偏航，弧度。
    pub yaw: f64,
}
/// 通过完整制动尾段检查的速度候选。
#[derive(Debug, Clone)]
pub struct ControlChoice {
    /// 本周期可发送的速度，符合加速度与方向能力。
    pub command: Twist2,
    /// 保守停止位置，供诊断而非到达证明。
    pub stopped_pose: PlanarPose,
    /// 检查的运动样本数量。
    pub samples: usize,
}
fn finite(v: Twist2) -> bool {
    [v.linear.x, v.linear.y, v.angular_z]
        .into_iter()
        .all(f64::is_finite)
}
fn toward(value: f64, target: f64, limit: f64) -> f64 {
    target.clamp(value - limit, value + limit)
}
fn angle(value: f64) -> f64 {
    (value + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}
/// 精确积分一个恒定机体速度段，支持横移；加速段由小段平均速度逼近。
pub fn integrate(pose: PlanarPose, velocity: Twist2, time: f64) -> PlanarPose {
    let next = pose.yaw + velocity.angular_z * time;
    let (dx, dy) = if velocity.angular_z.abs() < 1e-9 {
        (
            (velocity.linear.x * pose.yaw.cos() - velocity.linear.y * pose.yaw.sin()) * time,
            (velocity.linear.x * pose.yaw.sin() + velocity.linear.y * pose.yaw.cos()) * time,
        )
    } else {
        (
            (velocity.linear.x * (next.sin() - pose.yaw.sin())
                + velocity.linear.y * (next.cos() - pose.yaw.cos()))
                / velocity.angular_z,
            (velocity.linear.x * (pose.yaw.cos() - next.cos())
                + velocity.linear.y * (next.sin() - pose.yaw.sin()))
                / velocity.angular_z,
        )
    };
    PlanarPose {
        x: pose.x + dx,
        y: pose.y + dy,
        yaw: angle(next),
    }
}
fn allowed(command: Twist2, limits: &MotionLimits, reverse: bool) -> bool {
    finite(command)
        && command.linear.x <= limits.max_forward_mps + 1e-9
        && command.linear.x >= -if reverse { limits.max_reverse_mps } else { 0.0 } - 1e-9
        && command.linear.y.abs() <= limits.max_lateral_mps + 1e-9
        && command.angular_z.abs() <= limits.max_angular_rps + 1e-9
        && (limits.can_rotate_in_place
            || command.angular_z.abs() <= 1e-9
            || command.linear.x.hypot(command.linear.y) > 1e-8)
        && (limits.minimum_turning_radius_m == 0.0
            || command.angular_z.abs() * limits.minimum_turning_radius_m
                <= command.linear.x.hypot(command.linear.y) + 1e-9)
}

fn canonical_allowed_command(mut command:Twist2,limits:&MotionLimits,reverse:bool)->Twist2 {
    // 仅处理已通过原允许判定的候选，让禁用通道的数值残差在评分与预测前精确归零。
    // 实际已观察速度仍完整进入后续过渡和制动，不能把它当成已停止。
    if limits.max_lateral_mps==0.0 {command.linear.y=0.0;}
    if !reverse || limits.max_reverse_mps==0.0 {command.linear.x=command.linear.x.max(0.0);}
    if limits.minimum_turning_radius_m>0.0 {
        let maximum=command.linear.x.hypot(command.linear.y)/limits.minimum_turning_radius_m;
        command.angular_z=command.angular_z.clamp(-maximum,maximum);
    } else if !limits.can_rotate_in_place && command.linear.x==0.0 && command.linear.y==0.0 {
        command.angular_z=0.0;
    }
    command
}
fn reachable(
    preferred: Twist2,
    current: Twist2,
    limits: &MotionLimits,
    dt: f64,
    reverse: bool,
) -> Twist2 {
    let x = preferred.linear.x.clamp(
        -if reverse { limits.max_reverse_mps } else { 0.0 },
        limits.max_forward_mps,
    );
    let y = preferred
        .linear
        .y
        .clamp(-limits.max_lateral_mps, limits.max_lateral_mps);
    let w = preferred
        .angular_z
        .clamp(-limits.max_angular_rps, limits.max_angular_rps);
    let axis = |now: f64, next: f64, a: f64, b: f64| {
        // 换向先制动至零，不能一次跳过原速度方向。
        let next = if now * next < 0.0 { 0.0 } else { next };
        let rate = if next.abs() > now.abs() { a } else { b };
        toward(now, next, rate * dt)
    };
    // 平移加减速是设备声明的矢量上界；逐轴先截断会把停车变成45度减速，
    // 与真实执行器及完整停车预测的径向制动不一致，且持续浪费可用转向加速度。
    // 全向平移的分量可在连续矢量加速度内过零；强制每个分量先精确为零，
    // 会在带侧向速度时令微小前向分量渐近不归零，永久禁止离墙所需的倒退。
    let x = if limits.model!=MotionModel::Omnidirectional && current.linear.x*x<0.0 {0.0}else{x};
    let y = if limits.model!=MotionModel::Omnidirectional && current.linear.y*y<0.0 {0.0}else{y};
    let dx = x - current.linear.x;
    let dy = y - current.linear.y;
    let rate = if dx * current.linear.x + dy * current.linear.y < 0.0 {
        limits
            .guaranteed_braking_mps2
            .min(limits.linear_acceleration_mps2)
    } else {
        limits.linear_acceleration_mps2
    };
    let ratio = (rate * dt / dx.hypot(dy).max(1e-12)).min(1.0);
    Twist2 {
        linear: Point2::new(current.linear.x + dx * ratio, current.linear.y + dy * ratio),
        angular_z: axis(
            current.angular_z,
            w,
            limits.angular_acceleration_rps2,
            limits.guaranteed_angular_braking_rps2,
        ),
    }
}

/// 从同一真实可达窗口补充本拍平均曲率相符的速度，不替代原候选或运动许可。
fn append_curvature_candidates(
    velocities:&mut Vec<Twist2>,curvature:f64,preferred:Twist2,current:Twist2,
    limits:&MotionLimits,dt:f64,reverse:bool,
) {
    if !curvature.is_finite() || preferred.linear.x<=1e-9 || current.linear.x<0.0
        || preferred.linear.y!=0.0 || current.linear.y!=0.0 || limits.max_lateral_mps!=0.0 {
        return;
    }
    if limits.minimum_turning_radius_m>0.0 && curvature.abs()>1.0/limits.minimum_turning_radius_m {
        return;
    }
    let low=reachable(Twist2{linear:Point2::new(0.0,0.0),angular_z:-limits.max_angular_rps},
        current,limits,dt,reverse);
    let high=reachable(Twist2{linear:Point2::new(limits.max_forward_mps,0.0),angular_z:limits.max_angular_rps},
        current,limits,dt,reverse);
    let mut lower=low.linear.x.max(0.0);let mut upper=high.linear.x;
    if curvature==0.0 {
        if -current.angular_z<low.angular_z || -current.angular_z>high.angular_z {return;}
    } else {
        let a=(current.angular_z+low.angular_z)/curvature-current.linear.x;
        let b=(current.angular_z+high.angular_z)/curvature-current.linear.x;
        lower=lower.max(a.min(b));upper=upper.min(a.max(b));
    }
    if !lower.is_finite() || !upper.is_finite() || lower>upper {return;}
    // 原窗口的中间线速也须具有匹配曲率的角速；最高档被几何拒绝时仍能沿原弧推进。
    let mut speeds=vec![lower,preferred.linear.x.clamp(lower,upper),upper];
    for velocity in velocities.iter() {
        let speed=velocity.linear.x;
        if velocity.linear.y==0.0 && speed>=lower && speed<=upper && !speeds.contains(&speed) {
            speeds.push(speed);
        }
    }
    for speed in speeds {
        // 中点积分的平均角速度与平均线速度保持同一曲率，包含已有真实速度。
        let requested=Twist2{linear:Point2::new(speed,0.0),
            angular_z:curvature*(current.linear.x+speed)-current.angular_z};
        let candidate=reachable(requested,current,limits,dt,reverse);
        if allowed(candidate,limits,reverse) {
            let candidate=canonical_allowed_command(candidate,limits,reverse);
            if !velocities.contains(&candidate) {velocities.push(candidate);}
        }
    }
}

/// 验证响应延迟、一个控制周期和停止尾段。回调须检查相邻样本间完整机体扫掠。
/// 固定检查即时响应及速度过渡两类轨迹；每条至多1024段、20秒，不截短制动尾段。
pub fn check_stopping_trajectory(
    pose: PlanarPose,
    current: Twist2,
    command: Twist2,
    limits: &MotionLimits,
    control_dt: f64,
    spatial_step_m: f64,
    body_radius_m: f64,
    mut clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
) -> Option<ControlChoice> {
    check_stopping_trajectory_with_period_check(pose,current,command,limits,control_dt,
        spatial_step_m,body_radius_m,&mut clear,|_,_,_|true)
}

/// 同时检查细分几何与整拍物理接续；两类有状态检查必须分别保存预测站姿。
#[allow(clippy::too_many_arguments)]
pub fn check_stopping_trajectory_with_period_check(
    pose:PlanarPose,current:Twist2,command:Twist2,limits:&MotionLimits,control_dt:f64,
    spatial_step_m:f64,body_radius_m:f64,
    mut clear:impl FnMut(PlanarPose,PlanarPose,f64)->bool,
    mut period_clear:impl FnMut(PlanarPose,PlanarPose,f64)->bool,
)->Option<ControlChoice> {
    let mut current_stop=None;
    check_stopping_trajectory_cached(pose,current,command,limits,control_dt,spatial_step_m,
        body_radius_m,&mut clear,&mut period_clear,&mut current_stop)
}

#[allow(clippy::too_many_arguments)]
fn check_stopping_trajectory_cached(
    pose:PlanarPose,current:Twist2,command:Twist2,limits:&MotionLimits,control_dt:f64,
    spatial_step_m:f64,body_radius_m:f64,
    mut clear:&mut impl FnMut(PlanarPose,PlanarPose,f64)->bool,
    period_clear:&mut impl FnMut(PlanarPose,PlanarPose,f64)->bool,
    current_stop:&mut Option<Option<ControlChoice>>,
)->Option<ControlChoice> {
    if limits.validate().is_err()
        || !finite(current)
        || !finite(command)
        || ![
            pose.x,
            pose.y,
            pose.yaw,
            control_dt,
            spatial_step_m,
            body_radius_m,
        ]
        .into_iter()
        .all(f64::is_finite)
        || control_dt <= 0.0
        || control_dt > 1.0
        || spatial_step_m <= 0.0
        || body_radius_m < 0.0
    {
        return None;
    }
    // 同一选择内，当前速度的立即停车与候选无关，只在固定几何和检查时刻下验证一次。
    if current_stop.is_none() {
        *current_stop=Some(stopping_from_velocity(pose,current,limits.reaction_time_sec,
            limits,spatial_step_m,body_radius_m,&mut clear));
    }
    let current_choice=current_stop.as_ref()?.as_ref()?;
    // 每个候选都核对实际速度过渡及离散停车。即使同向直线，最后不足一个
    // 控制周期的减速也不能用连续制动距离替代，否则贴近障碍时会漏掉毫米级尾段。
    // 先检查实际整拍过程，可提前拒绝未来足迹已不许可的候选，减少无效长扫掠。
    let mut transition_samples=0;
    for delay in [0.0,limits.reaction_time_sec] {
        transition_samples += stopping_after_velocity_transition(pose,current,command,delay,limits,
            control_dt,spatial_step_m,body_radius_m,&mut clear,period_clear)?;
        if limits.reaction_time_sec == 0.0 {break;}
    }
    // 即时响应边界仍是独立必要条件，不能被实际加速模型替代。
    let mut choice=if command!=Twist2::ZERO {
        stopping_from_velocity(pose,command,control_dt+limits.reaction_time_sec,
            limits,spatial_step_m,body_radius_m,&mut clear)?
    } else {current_choice.clone()};
    choice.command=command;
    choice.samples+=transition_samples;
    Some(choice)
}

/// 一个控制周期的中点积分；细分只增加几何采样，周期末位置与执行器速度回执模型一致。
fn sweep_velocity_transition(
    origin: PlanarPose, before: Twist2, after: Twist2, duration: f64, elapsed: f64,
    spatial_step_m: f64, body_radius_m: f64, samples: &mut usize,
    clear: &mut impl FnMut(PlanarPose,PlanarPose,f64)->bool,
) -> Option<PlanarPose> {
    if duration <= 0.0 {return Some(origin);}
    let rate = before.linear.x.hypot(before.linear.y).max(after.linear.x.hypot(after.linear.y))
        + body_radius_m*before.angular_z.abs().max(after.angular_z.abs());
    let count=(rate*duration/spatial_step_m).ceil().max(1.0)as usize;
    if count>1024 || *samples+count>1024 {return None;}
    let mut previous=origin;
    for index in 1..=count {
        let fraction=index as f64/count as f64;
        let mean=Twist2 {linear:Point2::new(
            before.linear.x+(after.linear.x-before.linear.x)*fraction*0.5,
            before.linear.y+(after.linear.y-before.linear.y)*fraction*0.5),
            angular_z:before.angular_z+(after.angular_z-before.angular_z)*fraction*0.5};
        let time=duration*fraction;
        let next=integrate(origin,mean,time);
        *samples+=1;
        if !clear(previous,next,elapsed+time) {return None;}
        previous=next;
    }
    Some(previous)
}

/// 当前速度到本拍命令的实际加速接续，再完整积分立即/最长响应时间后的停车。
fn stopping_after_velocity_transition(
    pose: PlanarPose, current: Twist2, command: Twist2, delay: f64, limits: &MotionLimits,
    control_dt: f64, spatial_step_m: f64, body_radius_m: f64,
    clear: &mut impl FnMut(PlanarPose,PlanarPose,f64)->bool,
    period_clear: &mut impl FnMut(PlanarPose,PlanarPose,f64)->bool,
) -> Option<usize> {
    let mut velocity=reachable(command,current,limits,control_dt,true);
    let tail=(velocity.linear.x.hypot(velocity.linear.y)/limits.guaranteed_braking_mps2)
        .max(velocity.angular_z.abs()/limits.guaranteed_angular_braking_rps2);
    if !tail.is_finite() || control_dt+delay+tail>20.0 {return None;}
    if !clear(pose,pose,0.0) || !period_clear(pose,pose,0.0) {return None;}
    let mut samples=0;
    let mut here=sweep_velocity_transition(pose,current,velocity,control_dt,0.0,
        spatial_step_m,body_radius_m,&mut samples,clear)?;
    // 细分只用于连续几何；执行器每拍确认的端点从独立物理站姿接续。
    if !period_clear(pose,here,control_dt) {return None;}
    let mut elapsed=control_dt;
    let mut remaining_delay=delay;
    while remaining_delay>0.0 {
        let duration=remaining_delay.min(control_dt);
        let origin=here;
        here=sweep_velocity_transition(origin,velocity,velocity,duration,elapsed,
            spatial_step_m,body_radius_m,&mut samples,clear)?;
        elapsed+=duration;
        if !period_clear(origin,here,elapsed) {return None;}
        remaining_delay=(remaining_delay-duration).max(0.0);
    }
    while velocity.linear.x.hypot(velocity.linear.y)>1e-12 || velocity.angular_z.abs()>1e-12 {
        let speed=velocity.linear.x.hypot(velocity.linear.y);
        let next=if limits.model==MotionModel::Ackermann {
            let remaining=(speed/limits.guaranteed_braking_mps2)
                .max(velocity.angular_z.abs()/limits.guaranteed_angular_braking_rps2);
            let ratio=(1.0-control_dt/remaining.max(1e-12)).max(0.0);
            Twist2{linear:Point2::new(velocity.linear.x*ratio,velocity.linear.y*ratio),angular_z:velocity.angular_z*ratio}
        } else {
            let ratio=(1.0-limits.guaranteed_braking_mps2*control_dt/speed.max(1e-12)).max(0.0);
            Twist2{linear:Point2::new(velocity.linear.x*ratio,velocity.linear.y*ratio),
                angular_z:toward(velocity.angular_z,0.0,limits.guaranteed_angular_braking_rps2*control_dt)}
        };
        // 速度每个控制周期更新，最后一拍也按中点模型积分，不能提前缩短成连续制动距离。
        let origin=here;
        here=sweep_velocity_transition(origin,velocity,next,control_dt,elapsed,
            spatial_step_m,body_radius_m,&mut samples,clear)?;
        elapsed+=control_dt;
        if !period_clear(origin,here,elapsed) {return None;}
        velocity=next;
    }
    Some(samples)
}

/// 候选评分复用同一整拍速度过渡及完整停车积分，几何细分不影响平面末态。
fn stopping_pose_after_command(
    pose:PlanarPose,current:Twist2,command:Twist2,limits:&MotionLimits,control_dt:f64,
)->Option<PlanarPose> {
    let mut stopped=pose;
    let mut track=|_:PlanarPose,to:PlanarPose,_:f64| {stopped=to;true};
    // 每拍只取末端即可得到相同平面结果；实际许可仍按原空间步长检查全部扫掠。
    stopping_after_velocity_transition(pose,current,command,limits.reaction_time_sec,limits,
        control_dt,f64::MAX,0.0,&mut track,&mut |_,_,_|true)?;
    Some(stopped)
}

fn stopping_from_velocity(
    pose: PlanarPose,
    initial: Twist2,
    hold: f64,
    limits: &MotionLimits,
    spatial_step_m: f64,
    body_radius_m: f64,
    mut clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
) -> Option<ControlChoice> {
    let speed = initial.linear.x.hypot(initial.linear.y);
    let angular = initial.angular_z.abs();
    let tail = (speed / limits.guaranteed_braking_mps2)
        .max(angular / limits.guaranteed_angular_braking_rps2);
    let duration = hold + tail;
    if !duration.is_finite() || duration > 20.0 {
        return None;
    }
    let sample_dt = (spatial_step_m / (speed + body_radius_m * angular).max(0.01)).min(0.05);
    if (duration / sample_dt).ceil() > 1024.0 {
        return None;
    }
    let mut here = pose;
    let mut velocity = initial;
    let mut elapsed = 0.0;
    let mut samples = 0;
    if !clear(here, here, 0.0) {
        return None;
    }
    for (braking, time) in [(false, hold), (true, tail)] {
        let steps = (time / sample_dt).ceil() as usize;
        if steps == 0 {
            continue;
        }
        let dt = time / steps as f64;
        for i in 1..=steps {
            let next = if !braking {
                initial
            } else if limits.model == MotionModel::Ackermann {
                let ratio = (1.0 - i as f64 * dt / tail.max(1e-9)).max(0.0);
                Twist2 {
                    linear: Point2::new(initial.linear.x * ratio, initial.linear.y * ratio),
                    angular_z: initial.angular_z * ratio,
                }
            } else {
                let ratio = (1.0
                    - limits.guaranteed_braking_mps2 * dt
                        / velocity.linear.x.hypot(velocity.linear.y).max(1e-12))
                .max(0.0);
                Twist2 {
                    linear: Point2::new(velocity.linear.x * ratio, velocity.linear.y * ratio),
                    angular_z: toward(
                        velocity.angular_z,
                        0.0,
                        limits.guaranteed_angular_braking_rps2 * dt,
                    ),
                }
            };
            let mean = Twist2 {
                linear: Point2::new(
                    (velocity.linear.x + next.linear.x) * 0.5,
                    (velocity.linear.y + next.linear.y) * 0.5,
                ),
                angular_z: (velocity.angular_z + next.angular_z) * 0.5,
            };
            let to = integrate(here, mean, dt);
            elapsed += dt;
            samples += 1;
            if samples > 1024 || !clear(here, to, elapsed) {
                return None;
            }
            here = to;
            velocity = next;
        }
    }
    Some(ControlChoice {
        command: Twist2::ZERO,
        stopped_pose: here,
        samples,
    })
}

/// 在有界速度窗口中选择能完整停车且朝目标推进的候选，至多25个去重组合。
pub fn choose_control(
    pose: PlanarPose,
    current: Twist2,
    preferred: Twist2,
    target: PlanarPose,
    limits: &MotionLimits,
    control_dt: f64,
    spatial_step_m: f64,
    body_radius_m: f64,
    allow_reverse: bool,
    clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
) -> Option<ControlChoice> {
    choose_control_with_candidate_filter(pose,current,preferred,target,limits,control_dt,
        spatial_step_m,body_radius_m,allow_reverse,|_|true,clear)
}

/// 在选择内部排除上层确认无法推进任务的候选，几何与完整制动检查仍统一执行。
/// 同次选择的回调须基于同一环境和检查时刻，当前速度停车段会在候选间复用。
#[allow(clippy::too_many_arguments)]
pub fn choose_control_with_candidate_filter(
    pose: PlanarPose,
    current: Twist2,
    preferred: Twist2,
    target: PlanarPose,
    limits: &MotionLimits,
    control_dt: f64,
    spatial_step_m: f64,
    body_radius_m: f64,
    allow_reverse: bool,
    mut admissible: impl FnMut(Twist2) -> bool,
    mut clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
) -> Option<ControlChoice> {
    choose_control_with_period_check(pose,current,preferred,target,limits,control_dt,
        spatial_step_m,body_radius_m,allow_reverse,&mut admissible,&mut clear,|_,_,_|true)
}

/// 几何细分和整拍物理站姿分别预测，任一拒绝都在候选选择内生效。
#[allow(clippy::too_many_arguments)]
pub fn choose_control_with_period_check(
    pose: PlanarPose,
    current: Twist2,
    preferred: Twist2,
    target: PlanarPose,
    limits: &MotionLimits,
    control_dt: f64,
    spatial_step_m: f64,
    body_radius_m: f64,
    allow_reverse: bool,
    admissible: impl FnMut(Twist2) -> bool,
    clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
    period_clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
) -> Option<ControlChoice> {
    choose_control_ordered(pose,current,preferred,target,limits,control_dt,spatial_step_m,
        body_radius_m,allow_reverse,control_dt,None,|_,_,_|None,admissible,clear,period_clear)
}

/// 路径跟随与停车接近目标分开保留，避免速度收益掩盖偏离，也避免曲率微动压过全面更好的纠偏。
#[derive(Clone, Copy, Debug)]
pub struct TrackingScore {
    /// 预览机体位置及切向相对原路径的误差，单位为米。
    pub path_error_m:f64,
    /// 完整停车末态接近局部目标的代价。
    pub target_cost:f64,
}

/// 用本拍真实速度过渡及随后恒定候选的三个预览样本评价路径跟随。
/// 为原窗口线速与边界补充曲率耦合提案，先保持可达曲率，再用预览评分择优；全部停车许可仍须通过。
/// 评分返回None时保留原位姿目标评分，不授予任何额外运动能力。
#[allow(clippy::too_many_arguments)]
pub fn choose_tracking_control_with_period_check(
    pose:PlanarPose,current:Twist2,preferred:Twist2,target:PlanarPose,limits:&MotionLimits,
    control_dt:f64,spatial_step_m:f64,body_radius_m:f64,allow_reverse:bool,
    tracking_horizon:f64,
    reference_curvature:Option<f64>,
    score:impl FnMut(Twist2,[PlanarPose;3],PlanarPose)->Option<TrackingScore>,
    admissible:impl FnMut(Twist2)->bool,
    clear:impl FnMut(PlanarPose,PlanarPose,f64)->bool,
    period_clear:impl FnMut(PlanarPose,PlanarPose,f64)->bool,
) -> Option<ControlChoice> {
    choose_control_ordered(pose,current,preferred,target,limits,control_dt,spatial_step_m,
        body_radius_m,allow_reverse,tracking_horizon,reference_curvature,score,admissible,clear,period_clear)
}

#[allow(clippy::too_many_arguments)]
fn choose_control_ordered(
    pose:PlanarPose,current:Twist2,preferred:Twist2,target:PlanarPose,limits:&MotionLimits,
    control_dt:f64,spatial_step_m:f64,body_radius_m:f64,allow_reverse:bool,tracking_horizon:f64,
    reference_curvature:Option<f64>,
    mut tracking_score:impl FnMut(Twist2,[PlanarPose;3],PlanarPose)->Option<TrackingScore>,
    mut admissible:impl FnMut(Twist2)->bool,
    mut clear:impl FnMut(PlanarPose,PlanarPose,f64)->bool,
    mut period_clear:impl FnMut(PlanarPose,PlanarPose,f64)->bool,
) -> Option<ControlChoice> {
    if limits.validate().is_err()
        || !finite(current)
        || !finite(preferred)
        || ![target.x, target.y, target.yaw,control_dt,tracking_horizon]
            .into_iter()
            .all(f64::is_finite)
        || control_dt<=0.0 || tracking_horizon<control_dt || tracking_horizon>20.0
        || reference_curvature.is_some_and(|value|!value.is_finite())
    {
        return None;
    }
    if preferred == Twist2::ZERO {
        return check_stopping_trajectory_with_period_check(
            pose,
            current,
            Twist2::ZERO,
            limits,
            control_dt,
            spatial_step_m,
            body_radius_m,
            clear,
            period_clear,
        );
    }
    let mut velocities = Vec::new();
    let desired=reachable(preferred,current,limits,control_dt,allow_reverse);
    for scale in [1.0, 0.6, 0.3, 0.0] {
        for w in [
            desired.angular_z,
            0.0,
            current.angular_z - limits.angular_acceleration_rps2 * control_dt,
            current.angular_z + limits.angular_acceleration_rps2 * control_dt,
            current.angular_z,
        ] {
            let mut candidate = reachable(
                Twist2 {
                    // 在实际可达窗口内采样；先缩放远处参考再截断会把所有非零候选压成同一个速度。
                    linear: Point2::new(current.linear.x+(desired.linear.x-current.linear.x)*scale,
                        current.linear.y+(desired.linear.y-current.linear.y)*scale),
                    angular_z: w,
                },
                current,
                limits,
                control_dt,
                allow_reverse,
            );
            if limits.minimum_turning_radius_m > 0.0 {
                let limit =
                    candidate.linear.x.hypot(candidate.linear.y) / limits.minimum_turning_radius_m;
                candidate.angular_z = candidate.angular_z.clamp(-limit, limit);
            }
            if allowed(candidate, limits, allow_reverse) {
                let candidate=canonical_allowed_command(candidate,limits,allow_reverse);
                if !velocities.iter().any(|old: &Twist2| old == &candidate) {velocities.push(candidate);}
            }
        }
    }
    let mut braking=reachable(Twist2::ZERO,current,limits,control_dt,allow_reverse);
    if limits.minimum_turning_radius_m>0.0 {
        let limit=braking.linear.x.hypot(braking.linear.y)/limits.minimum_turning_radius_m;
        braking.angular_z=braking.angular_z.clamp(-limit,limit);
    }
    if allowed(braking,limits,allow_reverse) {
        let braking=canonical_allowed_command(braking,limits,allow_reverse);
        if !velocities.contains(&braking) {velocities.push(braking);}
    }
    if let Some(curvature)=reference_curvature {
        append_curvature_candidates(&mut velocities,curvature,preferred,current,limits,control_dt,allow_reverse);
        if preferred.linear.x>1e-9 {
            append_curvature_candidates(&mut velocities,preferred.angular_z/preferred.linear.x,
                preferred,current,limits,control_dt,allow_reverse);
        }
    }
    let mut ranked:Vec<_>=velocities.into_iter().filter_map(|velocity| {
        let stopped=stopping_pose_after_command(pose,current,velocity,limits,control_dt)?;
        let applied=reachable(velocity,current,limits,control_dt,true);
        let mean=Twist2 {linear:Point2::new(
            current.linear.x+(applied.linear.x-current.linear.x)*0.5,
            current.linear.y+(applied.linear.y-current.linear.y)*0.5),
            angular_z:current.angular_z+(applied.angular_z-current.angular_z)*0.5};
        let first=integrate(pose,mean,control_dt);
        let preview=[first,integrate(first,applied,(tracking_horizon-control_dt)*0.5),
            integrate(first,applied,tracking_horizon-control_dt)];
        let tracking=tracking_score(velocity,preview,stopped);
        if tracking.is_some_and(|value|!value.path_error_m.is_finite()||!value.target_cost.is_finite()) {return None;}
        let score=tracking.map(|value|value.path_error_m+value.target_cost).unwrap_or_else(||(stopped.x-target.x).hypot(stopped.y-target.y)
            +0.1*angle(stopped.yaw-target.yaw).abs()
            +0.005*(velocity.angular_z-preferred.angular_z).abs());
        let curvature_error=if reference_curvature.is_some() && preferred.linear.x>1e-9 {
            let curvature=preferred.angular_z/preferred.linear.x;
            let linear=curvature*(current.linear.x+velocity.linear.x);
            let angular=current.angular_z+velocity.angular_z;
            let residual=(linear-angular).abs();
            // 同一解析曲率的末位舍入不得改变档位排序；该界只来自浮点运算精度。
            if residual<=16.0*f64::EPSILON*(linear.abs()+angular.abs()) {0.0}
            else {residual/(1.0+curvature*curvature).sqrt()}
        }else {0.0};
        // 静止零命令虽然在任意曲率直线上，却不能压过原准入已证明有用的恢复转向。
        let stationary_idle=reference_curvature.is_some() && preferred.linear.x>1e-9
            && current==Twist2::ZERO && velocity==Twist2::ZERO;
        score.is_finite().then_some((stationary_idle,curvature_error,score,velocity,stopped,tracking))
    }).collect();
    // 停车距离和停车转角都进入目标误差，不能靠短时更接近目标而一直保持过高速度。
    // 排序只用已声明的动力学；首个候选仍须完成原几何、观察和整拍物理检查。
    ranked.sort_by(|left,right|left.0.cmp(&right.0).then_with(||left.1.total_cmp(&right.1)).then_with(||left.2.total_cmp(&right.2)));
    let mut best=None;let mut best_tracking=None;let mut current_stop=None;
    let mut tested=vec![false;ranked.len()];
    for (index,&(_,_,_,velocity,stopped,tracking)) in ranked.iter().enumerate() {
        tested[index]=true;
        if !admissible(velocity) {continue;}
        if let Some(mut choice)=check_stopping_trajectory_cached(pose,current,velocity,limits,control_dt,
            spatial_step_m,body_radius_m,&mut clear,&mut period_clear,&mut current_stop) {
            choice.stopped_pose=stopped;
            best=Some(choice);best_tracking=tracking;break;
        }
        if current_stop.as_ref().is_some_and(|result|result.is_none()) {break;}
    }
    // 曲率首选若只能微动，不得饿死路径跟随和停车接近目标同时更好的原候选。
    // 两项必须都不劣且至少一项改善；速度更快但更偏离路径的候选不能据此替换。
    if best.is_some() && best_tracking.is_some() {
        for (index,&(stationary_idle,_,_,velocity,stopped,tracking)) in ranked.iter().enumerate() {
            if tested[index] || stationary_idle {continue;}
            let (Some(candidate),Some(previous))=(tracking,best_tracking)else{continue;};
            if candidate.path_error_m>previous.path_error_m || candidate.target_cost>previous.target_cost
                || candidate.path_error_m==previous.path_error_m && candidate.target_cost==previous.target_cost {continue;}
            if !admissible(velocity) {continue;}
            if let Some(mut choice)=check_stopping_trajectory_cached(pose,current,velocity,limits,control_dt,
                spatial_step_m,body_radius_m,&mut clear,&mut period_clear,&mut current_stop) {
            choice.stopped_pose=stopped;best=Some(choice);best_tracking=Some(candidate);
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    fn limits() -> MotionLimits {
        MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 2.0,
            max_reverse_mps: 1.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 2.0,
            linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0,
            angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0,
            reaction_time_sec: 0.2,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        }
    }
    fn pose() -> PlanarPose {
        PlanarPose {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        }
    }
    fn twist(x: f64, y: f64, w: f64) -> Twist2 {
        Twist2 {
            linear: Point2::new(x, y),
            angular_z: w,
        }
    }
    #[test]
    fn stopping_check_order_keeps_all_boundaries_and_short_circuits_failed_periods() {
        // 保留旧检查次序作兼容性参照；每个轨迹仍调用同一真实动力学积分。
        fn prior_order(current:Twist2,command:Twist2,limits:&MotionLimits,
            clear:&mut impl FnMut(PlanarPose,PlanarPose,f64)->bool,
            period:&mut impl FnMut(PlanarPose,PlanarPose,f64)->bool)->Option<ControlChoice> {
            let stopped=stopping_from_velocity(pose(),current,limits.reaction_time_sec,limits,0.02,0.3,&mut *clear)?;
            let mut result=if command==Twist2::ZERO {stopped}else{
                stopping_from_velocity(pose(),command,0.1+limits.reaction_time_sec,limits,0.02,0.3,&mut *clear)?};
            result.command=command;
            for delay in [0.0,limits.reaction_time_sec] {
                result.samples+=stopping_after_velocity_transition(pose(),current,command,delay,
                    limits,0.1,0.02,0.3,clear,period)?;
                if limits.reaction_time_sec==0.0 {break;}
            }
            Some(result)
        }
        for model in [MotionModel::Differential,MotionModel::Omnidirectional,MotionModel::Ackermann] {
            for delay in [0.0,0.2] {
                for mode in 0..3 {
                    let mut limits=limits();limits.model=model;limits.reaction_time_sec=delay;
                    if model==MotionModel::Omnidirectional {limits.max_lateral_mps=1.0;}
                    if model==MotionModel::Ackermann {limits.minimum_turning_radius_m=0.5;limits.can_rotate_in_place=false;}
                    let (current,command)=if mode==2 {(twist(0.4,0.0,0.0),twist(0.5,0.0,0.0))}
                        else if model==MotionModel::Omnidirectional {(twist(0.4,0.1,0.2),twist(0.45,0.15,0.3))}
                        else {(twist(0.4,0.0,0.2),twist(0.5,0.0,0.3))};
                    let run=|prior| {
                        let trace=std::cell::RefCell::new(Vec::new());
                        let record=|basis,from:PlanarPose,to:PlanarPose,elapsed:f64| {
                            trace.borrow_mut().push([basis,from.x.to_bits(),from.y.to_bits(),from.yaw.to_bits(),
                                to.x.to_bits(),to.y.to_bits(),to.yaw.to_bits(),elapsed.to_bits()]);
                        };
                        let mut clear=|from,to:PlanarPose,time| {
                            record(0,from,to,time);
                            // 该障碍位于真实加速停车与即时响应停车的末端之间。
                            mode!=2 || to.x<=0.5*(0.1+delay)+0.125-0.0025
                        };
                        let mut period=|from,to,time| {record(1,from,to,time);mode!=1 || time==0.0};
                        let result=if prior {prior_order(current,command,&limits,&mut clear,&mut period)}else{
                            check_stopping_trajectory_with_period_check(pose(),current,command,&limits,
                                0.1,0.02,0.3,&mut clear,&mut period)};
                        let mut trace=trace.into_inner();trace.sort();
                        (result.map(|r|(r.command,r.stopped_pose,r.samples)),trace)
                    };
                    let prior=run(true);let reordered=run(false);
                    assert_eq!(prior.0,reordered.0,"检查次序不能改变完整许可或停止状态：{model:?}/{delay}/{mode}");
                    if mode==0 {
                        assert!(reordered.0.is_some());
                        assert_eq!(prior.1,reordered.1,"成功候选必须保留每个几何与整拍检查样本");
                    }else if mode==1 {
                        assert!(reordered.0.is_none());
                        assert!(reordered.1.len()<prior.1.len(),"实际过程已拒绝后应停止无用即时边界查询");
                    }else {
                        assert!(reordered.0.is_none(),"实际过程通过不能省略即时响应边界");
                    }
                }
            }
        }
    }

    #[test]
    fn disabled_command_axes_are_exact_while_observed_motion_is_preserved() {
        for lateral_disabled in [true,false] {
            let mut limits=limits();
            if !lateral_disabled {limits.model=MotionModel::Omnidirectional;limits.max_lateral_mps=1.0;}
            let current=if lateral_disabled {twist(0.3,1e-14,0.0)}else{twist(-1e-14,0.3,0.0)};
            let preferred=if lateral_disabled {twist(0.4,0.0,0.0)}else{twist(0.0,0.4,0.0)};
            let target=if lateral_disabled {PlanarPose{x:2.0,y:0.0,yaw:0.0}}else{PlanarPose{x:0.0,y:2.0,yaw:0.0}};
            let mut observed_axis=false;
            let choice=choose_control_with_period_check(pose(),current,preferred,target,&limits,
                0.1,0.01,0.3,false,|_|true,|_,_,_|true,|_,to,_| {
                    observed_axis|=if lateral_disabled {to.y>0.0}else{to.x<0.0};true
                }).unwrap();
            if lateral_disabled {assert_eq!(choice.command.linear.y,0.0);assert!(choice.command.linear.x>0.0);}
            else {assert_eq!(choice.command.linear.x,0.0);assert!(choice.command.linear.y>0.0);}
            assert!(observed_axis,"原始实际速度残差必须继续进入整拍预测");
            let unsupported=if lateral_disabled {twist(0.3,0.3,0.0)}else{twist(-0.3,0.3,0.0)};
            assert!(choose_control(pose(),unsupported,preferred,target,&limits,0.1,0.01,0.3,false,
                |_,_,_|true).is_none(),"不允许的实际速度不能靠候选归零制造可达命令");
        }
    }

    #[test]
    fn tracking_preview_uses_the_actual_first_period_and_keeps_hard_permissions() {
        let current=twist(0.3,0.0,0.4);let preferred=twist(0.4,0.0,0.2);
        let target=PlanarPose{x:2.0,y:0.0,yaw:0.0};let limits=limits();
        let checked=std::cell::Cell::new(0);
        let choice=choose_tracking_control_with_period_check(pose(),current,preferred,target,&limits,
            0.05,0.025,0.3,false,0.3,Some(1.0),
            |candidate,preview,_| {
                let applied=reachable(candidate,current,&limits,0.05,true);
                let mean=twist((current.linear.x+applied.linear.x)*0.5,0.0,
                    (current.angular_z+applied.angular_z)*0.5);
                let expected=integrate(pose(),mean,0.05);
                assert!((preview[0].x-expected.x).abs()<1e-12);
                assert!((preview[0].y-expected.y).abs()<1e-12);
                assert!((preview[0].yaw-expected.yaw).abs()<1e-12);
                checked.set(checked.get()+1);
                Some(TrackingScore{path_error_m:(candidate.angular_z-0.3).abs(),target_cost:0.0})
            },|_|true,|_,_,_|true,|_,_,_|true).unwrap();
        assert!(checked.get()>1);
        assert!((choice.command.angular_z-0.3).abs()<1e-9);
        for reject_geometry in [false,true] {
            assert!(choose_tracking_control_with_period_check(pose(),current,preferred,target,&limits,
                0.05,0.025,0.3,false,0.3,Some(1.0),|_,_,_|Some(TrackingScore{path_error_m:0.0,target_cost:-1e9}),|_|true,
                |_,_,_|!reject_geometry,|_,_,_|reject_geometry).is_none(),
                "任意低评分也不能绕过几何或整拍许可");
        }
        let ordinary=choose_control_with_period_check(pose(),current,preferred,target,&limits,
            0.05,0.025,0.3,false,|_|true,|_,_,_|true,|_,_,_|true).unwrap();
        let fallback=choose_tracking_control_with_period_check(pose(),current,preferred,target,&limits,
            0.05,0.025,0.3,false,0.3,None,|_,_,_|None,|_|true,|_,_,_|true,|_,_,_|true).unwrap();
        assert_eq!(ordinary.command,fallback.command,"缺少跟随参考时保留原完整评分");
    }

    #[test]
    fn curvature_candidates_match_the_actual_period_and_preserve_the_original_window() {
        let mut limits=limits();limits.max_forward_mps=1.0;limits.max_reverse_mps=0.0;
        limits.max_angular_rps=0.8;limits.reaction_time_sec=0.05;
        let cases=[
            (Twist2::ZERO,twist(1.0,0.0,0.3422487129396208),0.3422487129396208),
            (twist(0.10395200376906219,0.0,0.5007321947613922),
                twist(0.06651956194942318,0.0,0.27727242705926103),
                0.27727242705926103/0.06651956194942318)];
        for (current,preferred,curvature) in cases {
            let original=vec![reachable(preferred,current,&limits,0.05,false),
                reachable(Twist2::ZERO,current,&limits,0.05,false)];
            let mut candidates=original.clone();
            append_curvature_candidates(&mut candidates,curvature,preferred,current,&limits,0.05,false);
            assert_eq!(&candidates[..original.len()],original.as_slice(),"原后备候选不得被替换");
            assert!(candidates.len()>original.len(),"冻结输入应能生成原窗口遗漏的曲率速度");
            for candidate in &candidates[original.len()..] {
                assert!(allowed(*candidate,&limits,false));
                let actual=reachable(*candidate,current,&limits,0.05,false);
                assert!((actual.linear.x-candidate.linear.x).abs()<1e-12);
                assert!((actual.angular_z-candidate.angular_z).abs()<1e-12);
                assert!((current.angular_z+actual.angular_z-curvature*(current.linear.x+actual.linear.x)).abs()<1e-12,
                    "减角速时也必须使用当前与下一实际速度均值，不能只比较命令末态w/v");
            }
            if current==Twist2::ZERO {
                assert!(candidates.iter().any(|c|(c.linear.x-0.05).abs()<1e-12
                    &&(c.angular_z-0.01711243564698104).abs()<1e-12),
                    "9600首拍原弧曲率必须真实进入候选");
            }
        }
        let curvature=0.3422487129396208;
        let current=twist(0.6,0.0,curvature*0.6);
        let mut intermediate=vec![twist(0.63,0.0,current.angular_z),twist(0.615,0.0,current.angular_z)];
        append_curvature_candidates(&mut intermediate,curvature,twist(1.0,0.0,curvature),
            current,&limits,0.05,false);
        for speed in [0.63,0.615] {
            assert!(intermediate.iter().any(|candidate|candidate.linear.x==speed &&
                (candidate.angular_z-curvature*(current.linear.x+speed)+current.angular_z).abs()<1e-12),
                "最高档真实扫掠拒绝后，中间档必须仍有沿原弧的可达提案");
        }
        let mut impossible=Vec::new();
        append_curvature_candidates(&mut impossible,-0.5,twist(0.4,0.0,-0.2),
            twist(0.3,0.0,0.4),&limits,0.05,false);
        assert!(impossible.is_empty(),"不能瞬时跳过尚未刹完的角速度方向");
        limits.model=MotionModel::Ackermann;limits.minimum_turning_radius_m=0.5;limits.can_rotate_in_place=false;
        append_curvature_candidates(&mut impossible,3.0,twist(0.2,0.0,0.6),
            Twist2::ZERO,&limits,0.05,false);
        assert!(impossible.is_empty(),"曲率提案不能新增超出声明最小半径的能力");
    }
    #[test]
    fn feasible_curvature_reduction_precedes_a_faster_departure_from_the_path() {
        let mut limits=limits();limits.max_forward_mps=1.0;limits.max_reverse_mps=0.0;
        limits.max_angular_rps=0.8;limits.reaction_time_sec=0.05;
        let curvature=0.3422487129396208;
        let current=twist(0.23,0.0,curvature*0.23);
        let preferred=twist(1.0,0.0,curvature);
        let choice=choose_tracking_control_with_period_check(pose(),current,preferred,
            PlanarPose{x:1.0,y:0.0,yaw:0.0},&limits,0.05,0.025,0.34,false,0.25,
            Some(curvature),|candidate,_,_|Some(TrackingScore{path_error_m:(candidate.angular_z-curvature*candidate.linear.x).abs(),target_cost:-candidate.linear.x}),
            |candidate| (candidate.linear.x-0.18).abs()<1e-12
                || ((candidate.linear.x-0.23).abs()<1e-12 && candidate.angular_z==0.0),
            |_,_,_|true,|_,_,_|true).expect("原窗口内存在真实许可的保形减速");
        assert!((choice.command.linear.x-0.18).abs()<1e-12);
        assert!((choice.command.angular_z-curvature*0.18).abs()<1e-12,
            "不能为了稍多前进距离丢弃仍然许可的曲率候选");
    }
    #[test]
    fn fully_better_safe_tracking_can_escape_an_exact_curvature_creep() {
        let mut limits=limits();limits.max_forward_mps=1.0;limits.max_reverse_mps=0.0;
        limits.max_angular_rps=0.8;limits.reaction_time_sec=0.05;
        let current=twist(1.19952411212823e-5,0.0,1.251979718304631e-5);
        let preferred=twist(0.41648943448239484,0.0,0.43470266214651804);
        for boundary in 0..4 {
            let choice=choose_tracking_control_with_period_check(pose(),current,preferred,
                PlanarPose{x:1.0,y:0.0,yaw:0.0},&limits,0.05,0.025,0.34,false,0.25,
                Some(0.8),|candidate,_,_| {
                    let advancing=candidate.linear.x>0.04;
                    Some(if advancing {TrackingScore {
                        path_error_m:if boundary==1 {0.02}else{0.009765279936820921},
                        target_cost:if boundary==2 {0.3}else{0.27438112194461384},
                    }}else{TrackingScore {path_error_m:0.010809763672868683,target_cost:0.2798315901192446}})
                },|candidate| candidate.linear.x>0.04 && candidate.angular_z>0.09
                    || (candidate.linear.x-current.linear.x).abs()<1e-12
                        && (candidate.angular_z-current.angular_z).abs()<1e-12,
                |_,to,_|boundary!=3 || to.x<0.001,|_,_,_|true).unwrap();
            if boundary==0 {
                assert!(choice.command.linear.x>0.04 && choice.command.angular_z>0.09,
                    "跟随误差和停车目标同时改善的真实许可纠偏不能被微动饿死");
            }else{
                assert!(choice.command.linear.x<0.001,
                    "任一评分变差或完整几何拒绝时，都必须保留原许可首选");
            }
        }
    }

    #[test]
    fn curvature_order_preserves_restoring_turns_and_equivalent_speed_choices() {
        let mut limits=limits();limits.max_forward_mps=1.0;limits.max_reverse_mps=0.0;
        limits.max_angular_rps=0.8;
        for curvature in [0.0,1e-12,0.3422487129396208] {
            let preferred=twist(1.0,0.0,curvature);
            let select=|current,restore:bool| choose_tracking_control_with_period_check(pose(),current,preferred,
                PlanarPose{x:1.0,y:0.0,yaw:0.0},&limits,0.05,0.025,0.34,false,0.25,
                Some(curvature),|candidate,_,_|Some(TrackingScore{path_error_m:0.0,target_cost:-candidate.linear.x}),
                |candidate|!restore || candidate.linear.x==0.0,|_,_,_|true,|_,_,_|true).unwrap();
            let choice=select(Twist2::ZERO,true);
            assert!(choice.command.angular_z!=0.0,"零速许可不得饿死有用恢复转向");
            for speed in [0.0,1e-8,0.23] {
                let choice=select(twist(speed,0.0,speed*curvature),false);
                assert!((choice.command.linear.x-speed-0.05).abs()<1e-12,
                    "同一数学曲率的末位残差不能抢占原速度评分");
            }
        }
    }
    #[test]
    fn complete_stopping_distance_includes_latency_control_and_braking() {
        let v = twist(1.0, 0.0, 0.0);
        let result =
            check_stopping_trajectory(pose(), v, v, &limits(), 0.1, 0.01, 0.3, |_, _, _| true)
                .unwrap();
        assert!((result.stopped_pose.x - 0.8).abs() < 1e-10);
        assert!(
            check_stopping_trajectory(pose(), v, v, &limits(), 0.1, 0.01, 0.3, |_, to, _| to.x
                < 0.6)
            .is_none(),
            "短时前进段安全，但制动尾段会碰墙"
        );
    }
    #[test]
    fn omnidirectional_braking_uses_vector_magnitude_not_two_independent_brakes() {
        let mut limits = limits();
        limits.model = MotionModel::Omnidirectional;
        limits.max_lateral_mps = 2.0;
        let v = twist(0.6, 0.8, 0.0);
        let result =
            check_stopping_trajectory(pose(), v, v, &limits, 0.1, 0.01, 0.3, |_, _, _| true)
                .unwrap();
        assert!((result.stopped_pose.x.hypot(result.stopped_pose.y) - 0.8).abs() < 1e-10);
    }
    #[test]
    fn sampled_acceleration_and_reverse_permission_hold() {
        let target = PlanarPose {
            x: 10.0,
            y: 0.0,
            yaw: 0.0,
        };
        let result = choose_control(
            pose(),
            Twist2::ZERO,
            twist(2.0, 0.0, 0.0),
            target,
            &limits(),
            0.05,
            0.05,
            0.3,
            false,
            |_, _, _| true,
        )
        .unwrap();
        assert!(result.command.linear.x > 0.0 && result.command.linear.x <= 0.05 + 1e-10);
        let backward = PlanarPose { x: -10.0, ..target };
        let blocked = choose_control(
            pose(),
            Twist2::ZERO,
            twist(-1.0, 0.0, 0.0),
            backward,
            &limits(),
            0.05,
            0.05,
            0.3,
            false,
            |_, _, _| true,
        )
        .unwrap();
        assert!(blocked.command.linear.x >= 0.0);
        let allowed = choose_control(
            pose(),
            Twist2::ZERO,
            twist(-1.0, 0.0, 0.0),
            backward,
            &limits(),
            0.05,
            0.05,
            0.3,
            true,
            |_, _, _| true,
        )
        .unwrap();
        assert!(allowed.command.linear.x < 0.0);
    }
    #[test]
    fn unknown_space_and_unbounded_tail_fail_closed() {
        assert!(choose_control(
            pose(),
            Twist2::ZERO,
            twist(1.0, 0.0, 0.0),
            PlanarPose { x: 1.0, ..pose() },
            &limits(),
            0.05,
            0.05,
            0.3,
            false,
            |_, _, _| false
        )
        .is_none());
        let mut low = limits();
        low.guaranteed_braking_mps2 = 1e-6;
        assert!(check_stopping_trajectory(
            pose(),
            twist(1.0, 0.0, 0.0),
            Twist2::ZERO,
            &low,
            0.05,
            0.05,
            0.3,
            |_, _, _| true
        )
        .is_none());
    }
    #[test]
    fn straight_motion_checks_the_last_discrete_braking_period() {
        let mut limits=limits();limits.reaction_time_sec=0.0;
        let current=twist(0.15,0.0,0.0);
        let clear=|_:PlanarPose,to:PlanarPose,_:f64|to.x<0.027;
        assert!(stopping_from_velocity(pose(),current,0.1,&limits,0.01,0.3,clear).is_some());
        // 本拍前进15毫米，再以0.15→0.05→0离散减速，真实终点27.5毫米。
        assert!(check_stopping_trajectory(pose(),current,current,&limits,0.1,0.01,0.3,clear).is_none());
    }

    #[test]
    fn velocity_samples_cover_small_progress_inside_the_reachable_window() {
        let mut limits=limits();limits.reaction_time_sec=0.05;
        let choice=choose_control(pose(),Twist2::ZERO,twist(1.0,0.0,0.0),
            PlanarPose{x:1.0,y:0.0,yaw:0.0},&limits,0.05,0.001,0.3,false,
            |_,to,_|to.x<0.002).unwrap();
        assert!(choice.command.linear.x>0.0 && choice.command.linear.x<0.03,
            "毫米级起步空间应采用窗口内的小速度，不能只有被截成0.05的候选和零速度: {:?}",choice.command);
    }

    #[test]
    fn acceleration_offset_and_braking_tail_are_checked_before_selecting_velocity() {
        let mut limits=limits();limits.model=MotionModel::Omnidirectional;limits.max_lateral_mps=1.0;
        limits.reaction_time_sec=0.05;
        let current=twist(0.5,0.0,0.0);let command=twist(0.5,0.1,0.0);
        // 小障碍位于“加速一个周期再制动”的实际轨迹上，两条瞬时速度边界都绕开它。
        let clear=|from:PlanarPose,to:PlanarPose,_:f64| {
            let dx=to.x-from.x;let dy=to.y-from.y;
            let t=if dx*dx+dy*dy>0.0 {((0.06-from.x)*dx+(0.007-from.y)*dy)/(dx*dx+dy*dy)}else{0.0}.clamp(0.0,1.0);
            (from.x+t*dx-0.06).hypot(from.y+t*dy-0.007)>0.001
        };
        assert!(stopping_from_velocity(pose(),current,0.05,&limits,0.01,0.3,clear).is_some());
        assert!(stopping_from_velocity(pose(),command,0.15,&limits,0.01,0.3,clear).is_some());
        assert!(check_stopping_trajectory(pose(),current,command,&limits,0.1,0.01,0.3,clear).is_none(),
            "边界通过不能替代加速接续后的完整停车");
        let choice=choose_control_with_period_check(pose(),current,command,PlanarPose{x:5.0,y:1.0,yaw:0.0},
            &limits,0.1,0.01,0.3,true,|_|true,clear,clear).expect("应在选择时改用仍能前进的速度");
        assert_ne!(choice.command,command);
        assert!(choice.command.linear.x>0.0);
        // 独立执行器逐拍施加1米/秒²矢量加减速，直接积分命令，检查真实停车全过程。
        let mut here=pose();let mut actual=current;
        for tick in 0..20 {
            let target=if tick==0 {choice.command}else{Twist2::ZERO};
            let delta=Point2::new(target.linear.x-actual.linear.x,target.linear.y-actual.linear.y);
            let ratio=(0.1/delta.x.hypot(delta.y).max(1e-12)).min(1.0);
            let next=twist(actual.linear.x+delta.x*ratio,actual.linear.y+delta.y*ratio,
                toward(actual.angular_z,target.angular_z,0.2));
            let mean=twist((actual.linear.x+next.linear.x)*0.5,(actual.linear.y+next.linear.y)*0.5,
                (actual.angular_z+next.angular_z)*0.5);
            let to=integrate(here,mean,0.1);
            assert!(clear(here,to,0.0),"已选候选的实际停车不能穿过障碍");
            here=to;actual=next;
        }
    }

    #[test]
    fn translation_and_rotation_start_braking_before_the_target() {
        let mut limits=limits();limits.model=MotionModel::Omnidirectional;
        limits.max_forward_mps=1.0;limits.max_reverse_mps=1.0;limits.max_lateral_mps=1.0;
        limits.max_angular_rps=0.8;limits.reaction_time_sec=0.05;
        let dt=0.05;
        for sign in [-1.0,1.0] {
            for rotation in [false,true] {
                let current=if rotation {twist(0.0,0.0,sign*0.8)}else{twist(sign,0.0,0.0)};
                let target=if rotation {PlanarPose{x:0.0,y:0.0,yaw:sign*0.21}}
                    else {PlanarPose{x:sign*0.55,y:0.0,yaw:0.0}};
                let preferred=if rotation {twist(0.0,0.0,sign*0.21)}else{twist(sign*0.825,0.0,0.0)};
                let choice=choose_control(pose(),current,preferred,target,&limits,dt,0.025,0.3,true,
                    |_,_,_|true).unwrap();
                let mut velocity=current;let mut actual=pose();
                // 独立执行一次所选命令、一个已声明响应周期，再逐拍制动；只用标定更新速度。
                for tick in 0..100 {
                    let command=if tick<2 {choice.command}else{Twist2::ZERO};
                    let next=twist(
                        velocity.linear.x+(command.linear.x-velocity.linear.x).clamp(-dt,dt),0.0,
                        velocity.angular_z+(command.angular_z-velocity.angular_z).clamp(-2.0*dt,2.0*dt));
                    actual.x+=(velocity.linear.x+next.linear.x)*0.5*dt;
                    actual.yaw+=(velocity.angular_z+next.angular_z)*0.5*dt;
                    velocity=next;
                    if tick>=2 && velocity.linear.x.abs()<1e-12 && velocity.angular_z.abs()<1e-12 {break;}
                }
                let (stopped,requested)=if rotation {(actual.yaw,target.yaw)}else{(actual.x,target.x)};
                assert!(sign*(stopped-requested)<=1e-9,
                    "不能为改善0.2秒后的误差而保持过高速度：转向={rotation}，命令={:?}，停点={actual:?}",choice.command);
            }
        }
        let cruise=choose_control(pose(),twist(1.0,0.0,0.0),twist(1.0,0.0,0.0),
            PlanarPose{x:0.75,y:0.0,yaw:0.0},&limits,dt,0.025,0.3,true,|_,_,_|true).unwrap();
        assert_eq!(cruise.command,twist(1.0,0.0,0.0),"前方有完整停车距离的直段继续巡航");
    }

    #[test]
    fn geometry_subdivision_cannot_reset_the_control_period_height_reference() {
        let mut limits=limits();limits.model=MotionModel::Omnidirectional;
        limits.max_lateral_mps=1.0;limits.reaction_time_sec=0.05;
        let current=twist(0.75,0.0,0.0);let command=twist(0.70,0.0,0.0);
        // 实录反例中的中间高度带让细分预测降低11毫米；整拍终点没有这个站姿变化。
        // 两个检查各自持有参考高度，不能借细分后的较低机体放行实际整拍。
        let height_band=|| {
            let mut height=0.011;
            move |_:PlanarPose,to:PlanarPose,elapsed:f64| {
                if elapsed==0.0 {height=0.011;}
                if (0.01..=0.025).contains(&to.x) {height=0.0;}
                to.x<0.03 || height==0.0
            }
        };
        for spatial_step in [0.025,0.0125] {
            assert!(check_stopping_trajectory(pose(),current,command,&limits,0.05,
                spatial_step,0.3,height_band()).is_some(),"仅细分检查会借中间高度带取得许可");
            assert!(check_stopping_trajectory_with_period_check(pose(),current,command,&limits,
                0.05,spatial_step,0.3,height_band(),height_band()).is_none(),
                "改变几何采样密度不能抹去整拍真实接续的拒绝");
        }
    }

    #[test]
    fn ackermann_does_not_invent_rotation_in_place() {
        let mut limits = limits();
        limits.model = MotionModel::Ackermann;
        limits.can_rotate_in_place = false;
        limits.minimum_turning_radius_m = 1.0;
        let result = choose_control(
            pose(),
            Twist2::ZERO,
            twist(0.0, 0.0, 1.0),
            PlanarPose { yaw: 1.0, ..pose() },
            &limits,
            0.05,
            0.05,
            0.3,
            false,
            |_, _, _| true,
        )
        .unwrap();
        assert_eq!(result.command, Twist2::ZERO);
    }
}
