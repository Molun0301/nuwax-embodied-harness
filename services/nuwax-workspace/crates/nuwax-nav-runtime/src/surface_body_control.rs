//! 完整三维机体路径跟随与共享制动检查，仍由原调度器唯一写入运动守卫。
use super::*;
use nuwax_nav_control::{choose_tracking_control_with_period_check, PlanarPose, TrackingScore};
use crate::command_dispatch::{latest_start_for_query, ObservationClockAnchor, ObservationProofAccumulator};
type BodyPose = ([f64; 3], [f64; 4]);
// Existing waypoint rotation completion tolerance, also reused as experimental
// alignment guidance. It is not a body-clearance or motion-permission threshold.
const WAYPOINT_ROTATION_TOLERANCE_RAD: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
struct BodyMotionSuccess {
    pose: BodyPose,
    // Dependency of the entire successful query, including its future-view check.
    latest_start_ns: Option<u64>,
}
#[cfg(test)]
type BodyMotionResult = Result<BodyMotionSuccess, nuwax_nav_mls_planner::surface::SurfaceError>;


#[derive(Clone, Debug, PartialEq)]
enum BodyMotionFailure {
    Legacy(nuwax_nav_mls_planner::surface::SurfaceError),
    Body(nuwax_nav_mls_planner::surface::BodyFailureWitness),
}
impl BodyMotionFailure {
    #[cfg(test)]
    fn legacy_error(&self)->nuwax_nav_mls_planner::surface::SurfaceError {
        match self {Self::Legacy(error)=>error.clone(),Self::Body(witness)=>witness.rejection.error()}
    }
}
impl From<nuwax_nav_mls_planner::surface::SurfaceError> for BodyMotionFailure {
    fn from(error:nuwax_nav_mls_planner::surface::SurfaceError)->Self {Self::Legacy(error)}
}
impl std::fmt::Display for BodyMotionFailure {
    fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result {
        match self {Self::Legacy(error)=>write!(f,"{error}"),Self::Body(witness)=>write!(f,"{}; {witness}",witness.rejection.error())}
    }
}
fn motion_failure<T>(result:Result<T,nuwax_nav_mls_planner::surface::SurfaceError>,
    witness:Option<nuwax_nav_mls_planner::surface::BodyFailureWitness>)->Result<T,BodyMotionFailure> {
    result.map_err(|error|witness.map_or_else(||BodyMotionFailure::Legacy(error),BodyMotionFailure::Body))
}

/// 仅在一次控制选择内共享；地图、当前足迹和观察集合由该调用固定。
#[derive(Hash, PartialEq, Eq)]
struct BodyMotionQuery {
    from: [u64; 7],
    target: [u64; 3],
    preserve_height: bool,
    observation_time_ns: Option<u64>,
}

#[cfg(test)]
fn memoized_body_motion(
    cache: &std::cell::RefCell<std::collections::HashMap<BodyMotionQuery, BodyMotionResult>>,
    proof: &ObservationProofAccumulator,
    from: BodyPose,
    target: PlanarPose,
    preserve_height: bool,
    observation_time_ns: Option<u64>,
    check: impl FnOnce() -> BodyMotionResult,
) -> BodyMotionResult {
    memoized_body_motion_impl(cache,proof,from,target,preserve_height,observation_time_ns,check)
}

fn memoized_body_motion_impl<E:Clone>(
    cache:&std::cell::RefCell<std::collections::HashMap<BodyMotionQuery,Result<BodyMotionSuccess,E>>>,
    proof:&ObservationProofAccumulator,from:BodyPose,target:PlanarPose,preserve_height:bool,
    observation_time_ns:Option<u64>,check:impl FnOnce()->Result<BodyMotionSuccess,E>,
)->Result<BodyMotionSuccess,E> {
    // 不量化坐标、不归一化四元数；不同的整拍/细分站姿必须分别查询。
    let key = BodyMotionQuery {
        from: [from.0[0], from.0[1], from.0[2], from.1[0], from.1[1], from.1[2], from.1[3]]
            .map(f64::to_bits),
        target: [target.x, target.y, target.yaw].map(f64::to_bits),
        preserve_height,
        observation_time_ns,
    };
    if let Some(result) = cache.borrow().get(&key) {
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::MotionMemoHits);
        if let Ok(success)=result {proof.merge(success.latest_start_ns);}
        return result.clone();
    }
    #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::MotionMemoMisses);
    let result = check();
    if let Ok(success)=&result {proof.merge(success.latest_start_ns);}
    cache.borrow_mut().insert(key, result.clone());
    result
}

fn body(p: &PoseStamped) -> ([f64; 3], [f64; 4]) {
    let q = p.pose.orientation;
    (xyz(p.pose.position), [q.x, q.y, q.z, q.w])
}
fn planar(p: &PoseStamped) -> PlanarPose {
    PlanarPose {
        x: p.pose.position.x,
        y: p.pose.position.y,
        yaw: p.pose.orientation.yaw(),
    }
}
fn horizontal_distance(a: &PoseStamped, b: &PoseStamped) -> f64 {
    (a.pose.position.x - b.pose.position.x).hypot(a.pose.position.y - b.pose.position.y)
}
fn is_rotation(a:&PoseStamped,b:&PoseStamped)->bool {
    horizontal_distance(a,b)<1e-5
        && angle_diff(a.pose.orientation.yaw(),b.pose.orientation.yaw()).abs()>1e-6
}
// Route-intent eligibility only. This grants no body-motion permission.
// Duplicate points are harmless only when they carry no explicit yaw change.
fn forward_straight_run(path: &Path3D, first: usize, last: usize, actual: Point3) -> bool {
    if first == 0 || last < first || last >= path.poses.len() { return false; }
    let a=&path.poses[first-1]; let b=&path.poses[first];
    let dx=b.pose.position.x-a.pose.position.x; let dy=b.pose.position.y-a.pose.position.y;
    if dx.hypot(dy)<=1e-5 { return false; }
    let direction=dy.atan2(dx);
    for pair in path.poses[first-1..=last].windows(2) {
        let x=pair[1].pose.position.x-pair[0].pose.position.x;
        let y=pair[1].pose.position.y-pair[0].pose.position.y;
        if angle_diff(pair[1].pose.orientation.yaw(),pair[0].pose.orientation.yaw()).abs()>1e-6 {
            return false;
        }
        if x.hypot(y)>1e-5 && angle_diff(y.atan2(x),direction).abs()>1e-6 { return false; }
    }
    let target=&path.poses[last].pose.position;
    (target.x-actual.x)*direction.cos()+(target.y-actual.y)*direction.sin()>0.0
}

// Eligibility for omitting future straight-line progress advice only. The
// actual command still passes the complete current-state guard. Check every
// duplicate's height, not just endpoints that a same-XY scan can skip over.
fn collinear_same_height_progress_run(path: &Path3D, first: usize, last: usize, actual: Point3) -> bool {
    forward_straight_run(path,first,last,actual)
        && path.poses[first-1..=last].iter()
            .all(|pose|pose.pose.position.z==path.poses[first-1].pose.position.z)
}

fn tracking_error(preview:&[PlanarPose],segments:&[(PlanarPose,PlanarPose)],radius:f64)->Option<f64> {
    #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::TrackingReferenceProjection);
    if segments.is_empty() {return None;}
    let mut total=0.0;
    for pose in preview {
        let mut closest=None;
        for &(a,b) in segments {
            let dx=b.x-a.x;let dy=b.y-a.y;let length2=dx*dx+dy*dy;
            if length2<=1e-12 {continue;}
            let fraction=(((pose.x-a.x)*dx+(pose.y-a.y)*dy)/length2).clamp(0.0,1.0);
            let distance=(pose.x-a.x-fraction*dx).hypot(pose.y-a.y-fraction*dy);
            let turn=angle_diff(b.yaw,a.yaw);
            let direction=dy.atan2(dx);
            // 位置直段可能保留旧同向姿态；严格前进应沿XY切向，不把原侧移朝向当作纠偏目标。
            // 真正曲线继续使用规划切向变化；同XY显式转向不进入平移段评分。
            let heading=if turn.abs()<=1e-6 && angle_diff(direction,a.yaw).abs()>1e-6 {
                direction
            }else {a.yaw+fraction*turn};
            if closest.is_none_or(|(best,_)|distance<best) {closest=Some((distance,heading));}
        }
        let (distance,heading)=closest?;
        // 角度偏差按真实机体半径折算为端部偏移，与路径横差使用同一距离单位。
        total+=distance+radius*angle_diff(pose.yaw,heading).abs();
    }
    Some(total/preview.len().max(1) as f64)
}

/// 原控制周期内离散降速至弯道速度的距离；包含本拍及声明响应延迟。
fn distance_to_curve_speed(speed:f64, curve_speed:f64, rate:f64, dt:f64, delay:f64)->f64 {
    if speed<=curve_speed {return 0.0;}
    let delta=speed-curve_speed;
    let steps=(delta/(rate*dt)).ceil();
    speed*(dt+delay)+dt*(steps*speed-rate*dt*steps*(steps-1.0)*0.5-delta*0.5)
}
/// 接续必须从实际站姿沿同一地形模型推进，不能借旧路径的高度/倾斜跨过障碍。
pub(super) fn connection_endpoint(map: &SurfaceSnapshot, from: ([f64; 3], [f64; 4]), next: &PoseStamped) -> Option<([f64;3],[f64;4])> {
    #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ConnectionEndpoint);
    let model=map.body_envelope()?;
    let q=from.1;
    let initial_yaw=Quaternion{x:q[0],y:q[1],z:q[2],w:q[3]}.yaw();
    let turn=angle_diff(next.pose.orientation.yaw(),initial_yaw);
    let dx=next.pose.position.x-from.0[0];let dy=next.pose.position.y-from.0[1];
    let distance=dx.hypot(dy);
    let count=((distance+model.radius()*turn.abs())/(f64::from(map.source_voxel_size_m())*0.5))
        .max(turn.abs()/0.05).ceil().max(1.0) as usize;
    if count>4096 {return None;}
    let mut previous=from;
    for index in 1..=count {
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::ConnectionSubsteps);
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::BodyProjectionCalls);
        let fraction=index as f64/count as f64;
        let projection={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ConnectorProjection);
            map.body_pose_after_motion(previous,
                [from.0[0]+dx*fraction,from.0[1]+dy*fraction],initial_yaw+turn*fraction)
        };
        let Ok(mut projected)=projection else {return None;};
        if distance<1e-9 {projected.0[2]=previous.0[2];}
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::GeometrySweeps);
        let swept={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ConnectorSweep);
            map.validate_body_motion(previous,projected,&[])
        };
        if swept.is_err() {return None;}
        previous=projected;
    }
    Some(previous)
}

/// 以控制器实际前进圆弧核对接续；直弦不可行不能否定绕过墙角的曲线。
fn forward_arc_connects(map:&SurfaceSnapshot,from:BodyPose,next:&PoseStamped)->bool {
    #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ForwardArcConnector);
    let Some(model)=map.body_envelope()else{return false;};
    let q=from.1;let yaw=Quaternion{x:q[0],y:q[1],z:q[2],w:q[3]}.yaw();
    let dx=next.pose.position.x-from.0[0];let dy=next.pose.position.y-from.0[1];let distance=dx.hypot(dy);
    let error=angle_diff(dy.atan2(dx),yaw);
    if error.abs()>=std::f64::consts::FRAC_PI_2{return false;}
    let angle=2.0*error;let length=if error.abs()<1e-9{distance}else{distance*error/error.sin()};
    let count=((length+model.radius()*angle.abs())/(f64::from(map.source_voxel_size_m())*0.5)).max(angle.abs()/0.05).ceil().max(1.0)as usize;
    if count>4096{return false;}
    let start=PlanarPose{x:from.0[0],y:from.0[1],yaw};let motion=Twist2{linear:Point2::new(length,0.0),angular_z:angle};
    let mut previous=from;
    for index in 1..=count {
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::ForwardArcSubsteps);
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::BodyProjectionCalls);
        let planar=nuwax_nav_control::integrate(start,motion,index as f64/count as f64);
        let projection={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ConnectorProjection);
            map.body_pose_after_motion(previous,[planar.x,planar.y],planar.yaw)
        };
        let Ok(projected)=projection else{return false;};
        #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::GeometrySweeps);
        let swept={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::ConnectorSweep);
            map.validate_body_motion(previous,projected,&[])
        };
        if swept.is_err(){return false;}
        previous=projected;
    }
    true
}

pub(super) fn translation_connects(map:&SurfaceSnapshot,from:([f64;3],[f64;4]),next:&PoseStamped)->bool {
    #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::TranslationConnector);
    connection_endpoint(map,from,next).is_some() || {
        // 窄门前需要先对齐，直弦上同时平移和改朝向可能真实碰框。
        // 仅证明完整转向及后续接续；实际执行仍受前向候选和制动扫掠约束。
        let mut aligned=next.clone();
        aligned.pose.position=Point3::new(from.0[0],from.0[1],from.0[2]);
        connection_endpoint(map,from,&aligned)
            .is_some_and(|arrival|connection_endpoint(map,arrival,next).is_some())
    }
}

impl SurfaceFollower {
    /// 用已知前向路径曲率提前降速，原始能力与完整停车许可仍保持不变。
    fn curve_speed_limit(&self,path:&Path3D,index:usize,position:Point3,lookahead:f64,angular:f64)->Option<f64> {
        let cruise=self.config.controller.speed*self.config.controller.nerf_speed.min(1.0);
        let mut limit=cruise;
        let dt=1.0/self.config.controller.control_frequency;
        let physical=self.config.motion_limits.as_ref();
        let preview=physical.map_or(lookahead,|motion|lookahead.max(distance_to_curve_speed(
            self.last_command.linear.x.max(0.0),0.0,
            motion.guaranteed_braking_mps2.min(motion.linear_acceleration_mps2),dt,motion.reaction_time_sec)));
        let mut travelled=0.0;
        for pair_index in index.saturating_sub(1)..path.poses.len().saturating_sub(1) {
            let a=&path.poses[pair_index];let b=&path.poses[pair_index+1];
            let dx=b.pose.position.x-a.pose.position.x;let dy=b.pose.position.y-a.pose.position.y;
            let length=dx.hypot(dy);
            let turn=angle_diff(b.pose.orientation.yaw(),a.pose.orientation.yaw());
            if length<1e-5 {
                if turn.abs()>1e-6 {break;}
                continue;
            }
            let curvature=(2.0*(turn*0.5).sin()/length).abs();
            if curvature>0.0 {
                let curve_speed=angular/curvature;
                if curve_speed<limit {
                    let available=if let Some(motion)=physical {
                        let rate=motion.guaranteed_braking_mps2.min(motion.linear_acceleration_mps2);
                        if distance_to_curve_speed(limit,curve_speed,rate,dt,motion.reaction_time_sec)<=travelled {limit}
                        else {
                            let mut lower=curve_speed;let mut upper=limit;
                            for _ in 0..40 {
                                let middle=(lower+upper)*0.5;
                                if distance_to_curve_speed(middle,curve_speed,rate,dt,motion.reaction_time_sec)<=travelled {lower=middle;}
                                else {upper=middle;}
                            }
                            lower
                        }
                    } else {curve_speed};
                    limit=limit.min(available);
                }
            }
            let fraction=if pair_index==index.saturating_sub(1) {
                (((position.x-a.pose.position.x)*dx+(position.y-a.pose.position.y)*dy)/(length*length)).clamp(0.0,1.0)
            } else {0.0};
            travelled+=length*(1.0-fraction);
            if travelled>preview {break;}
        }
        (limit<cruise).then_some(limit)
    }

    /// 从实际站姿顺序推进短时轨迹，只允许真实地形引起的相对姿态变化。
    pub(super) fn predicted_body_motion_is_traversable(
        &self,
        command: Twist2,
        horizon: f64,
    ) -> bool {
        let (Some(map), Some(pose)) = (self.map.as_ref(), self.odometry.as_ref()) else {
            return false;
        };
        let Some(model) = map.body_envelope() else {
            return false;
        };
        let from = body(pose);
        let start = planar(pose);
        let count = ((command.linear.x.hypot(command.linear.y)
            + model.radius() * command.angular_z.abs())
            * horizon
            / (f64::from(map.source_voxel_size_m()) * 0.5))
            .ceil()
            .max(2.0) as usize;
        if count > 512 {
            return false;
        }
        let mut previous = from;
        let mut delayed_previous = from;
        let rotating_in_place = command.linear.x == 0.0 && command.linear.y == 0.0;
        for index in 1..=count {
            let time = horizon * index as f64 / count as f64;
            let next = nuwax_nav_control::integrate(start, command, time);
            let delayed = nuwax_nav_control::integrate(
                start,
                Twist2 {
                    linear: command.linear,
                    angular_z: 0.0,
                },
                time,
            );
            let advance=|from:([f64;3],[f64;4]),target:PlanarPose| {
                let mut terrain=map.body_pose_after_motion(from,[target.x,target.y],target.yaw).ok()?;
                if (target.x-from.0[0]).hypot(target.y-from.0[1])<1e-9 {terrain.0[2]=from.0[2];}
                Some(terrain)
            };
            let Some(next)=advance(previous,next) else {return false;};
            if rotating_in_place {
                // 纯转向的延迟响应停在同一位置；前段逐段扫掠已经包含所有
                // 中间转角，无需重复检查静止段和从该角度倒扫回起点。
                if map.validate_body_motion(previous, next, &self.boundary).is_err() {
                    return false;
                }
                previous = next;
                continue;
            }
            let Some(delayed)=advance(delayed_previous,delayed) else {return false;};
            if map.validate_body_motion(previous, next, &self.boundary).is_err()
                || map
                    .validate_body_motion(delayed_previous, delayed, &self.boundary)
                    .is_err()
                || map.validate_body_motion(next, delayed, &self.boundary).is_err()
            {
                return false;
            }
            previous = next;
            delayed_previous = delayed;
        }
        true
    }

    pub(super) fn begin_braking(&mut self) {
        self.last_command_dispatch_bound = None;
        // 地图位姿包含定位校正，不能差分成真实速度或另一份静止证明。
        // 仅在实际生成过非零控制时保留标定制动上界；重复零速不延长等待。
        if self.last_command != Twist2::ZERO {
            if let Some(limits) = &self.config.motion_limits {
                let dt=1.0/self.config.controller.control_frequency;
                let tail=(self.last_command.linear.x.hypot(self.last_command.linear.y)/limits.guaranteed_braking_mps2)
                    .max(self.last_command.angular_z.abs()/limits.guaranteed_angular_braking_rps2);
                // 与候选检查使用同一已确认速度及离散停车模型，小速度不必等满设备最高速度的停时。
                let seconds=limits.reaction_time_sec+(tail/dt).ceil()*dt;
                let duration=std::time::Duration::try_from_secs_f64(seconds)
                    .unwrap_or_else(|_|std::time::Duration::from_secs_f64(limits.maximum_stopping_time_sec()));
                if self.config.controller.simulation {
                    let now=self.odometry.as_ref().map_or(0,|pose|pose.stamp.nanos);
                    self.simulation_braking_deadline_ns=Some(now.saturating_add(duration.as_nanos().min(i64::MAX as u128)as i64));
                    self.braking_deadline=None;
                } else {
                    self.braking_deadline=Some(std::time::Instant::now()+duration);
                    self.simulation_braking_deadline_ns=None;
                }
            }
        }
        self.last_command = Twist2::ZERO;
    }
    pub(super) fn follow_body_step(&mut self, robot_width: f64) -> PlannerStep {
        self.follow_body_step_inner::<true>(robot_width)
    }

    // The false specialization is exercised only by differential tests. The
    // production specialization reuses no state beyond this one invocation.
    fn follow_body_step_inner<const REUSE_LOOKAHEAD: bool>(&mut self, robot_width: f64) -> PlannerStep {
        self.follow_body_step_policy::<REUSE_LOOKAHEAD, true, true, true>(robot_width)
    }

    // Route intent and eligible collinear progress remain advice;
    // the unchanged calibrated current-state guard authorizes each command.
    fn follow_body_step_policy<const REUSE_LOOKAHEAD: bool, const STRAIGHT_INTENT: bool, const ALIGN_STRAIGHT: bool, const TRANSLATION_INTENT: bool>(&mut self, robot_width: f64) -> PlannerStep {
        self.follow_body_step_progress_policy::<REUSE_LOOKAHEAD,STRAIGHT_INTENT,ALIGN_STRAIGHT,TRANSLATION_INTENT,true>(robot_width)
    }

    // The explicit specialization retains the legacy progress reference for
    // differential measurements and barrier tests.
    fn follow_body_step_progress_policy<const REUSE_LOOKAHEAD: bool, const STRAIGHT_INTENT: bool, const ALIGN_STRAIGHT: bool, const TRANSLATION_INTENT: bool, const COLLINEAR_PROGRESS: bool>(&mut self, robot_width: f64) -> PlannerStep {
        // Scalar provenance for a rejected connector; no additional geometry
        // query or hot-path allocation is needed to diagnose the chosen target.
        let initial_path_index=self.path_index;
        let initial_alignment_index=self.alignment_target_index;
        let initial_progress=self.progress_m;
        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::RouteIntent);
        if let Some(error) = &self.invalid_input {
            return PlannerStep::Failed(error.clone());
        }
        if let Some(deadline)=self.simulation_braking_deadline_ns {
            if self.odometry.as_ref().is_none_or(|pose|pose.stamp.nanos<deadline) {
                return PlannerStep::Command(Twist2::ZERO);
            }
            self.simulation_braking_deadline_ns=None;
        }
        if let Some(deadline) = self.braking_deadline {
            if std::time::Instant::now() < deadline {
                return PlannerStep::Command(Twist2::ZERO);
            }
            self.braking_deadline = None;
        }
        let (Some(map), Some(pose), Some(path), Some(goal)) = (
            self.map.as_ref(),
            self.odometry.as_ref(),
            self.path.as_ref(),
            self.goal.as_ref(),
        ) else {
            return PlannerStep::ReplanRequired("机体执行输入缺失".into());
        };
        if !robot_width.is_finite() || robot_width <= 0.0 {
            return PlannerStep::Failed("机器人宽度无效".into());
        }
        let mut body_witness=None;
        let body_validity={
            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::CurrentBodyValidation);
            map.validate_body_pose_with_witness(body(pose).0, body(pose).1,&mut body_witness)
        };
        if let Err(e) = motion_failure(body_validity,body_witness) {
            return PlannerStep::ReplanRequired(format!("当前三维机体包络无效: {e}"));
        }
        #[cfg(test)] let profile_support=crate::control_profile::guard(crate::control_profile::Stage::BodySupportGoal);
        let position = pose.pose.position;
        let yaw = pose.pose.orientation.yaw();
        let cruise_speed = self.config.controller.speed*self.config.controller.nerf_speed.min(1.0);
        // 前瞻仍是有限局部范围；高速直线须容纳巡航速度对应的控制距离，避免距离增益隐式限速。
        let lookahead_m = self.config.lookahead_m.max(cruise_speed/1.5+f64::from(map.config().voxel_size));
        let actual_support = map.locate_support_from_body(
            body(pose).0, body(pose).1, self.config.body_to_support_z_m,
        ).ok();
        // 目标选点与规划候选共同使用原三维容差球，同层不能扩大最终到达范围。
        let final_distance = position.distance(goal.pose.position);
        let final_yaw = angle_diff(goal.pose.orientation.yaw(), yaw);
        if final_distance < self.config.controller.goal_tolerance
            && (!self.options.requires_orientation() || final_yaw.abs() < self.config.controller.orientation_tolerance)
        {
            // 这里只确认到达规划终点；调度器随后撤租归零，等待守卫的物理
            // 停稳证明后才发布成功，避免地图噪声形成第二道停车门槛。
            return PlannerStep::Succeeded;
        }
        #[cfg(test)] drop(profile_support);
        #[cfg(test)] let profile_waypoints=crate::control_profile::guard(crate::control_profile::Stage::WaypointProgress);
        // 逐点保留转向停留；曲线点只在到达或沿已确认小段越过时消耗。
        let tolerance = (f64::from(map.config().voxel_size) * 0.7).min(0.07);
        while self.path_index < path.poses.len() {
            let target = &path.poses[self.path_index];
            let previous = &path.poses[self.path_index - 1];
            // 连续曲线拼接会保留重复端点；零长度、零偏航变化不产生新的转向意图。
            let rotation = is_rotation(target,previous);
            let error = angle_diff(target.pose.orientation.yaw(), yaw);
            let same_support = actual_support.zip(map.locate_goal_support_from_body(
                body(target).0, body(pose).1, self.config.body_to_support_z_m,
            ).ok()).is_some_and(|(a,b)| {
                (a.surface_xyz[2]-b.surface_xyz[2]).abs()
                    <= f64::from(map.source_voxel_size_m())+1e-6
            });
            // 路径高度用于选择地面层，接触参考按实际姿态转换；平面执行器
            // 不控制路径拟合的横滚/俯仰。地面厚层允许实际足端参考位于层内。
            // 已通过完整机体检查且关联
            // 同一高度层时，按水平进度消耗路径点；平面速度无法补偿层顶偏差。
            // 最终到达仍使用上方完整三维距离，不跨楼层或扩大用户容差。
            let distance = if same_support {
                (position.x-target.pose.position.x).hypot(position.y-target.pose.position.y)
            } else {position.distance(target.pose.position)};
            let passed = if rotation {
                // 实际步态可能一次越过多个细分角度；沿同向进度消费，不倒转追旧点。
                let step = angle_diff(target.pose.orientation.yaw(), previous.pose.orientation.yaw());
                let actual = angle_diff(yaw, previous.pose.orientation.yaw());
                step * actual > 0.0 && actual.abs() >= step.abs()
                    && actual.abs() < std::f64::consts::FRAC_PI_2
            } else {
                let a = previous.pose.position;
                let b = target.pose.position;
                let dx=b.x-a.x;let dy=b.y-a.y;
                let dot = (position.x - b.x) * dx + (position.y - b.y) * dy;
                let lateral=((position.x-b.x)*dy-(position.y-b.y)*dx).abs()/dx.hypot(dy);
                // 位姿可低于控制频率到达；同层沿线越过多个采样点时按横向偏差消费，不能反向追旧点。
                dot >= 0.0 && (distance < tolerance * 2.0 || same_support && lateral < tolerance * 2.0)
            };
            // 换线时的实际制动位移可能越过新曲线的起始采样，但仍有横差。
            // 这些点既不能按到点容差直接跳过，也不能掉头重新追赶；从实际站姿重接局部路线。
            // 同层、沿原段已越过且目标落在当前机体后方，三者共同确认过期的平移目标。
            if !rotation && !passed && same_support && distance > tolerance {
                let a = previous.pose.position;
                let b = target.pose.position;
                let along = (position.x - b.x) * (b.x - a.x)
                    + (position.y - b.y) * (b.y - a.y);
                let forward = (b.x - position.x) * yaw.cos() + (b.y - position.y) * yaw.sin();
                if along >= 0.0 && forward < 0.0 && horizontal_distance(previous, target) > 1e-5 {
                    return PlannerStep::ReplanRequired("实际机体已越过局部采样且存在横差，需要从当前站姿接续前方路线".into());
                }
            }
            // 容差区域内的替代终点也是路径终点。不能先按路径点容差消费它，
            // 再发现机器人仍在用户目标区域之外，陷入原地重规划循环。
            // 防止提前消费的是最终位置组；内部采样即使靠近原目标，也须允许沿已确认进度越过。
            // 否则实际高度使机体仍在原三维球外时，会被锁在身后内部点，无法走到合法末点。
            let terminal_position=path.poses[self.path_index..].iter()
                .all(|pose|horizontal_distance(target,pose)<1e-5);
            let goal_position=(target.pose.position.x-goal.pose.position.x)
                .hypot(target.pose.position.y-goal.pose.position.y)<self.config.controller.goal_tolerance;
            // 原目标外的安全截断末点仍须消费，交由后续分支请求新的完整路线。
            let reached_position = !terminal_position || !goal_position
                || final_distance < self.config.controller.goal_tolerance;
            let segment_length=horizontal_distance(target,previous);
            let curved=!rotation && segment_length>1e-5
                && angle_diff(target.pose.orientation.yaw(),previous.pose.orientation.yaw()).abs()>1e-6;
            // 短弧按相邻采样的中点切换，不能用整车到点范围一次消耗多段弯道。
            let arrival_distance=if curved {tolerance.min(segment_length*0.5)}else{tolerance};
            let midpoint_reached=curved && same_support && distance<tolerance && {
                let a=previous.pose.position;let b=target.pose.position;
                let dx=b.x-a.x;let dy=b.y-a.y;
                // 中点是沿路径的进度平面；末采样很短时，正常横差不能把其缩成无法到达的小圆。
                (position.x-a.x)*dx+(position.y-a.y)*dy >= (dx*dx+dy*dy)*0.5
            };
            // 前段已允许在当前实际位置完成转向，旋转本身只消费角度进度。
            // 再要求旧支点距离会让已完成的转向永远不能退出；最终位置仍由reached_position约束。
            let reached = reached_position && (if rotation {error.abs()<WAYPOINT_ROTATION_TOLERANCE_RAD || passed}
                else {distance<arrival_distance || passed || midpoint_reached});
            let mut safe_to_skip = true;
            if reached && !rotation
                && (target.pose.position.x-previous.pose.position.x)
                    .hypot(target.pose.position.y-previous.pose.position.y)>1e-5 {
                // 微小平移可能是为后续转向让出净空，不能仅凭到点容差省略。
                // 只有当前位置能完成后续同位置转向，才允许提前消费这段平移。
                let mut turn_end = self.path_index;
                while turn_end + 1 < path.poses.len()
                    && horizontal_distance(&path.poses[turn_end + 1],target) < 1e-5 {
                    turn_end += 1;
                }
                let has_turn=path.poses[self.path_index..=turn_end].windows(2)
                    .any(|pair|is_rotation(&pair[0],&pair[1]));
                if !has_turn {
                    // 重复端点后面的真实移动段才决定能否提前消费当前点。
                    // 用零长度重复边判折点会漏掉必须依次经过的小弧接点。
                    if let Some(next) = path.poses.get(turn_end + 1) {
                        let incoming = [target.pose.position.x - previous.pose.position.x,
                            target.pose.position.y - previous.pose.position.y];
                        let outgoing = [next.pose.position.x - target.pose.position.x,
                            next.pose.position.y - target.pose.position.y];
                        let lengths = incoming[0].hypot(incoming[1]) * outgoing[0].hypot(outgoing[1]);
                        // 平滑曲线的采样点按沿线进度消费，实际横差不把每个采样变成折角。
                        // 真实折角仍核对后继接续；每拍完整机体及制动检查负责实际运动许可。
                        let off_segment = ((position.x-target.pose.position.x)*outgoing[1]
                            - (position.y-target.pose.position.y)*outgoing[0]).abs();
                        let collinear_progress_advice=COLLINEAR_PROGRESS && TRANSLATION_INTENT
                            && self.config.motion_limits.is_some() && same_support && !curved
                            && collinear_same_height_progress_run(path,self.path_index,turn_end+1,position);
                        if lengths > 1e-10 && (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
                            < lengths * 0.25_f64.cos()
                            || !curved && !collinear_progress_advice
                                && off_segment > outgoing[0].hypot(outgoing[1]) * 1e-5) {
                            // 短平移可能是绕过墙角所必需，只有接续可行才提前消费折点。
                            // 从实际站姿沿地形推进，不能借路径拟合的横滚/俯仰取得接续许可。
                            safe_to_skip = translation_connects(map,body(pose),next);
                            if !safe_to_skip && !connection_endpoint(map,body(pose),target)
                                .is_some_and(|arrival|translation_connects(map,arrival,next)) {
                                return PlannerStep::ReplanRequired("平移折点的实际站姿无法接续下一段，需要从实际位姿重规划".into());
                            }
                        }
                    }
                }
                if has_turn {
                    let turn_clear=|initial:([f64;3],[f64;4])| {
                        let mut previous=initial;
                        for waypoint in &path.poses[self.path_index+1..=turn_end] {
                            let mut at_waypoint=waypoint.clone();
                            at_waypoint.pose.position=Point3::new(initial.0[0],initial.0[1],initial.0[2]);
                            let Some(next)=connection_endpoint(map,previous,&at_waypoint) else {return false;};
                            previous=next;
                        }
                        true
                    };
                    safe_to_skip=turn_clear(body(pose));
                    if !safe_to_skip && !connection_endpoint(map,body(pose),target).is_some_and(turn_clear) {
                        // 从当前位置及真正到达原转向点的站姿都无法接续时，旧路径已失效。
                        return PlannerStep::ReplanRequired("转向点的规划站姿与实际接触不一致，需要按当前足迹重新规划".into());
                    }
                }
            }
            if reached && safe_to_skip {
                self.progress_m = self.cumulative[self.path_index];
                self.path_index += 1;
            } else {
                break;
            }
        }
        #[cfg(test)] drop(profile_waypoints);
        // 显式原地转向按已有角度进度完成；同方向细分角由下方前瞻连续消费。
        // 不再按剩余角度或整段角度把尚未完成的转向改写成平移曲线。
        if self.path_index >= path.poses.len() {
            if final_distance >= self.config.controller.goal_tolerance {
                return PlannerStep::ReplanRequired("三维安全路径结束，原目标未到达".into());
            }
            self.control_target = Some(PlanarPose {
                x: position.x,
                y: position.y,
                yaw: goal.pose.orientation.yaw(),
            });
            return self.bounded_command(Twist2 {
                linear: Point2::new(0.0, 0.0),
                angular_z: final_yaw.clamp(-0.5, 0.5),
            });
        }
        let target = &path.poses[self.path_index];
        let previous = &path.poses[self.path_index - 1];
        let deviation = if self.config.local_planning_enabled {
            position.distance(previous.pose.position).min(position.distance(target.pose.position))
        } else {
            // Global routes may have long sparse segments. Their midpoint is on
            // the route, even when it is far from both stored endpoints. This is
            // a distance check only; the same body/braking guards still run.
            let (_, projected) = closest_in_window(path, &self.cumulative, position,
                self.cumulative[self.path_index - 1], self.cumulative[self.path_index]);
            position.distance(projected.pose.position)
        };
        if deviation > self.config.max_path_deviation_m {
            return PlannerStep::ReplanRequired("机体偏离连续三维路线".into());
        }
        #[cfg(test)] let profile_lookahead=crate::control_profile::guard(crate::control_profile::Stage::LookaheadSelection);
        let rotation = is_rotation(target,previous);
        let local_path_yaw=(target.pose.position.y-previous.pose.position.y)
            .atan2(target.pose.position.x-previous.pose.position.x);
        let mut target_index = self.path_index;
        if rotation {
            // 同方向细分转向共用前瞻，避免每个离散角度都减速；不跨越反转或平移。
            let direction = angle_diff(target.pose.orientation.yaw(), previous.pose.orientation.yaw());
            while target_index + 1 < path.poses.len() {
                let a = &path.poses[target_index];
                let b = &path.poses[target_index + 1];
                if horizontal_distance(a,b)<1e-5 && !is_rotation(a,b) {
                    if position.distance(b.pose.position)>lookahead_m {break;}
                    target_index+=1;
                    continue;
                }
                let step = angle_diff(b.pose.orientation.yaw(), a.pose.orientation.yaw());
                let ahead = angle_diff(b.pose.orientation.yaw(), yaw);
                if horizontal_distance(b,previous) >= 1e-5
                    || step * direction <= 0.0
                    || ahead * direction <= 0.0
                    || ahead.abs() > std::f64::consts::FRAC_PI_2
                {
                    break;
                }
                target_index += 1;
            }
        } else {
            let mut turn_direction=angle_diff(target.pose.orientation.yaw(),previous.pose.orientation.yaw());
            if turn_direction.abs()<=1e-6 {turn_direction=0.0;}
            while target_index + 1 < path.poses.len() {
                let a = &path.poses[target_index];
                let b = &path.poses[target_index + 1];
                if horizontal_distance(a,b)<1e-5 && !is_rotation(a,b) {
                    if position.distance(b.pose.position)>lookahead_m {break;}
                    target_index+=1;
                    continue;
                }
                let next_turn=angle_diff(b.pose.orientation.yaw(),a.pose.orientation.yaw());
                // 当前圆弧不能越过S弯的曲率换向，否则目标点虽在路线上，机体切向却会被拉偏。
                if next_turn.abs()>1e-6 {
                    if turn_direction*next_turn<0.0 {break;}
                    if turn_direction==0.0 {turn_direction=next_turn;}
                }
                let direction = [target.pose.position.x-previous.pose.position.x,
                    target.pose.position.y-previous.pose.position.y];
                let next_direction = [b.pose.position.x - a.pose.position.x,
                    b.pose.position.y - a.pose.position.y];
                let lengths = direction[0].hypot(direction[1]) * next_direction[0].hypot(next_direction[1]);
                // 同位置、同偏航的地形接点不能把每20厘米都变成停点。
                // 真正的原地转向仍保留到点语义，最终偏航不能提前把机体带出目标球。
                if horizontal_distance(a,b)<1e-5
                    && angle_diff(b.pose.orientation.yaw(),a.pose.orientation.yaw()).abs()>1e-6
                    || direction[0] * next_direction[0] + direction[1] * next_direction[1]
                        < lengths * 0.25_f64.cos()
                    || angle_diff(b.pose.orientation.yaw(), target.pose.orientation.yaw()).abs()
                        > 0.25
                    || position.distance(b.pose.position) > lookahead_m
                {
                    break;
                }
                target_index += 1;
            }
        }
        // This experimental policy retires only a legacy translation-advice
        // latch. Explicit same-position turns keep their original progress and
        // selection semantics; calibrated current control remains mandatory.
        let translation_intent=TRANSLATION_INTENT && self.config.motion_limits.is_some() && !rotation;
        if translation_intent { self.alignment_target_index=None; }
        if self.alignment_target_index.is_some_and(|index|index<self.path_index || index>=path.poses.len()) {
            self.alignment_target_index=None;
        }
        if let Some(index)=self.alignment_target_index {target_index=index;}
        let straight_intent = STRAIGHT_INTENT && self.config.motion_limits.is_some()
            && !rotation && self.alignment_target_index.is_none()
            && forward_straight_run(path, self.path_index, target_index, position);
        let advisory_intent=straight_intent || translation_intent;
        let mut confirmed_lookahead=false;
        if !advisory_intent && !rotation && self.alignment_target_index.is_none() {
            // 前瞻按实际机体可执行的前进圆弧选择；墙角附近退回较近的路径点，
            // 不能只沿受阻的长弦不断减速，否则已知可执行的短路径永远得不到跟随。
            // 不能用先转向来保留远处曲线目标，否则会过早朝远端切向旋转并离开原路线。
            while target_index > self.path_index {
                if forward_arc_connects(map,body(pose),&path.poses[target_index]) {
                    confirmed_lookahead=true;
                    break;
                }
                target_index -= 1;
            }
        }
        #[cfg(test)] drop(profile_lookahead);
        let target = &path.poses[target_index];
        let heading_error = angle_diff(target.pose.orientation.yaw(), yaw);
        self.control_target = Some(planar(target));
        let mut preferred = Twist2 {
            linear: Point2::new(0.0, 0.0),
            angular_z: heading_error.clamp(-0.6, 0.6),
        };
        let mut speed_limit=None;
        // 直弦插值可能包含机体侧移，不能用它批准前向圆弧。
        // Only reuse the successful probe of this exact selected target. Map,
        // source stance and target are unchanged in this scope. Falling back to
        // path_index, alignment and rotation retain their original checks; this
        // is neither a cross-step permission nor a replacement for body control.
        let track_without_alignment=advisory_intent || rotation || (REUSE_LOOKAHEAD && confirmed_lookahead)
            || forward_arc_connects(map,body(pose),target);
        let alignment = if ALIGN_STRAIGHT && straight_intent
            && self.config.kinematics().is_some_and(|motion|motion.can_rotate_in_place)
            && angle_diff((target.pose.position.y-position.y).atan2(target.pose.position.x-position.x),yaw).abs()>=WAYPOINT_ROTATION_TOLERANCE_RAD {
            Some((target.pose.position.y-position.y).atan2(target.pose.position.x-position.x))
        } else if !track_without_alignment
            && self.config.kinematics().is_none_or(|motion|motion.can_rotate_in_place) {
            let heading=(target.pose.position.y-position.y).atan2(target.pose.position.x-position.x);
            let mut translated=target.clone();
            translated.pose.orientation=Quaternion::from_yaw(heading);
            let mut turned=translated.clone();turned.pose.position=position;
            connection_endpoint(map,body(pose),&turned)
                .and_then(|at_turn|connection_endpoint(map,at_turn,&translated)).map(|_|heading)
        }else{None};
        if !track_without_alignment && alignment.is_none() {
            // 旧路径在实际站姿下已不可接续，继续向缺测边界逼近只会让速度趋于零。
            // 按同一完整机体几何重新规划，不用微速或等待时长判断是否卡住。
            return PlannerStep::ReplanRequired(format!(
                "局部路径点{}无法从实际站姿按前进圆弧或转向接续，需要重新规划; initial_path_index={} initial_alignment_index={:?} initial_progress={:?} selected_index={} alignment_index={:?} path_len={} target={:?} previous={:?} applied={:?}",
                self.path_index,initial_path_index,initial_alignment_index,initial_progress,
                target_index,self.alignment_target_index,path.poses.len(),target.pose,previous.pose,self.last_command));
        }
        if let Some(heading)=alignment {
            // 转向期间固定这次接续目标，不能随偏航在相邻前瞻点之间切换意图。
            if !advisory_intent {self.alignment_target_index=Some(target_index);}
            // 先对齐真正的平移方向；不能在路径末切向与实际前进方向之间反复切换。
            preferred.angular_z=angle_diff(heading,yaw).clamp(-0.6,0.6);
            self.control_target=Some(PlanarPose{x:position.x,y:position.y,yaw:heading});
        } else if !rotation {
            let dx = target.pose.position.x - position.x;
            let dy = target.pose.position.y - position.y;
            let distance=dx.hypot(dy);
            let travel_yaw = dy.atan2(dx);
            let error = angle_diff(travel_yaw, yaw);
            let alignment_error=angle_diff(local_path_yaw,yaw);
            let rotate_allowed = self
                .config
                .kinematics()
                .is_none_or(|m| m.can_rotate_in_place);
            // OFF receives sparse reference segments, not Scan's executable
            // curve tangents. Alignment with the incoming segment cannot permit
            // translation toward a waypoint still beside/behind the actual body;
            // alternating those intents caused a turn/micro-motion limit cycle.
            if self.config.local_planning_enabled && alignment_error.abs() < 0.6
                || error.abs() < 0.6 || !rotate_allowed {
                let angular=self.config.motion_limits.as_ref().map_or_else(||
                    self.config.kinematics().map_or(self.config.max_angular_speed_rps,|motion|
                        motion.max_angular_rps.min(self.config.max_angular_speed_rps)),|motion|motion.max_angular_rps);
                speed_limit=self.curve_speed_limit(path,self.path_index,position,lookahead_m,angular);
                // 前进圆弧同时约束线速度和角速度，不能独立截断角速度后继续高速切弯。
                let lateral=-dx*yaw.sin()+dy*yaw.cos();
                let curvature=if distance>0.0 {2.0*lateral/(distance*distance)}else{0.0};
                let turning_speed=if curvature==0.0 {cruise_speed}else{angular/curvature.abs()};
                if turning_speed<cruise_speed {speed_limit=Some(speed_limit.map_or(turning_speed,|limit|limit.min(turning_speed)));}
                preferred.linear.x=cruise_speed.min(distance*1.5).min(speed_limit.unwrap_or(cruise_speed));
                preferred.angular_z=preferred.linear.x*curvature;
                // 评分所用末朝向与当前实际前进圆弧一致，不借旧横移路径姿态拉回朝向。
                self.control_target=Some(PlanarPose{x:target.pose.position.x,y:target.pose.position.y,
                    yaw:yaw+2.0*error});
            } else {
                preferred.angular_z = error.clamp(-0.6, 0.6);
                // 侧方/后方目标先对齐实际前进方向，候选评分也必须使用这次转向意图，
                // 不能被旧路径末姿态牵回当前方向而一直原地等待。
                self.control_target=Some(PlanarPose{x:position.x,y:position.y,yaw:travel_yaw});
            }
        }
        if preferred.linear.x.hypot(preferred.linear.y)<=1e-9 && preferred.angular_z.abs()<=1e-9 {
            return PlannerStep::ReplanRequired(format!(
                "局部路径点{}仅剩不可执行的支撑差异（高度差{:.3}米），需要从实际位姿重新规划",
                self.path_index,target.pose.position.z-position.z));
        }
        self.bounded_command_with_speed_limit(preferred,speed_limit)
    }
    // Keep observation and projected-body callbacks identical for ranking and a
    // standalone selected-candidate proof. Caches live only for this invocation.
    fn with_body_motion_checks<R>(&self, run: impl FnOnce(
        &mut dyn FnMut(PlanarPose, PlanarPose, f64) -> bool,
        &mut dyn FnMut(PlanarPose, PlanarPose, f64) -> bool,
        &std::cell::RefCell<Option<BodyMotionFailure>>,
        &ObservationProofAccumulator,
    ) -> R) -> Result<R, String> {
        #[cfg(test)] let proof_scope=crate::command_proof_trace::GuardScope::begin();
        #[cfg(test)] let profile_setup=crate::control_profile::guard(crate::control_profile::Stage::GuardSetup);
        let map=self.map.as_ref().ok_or("缺少表面地图")?;
        let pose=self.odometry.as_ref().ok_or("缺少实际位姿")?;
        let observation_check = if let Some(age) = self.config.max_observation_age_sec {
            let Ok(occupied) = map.body_observation_footprint(body(pose)) else {
                return Err("三维近场实际观察缺失".into());
            };
            let anchor = ObservationClockAnchor::capture();
            Some((occupied, anchor, (age * 1e9) as u64))
        } else { None };
        #[cfg(test)] proof_scope.record_basis(pose,self.last_command,
            observation_check.as_ref().map(|(_,anchor,age)|(anchor.unix_ns,*age)));
        let proof=ObservationProofAccumulator::new(observation_check.as_ref().map(|(_,anchor,_)|*anchor));
        let last_failure = std::cell::RefCell::new(None);
        let source_map=self.source_map.as_ref().unwrap_or(map);
        let motion_queries = std::cell::RefCell::new(std::collections::HashMap::new());
        let period_queries = std::cell::RefCell::new(std::collections::HashMap::new());
        let future_views = std::cell::RefCell::new(std::collections::HashMap::<[u64;7],Result<Arc<nuwax_nav_mls_planner::SurfaceBodyQueryView>,BodyMotionFailure>>::new());
        let boundary=&self.boundary;
        let make_check = |rebase:bool| {
            let mut previous_body=body(pose);
            let mut previous_xy=[pose.pose.position.x,pose.pose.position.y];
            let mut previous_yaw=pose.pose.orientation.yaw();
            let last_failure=&last_failure;
            let observation_check=&observation_check;
            let proof=&proof;
            // One last footprint-cell set per callback, bound to this immutable
            // view and original exemption. This footprint context stores no
            // time/evidence outcomes; each actual checker invocation uses the
            // existing now+elapsed. Outer exact-time motion memoization is unchanged.
            let mut observed_motion=observation_check.as_ref().map(|(occupied,anchor,age)|
                (anchor.unix_ns,map.observed_body_motion_checker_with_expiry_and_witness(occupied,*age,boundary)));
            // 两种回调的地形依据不同，不能共用运动许可；视图按精确站姿在本次选择内复用。
            let motion_queries=if rebase {&period_queries}else{&motion_queries};
            let future_views=&future_views;
            move |from:PlanarPose,to:PlanarPose,elapsed:f64| {
                // 每类独立轨迹从实际机体重置，不能把几何细分引入的高度带变化带入整拍预测。
                if elapsed == 0.0 || (from.x - previous_xy[0]).hypot(from.y - previous_xy[1]) > 1e-5
                    || angle_diff(from.yaw,previous_yaw).abs()>1e-5 {
                    previous_body = body(pose);
                }
                let a = previous_body;
                let preserve_height = (to.x-from.x).hypot(to.y-from.y)<1e-9;
                let elapsed_ns=(elapsed * 1e9) as u64;
                let observation_time = observation_check.as_ref()
                    .map(|(_, anchor, _)| anchor.unix_ns.saturating_add(elapsed_ns));
                let b=match memoized_body_motion_impl(motion_queries,proof,a,to,preserve_height,observation_time,|| -> Result<BodyMotionSuccess,BodyMotionFailure> {
                    let mut witness=None;
                    let future=if rebase && a!=body(pose) {
                        let key=[a.0[0],a.0[1],a.0[2],a.1[0],a.1[1],a.1[2],a.1[3]].map(f64::to_bits);
                        let cached=future_views.borrow().get(&key).cloned();
                        Some(match cached {
                            Some(view)=>{
                                #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::FutureViewHits);
                                view?
                            },
                            None=>{
                                #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::FutureViewBuilds);
                                // 从同一环境源派生机体查询视图，旧足迹不会累积，也不重建全局路由。
                                let view={
                                    #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::FutureViewConstruction);
                                    {let result=source_map.body_query_with_current_support_and_witness(a.0,a.1,&mut witness);
                                    motion_failure(result,witness).map(Arc::new)}
                                };
                                future_views.borrow_mut().insert(key,view.clone());view?
                            }
                        })
                    }else{None};
                    #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::BodyProjectionCalls);
                    let mut b={
                        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::GuardProjection);
                        match &future {
                            Some(view)=>{let result=view.body_pose_after_motion_with_witness(a,[to.x,to.y],to.yaw,&mut witness);motion_failure(result,witness)?},
                            None=>{let result=map.body_pose_after_motion_with_witness(a,[to.x,to.y],to.yaw,&mut witness);motion_failure(result,witness)?},
                        }
                    };
                    if preserve_height {b.0[2]=a.0[2];}
                    // 未来足迹不能自行授予新区域许可，始终保留本拍初始视图的完整检查。
                    let latest_start_ns=match observed_motion.as_mut() {
                        Some((now, check)) => {
                            #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::ObservedSweeps);
                            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::GuardObservedSweep);
                            {let result=check(a,b,now.saturating_add(elapsed_ns),&mut witness);
                            latest_start_for_query(motion_failure(result,witness)?,elapsed_ns)}
                        },
                        None => {
                            #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::GeometrySweeps);
                            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::GuardGeometrySweep);
                            let result=map.validate_body_motion_with_witness(a,b,boundary,&mut witness);
                            motion_failure(result,witness)?;
                            None
                        },
                    };
                    if let Some(view)=future {
                        // 完全同源同足迹的视图可复用刚通过的同一扫掠；其它视图独立核验。
                        if !view.same_body_query_view(map) {
                            #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::FutureGeometrySweeps);
                            #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::GuardGeometrySweep);
                            let result=view.validate_body_motion_with_witness(a,b,boundary,&mut witness);
                            motion_failure(result,witness)?;
                        }
                    }
                    Ok(BodyMotionSuccess {pose:b,latest_start_ns})
                }) {
                    Ok(success)=>success.pose,
                    Err(error)=>{
                        *last_failure.borrow_mut()=Some(error);return false;
                    }
                };
                previous_body = b;
                previous_xy = [to.x, to.y];
                previous_yaw=to.yaw;
                true
            }
        };
        #[cfg(test)] drop(profile_setup);
        let mut geometric_check=make_check(false);
        let mut period_check=make_check(true);
        let result=run(&mut geometric_check, &mut period_check, &last_failure, &proof);
        #[cfg(test)] {
            let _profile=crate::control_profile::guard(crate::control_profile::Stage::GuardCleanup);
            drop(period_check);drop(geometric_check);
            drop(future_views);drop(period_queries);drop(motion_queries);
            drop(last_failure);drop(observation_check);
        }
        Ok(result)
    }

    /// Timing-only entry: callers supply a known reachable candidate. This is
    /// not a command issuance API and does not install or renew any permission.
    #[cfg(test)]
    pub(crate) fn check_selected_candidate_for_test(&mut self, candidate: Twist2) -> bool {
        self.check_selected_candidate_periods_for_test(candidate, 1)
    }

    #[cfg(test)]
    pub(crate) fn check_selected_candidate_periods_for_test(&mut self, candidate: Twist2, periods: usize) -> bool {
        if self.current_body_error().is_some() { return false; }
        let (Some(map),Some(pose),Some(limits))=(self.map.as_ref(),self.odometry.as_ref(),self.config.motion_limits.as_ref()) else { return false; };
        let Some(model)=map.body_envelope() else {return false;};
        let limits=forward_navigation_limits(limits);
        self.with_body_motion_checks(|clear,period_clear,_,_| {
            nuwax_nav_control::check_selected_stopping_trajectory_with_period_check(
                planar(pose),self.last_command,candidate,&limits,
                1.0/self.config.controller.control_frequency,periods,
                f64::from(map.source_voxel_size_m())*0.5,model.radius(),clear,period_clear,
            ).is_some()
        }).unwrap_or(false)
    }

    pub(super) fn bounded_body_command(&mut self, preferred: Twist2, speed_limit:Option<f64>) -> PlannerStep {
        self.last_command_dispatch_bound = None;
        #[cfg(test)] crate::command_proof_trace::invalidate_selection();
        #[cfg(test)] let profile_setup=crate::control_profile::guard(crate::control_profile::Stage::GuardSetup);
        let (Some(map), Some(pose), Some(physical_limits)) = (
            self.map.as_ref(),
            self.odometry.as_ref(),
            self.config.motion_limits.as_ref(),
        ) else {
            return PlannerStep::ReplanRequired("缺少机体控制标定".into());
        };
        let mut limits=forward_navigation_limits(physical_limits);
        let Some(model) = map.body_envelope() else {
            return PlannerStep::ReplanRequired("缺少机体包络".into());
        };
        let target = self.control_target.unwrap_or_else(|| planar(pose));
        // 用已下发控制及标定动态界预测制动扫掠；map位姿仍用于实际
        // 位置、障碍和偏离检查，其坐标校正不充当执行器速度。
        let current = self.last_command;
        if let Some(limit)=speed_limit {
            // 曲率速度参考进入可达窗口；已有实际速度只能按原标定逐拍降低。
            let rate=limits.guaranteed_braking_mps2.min(limits.linear_acceleration_mps2);
            let next_minimum=(current.linear.x-rate/self.config.controller.control_frequency).max(0.0);
            limits.max_forward_mps=limits.max_forward_mps.min(limit.max(next_minimum).max(f64::MIN_POSITIVE));
        }
        let dt=1.0/self.config.controller.control_frequency;
        let tracking_horizon=crate::DEFAULT_CONTROL_HORIZON.as_secs_f64()
            .max(dt+limits.reaction_time_sec+current.angular_z.abs()/limits.guaranteed_angular_braking_rps2)
            .min(20.0);
        let mut reference_segments=Vec::new();
        if preferred.linear.x.hypot(preferred.linear.y)>1e-9 {
            if let Some(path)=&self.path {
                let terminal=path.poses.last().is_some_and(|last|
                    last.pose.position.x==target.x && last.pose.position.y==target.y);
                if !terminal {
                    let span=self.config.lookahead_m
                        .max(self.config.controller.speed*self.config.controller.nerf_speed.min(1.0)/1.5
                            +f64::from(map.config().voxel_size))
                        .max(current.linear.x.abs().max(limits.max_forward_mps)*tracking_horizon);
                    let mut travelled=0.0;
                    let start=self.path_index.saturating_sub(1).min(path.poses.len());
                    for pair in path.poses[start..].windows(2) {
                        if is_rotation(&pair[0],&pair[1]) {break;}
                        let length=horizontal_distance(&pair[0],&pair[1]);
                        if length<1e-5 {continue;}
                        reference_segments.push((planar(&pair[0]),planar(&pair[1])));
                        travelled+=length;
                        if travelled>=span {break;}
                    }
                }
            }
        }
        let mut rotation_useful=std::collections::HashMap::<u64,bool>::new();
        let reference_curvature=reference_segments.first().map(|(a,b)|
            2.0*(angle_diff(b.yaw,a.yaw)*0.5).sin()/(b.x-a.x).hypot(b.y-a.y));
        #[cfg(test)] drop(profile_setup);
        let checked = self.with_body_motion_checks(|clear,period_clear,last_failure,proof| {
        #[cfg(test)] let _profile=crate::control_profile::guard(crate::control_profile::Stage::CandidateRankingSweeps);
        let result = choose_tracking_control_with_period_check(
            planar(pose),
            current,
            preferred,
            target,
            &limits,
            1.0 / self.config.controller.control_frequency,
            f64::from(map.source_voxel_size_m()) * 0.5,
            model.radius(),
            false,
            tracking_horizon,
            reference_curvature,
            |candidate,preview,stopped| {
                #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::TrackingScores);
                tracking_error(&preview,&reference_segments,model.radius()).map(|error|TrackingScore {
                    path_error_m:error,target_cost:(stopped.x-target.x).hypot(stopped.y-target.y)
                        +0.005*(candidate.angular_z-preferred.angular_z).abs()})
            },
            |candidate| {
                #[cfg(test)] crate::control_profile::count(crate::control_profile::Counter::CandidateEligibility);
                if current.linear.x.hypot(current.linear.y)<=1e-9
                    && preferred.linear.x.hypot(preferred.linear.y)>1e-9
                    && candidate.linear.x.hypot(candidate.linear.y)<=1e-9
                    && candidate.angular_z.abs()>1e-9 {
                    *rotation_useful.entry(candidate.angular_z.to_bits()).or_insert_with(||
                        self.rotation_can_restore_translation(preferred,candidate.angular_z))
                } else {true}
            },
            &mut *clear,
            &mut *period_clear,
        );
        // Ranking can inspect rejected and later Pareto candidates. Keep only
        // the exact returned command's full proof. This standalone call creates
        // a fresh current-stop accumulator, while both original callbacks and
        // their exact-time geometry memo entries stay in this invocation.
        proof.reset();
        let result=result.filter(|choice| {
            if choice.command.linear.x.hypot(choice.command.linear.y)<=1e-9
                && choice.command.angular_z.abs()<=1e-9 {return true;}
            nuwax_nav_control::check_stopping_trajectory_with_period_check(
                planar(pose),current,choice.command,&limits,dt,
                f64::from(map.source_voxel_size_m())*0.5,model.radius(),
                &mut *clear,&mut *period_clear,
            ).is_some()
        });
        let bound=if result.as_ref().is_some_and(|choice|
            choice.command.linear.x.hypot(choice.command.linear.y)>1e-9
                || choice.command.angular_z.abs()>1e-9)
        {proof.bound().map_err(str::to_owned)?}else{None};
        #[cfg(test)] if let Some(choice)=&result {
            crate::command_proof_trace::record_selected(choice.command);
        }
        Ok::<_,String>((result,last_failure.take(),bound))
        });
        #[cfg(test)] {
            let _profile=crate::control_profile::guard(crate::control_profile::Stage::GuardCleanup);
            drop(rotation_useful);drop(reference_segments);
        }
        let (result,last_failure,bound)=match checked.and_then(|value|value) {
            Ok(value)=>value,
            Err(error)=>return PlannerStep::ReplanRequired(error),
        };
        match result {
            Some(choice) if choice.command.linear.x.hypot(choice.command.linear.y) > 1e-9
                || choice.command.angular_z.abs() > 1e-9 => {
                self.last_command = choice.command;
                self.last_command_dispatch_bound = bound;
                PlannerStep::Command(choice.command)
            }
            Some(_) if current != Twist2::ZERO => {
                // 速度换向需要先过零；已验证的停止候选是控制接续，不代表整条路线失效。
                self.begin_braking();
                PlannerStep::Command(Twist2::ZERO)
            }
            _ => {
                self.begin_braking();
                PlannerStep::ReplanRequired(format!("三维局部控制未找到可推进速度，路径点{}，实际速度{:?}，参考速度{:?}，局部目标{:?}{}",
                    self.path_index,current,preferred,target,
                    last_failure.as_ref().map(|error|format!("，扫掠拒绝: {error}")).unwrap_or_default()))
            }
        }
    }
}

#[cfg(test)]
mod query_reuse_tests {
    use super::*;
    use nuwax_nav_control::choose_control_with_period_check;
    use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::{MotionLimits,Pose3};
    use std::{cell::{Cell, RefCell}, collections::{BTreeMap, HashMap}};

    fn narrow_door() -> SurfaceSnapshot {
        let fine=f64::from(0.01_f32);let width=32.0*fine;
        let mut keys:Vec<[i32;3]>=(-100..100).flat_map(|x|(-50..100).map(move|y|[x,y,-1])).collect();
        keys.extend((0..4).flat_map(|x|(-50..100).filter(|&y|y<0||y>=33)
            .flat_map(move|y|(0..80).map(move|z|[x,y,z]))));
        let points:Vec<_>=keys.iter().map(|k|(((f64::from(k[0])+0.5)*fine)as f32,
            ((f64::from(k[1])+0.5)*fine)as f32,((f64::from(k[2])+0.5)*fine)as f32)).collect();
        let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
        for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
            .insert(key,SurfaceObservation{observed_ns:0,hits:0});}
        let mut observations=SurfaceObservations::new(0.01,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        observations.mark_complete_geometry_source();
        let mut geometry=super::super::test_snapshot(0.0,false).config().clone();
        geometry.voxel_size=0.1;geometry.robot_height=0.1;geometry.surface_closing_radius=0.0;
        geometry.node_spacing_m=0.3;geometry.wall_clearance_m=0.0;geometry.step_threshold_m=0.0;
        SurfaceSnapshot::from_points(&points,geometry,0.01).unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.3,-width/2.0],
                [0.3,-width/2.0],[0.3,width/2.0],[-0.3,width/2.0]],z_bounds_m:[-0.15,0.15],
                body_to_support_z_m:0.4,max_step_up_m:0.0,max_step_down_m:0.0,
                max_cross_slope_deg:18.0,support_uncertainty_m:0.0}).unwrap()
            .with_observations(Some(Arc::new(observations)))
    }

    #[test]
    fn off_sparse_corner_turns_toward_actual_waypoint_before_translation() {
        let make=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let actual=make(2.55,1.85,0.3);
        let path=Path3D {poses:vec![make(1.05,0.55,0.15),make(2.35,2.05,0.17),make(5.05,1.55,-0.2)]};
        let source=Arc::new(map(0.0));
        let mut config=SurfaceFollowerConfig::new(0.3);config.local_planning_enabled=false;
        config.controller.simulation=true;config.controller.speed=0.5;config.controller.control_frequency=20.0;
        config.motion_limits=Some(nuwax_nav_types_messages::MotionLimits {
            model:nuwax_nav_types_messages::MotionModel::Differential,max_forward_mps:0.5,
            max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:0.8,
            linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        });
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.cumulative=cumulative_lengths(&path);follower.goal=path.poses.last().cloned();
        follower.path=Some(path);follower.path_index=1;follower.map=Some(source.clone());follower.source_map=Some(source);
        follower.odometry=Some(actual.clone());
        let step=follower.step(0.2);
        assert!(matches!(step,PlannerStep::Command(c) if c.linear.x==0.0 && c.angular_z>0.0),
            "incoming-segment alignment must not start a turn/translation cycle: {step:?}");
        let target=follower.control_target.unwrap();
        assert_eq!([target.x,target.y],[actual.pose.position.x,actual.pose.position.y]);
        assert!(follower.check_selected_candidate_for_test(follower.last_command));
    }

    #[test]
    fn narrow_door_connection_requires_real_alignment_before_entry() {
        let map=narrow_door();
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let start=pose(-0.31471258334529195,0.16213553950286064,0.05);
        let target=pose(-0.24500000044046072,0.1649999963119626,0.0);
        map.validate_body_pose(body(&start).0,body(&start).1).unwrap();
        assert!(connection_endpoint(&map,body(&start),&target).is_none(),
            "同一真实门框拒绝边进入边缓慢转正");
        assert!(translation_connects(&map,body(&start),&target),
            "门前完整转正后应能接回原直段");
        let inside=pose(0.02,0.1649999963119626,0.0);
        let forbidden=pose(0.03,0.1649999963119626,0.7);
        assert!(!translation_connects(&map,body(&inside),&forbidden),
            "门内没有转身空间时，两种接续都必须拒绝");
    }

    #[test]
    fn real_forward_arc_survives_a_blocked_chord_but_a_closed_connection_replans() {
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let start=pose(1.05,1.05,0.0);
        let target=pose(1.05+2.0*1.0_f64.sin(),1.05+2.0*(1.0-1.0_f64.cos()),1.0);
        for block_arc in [false,true] {
            let mut points:Vec<_>=(0..40).flat_map(|x|(0..40).map(move|y|
                ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
            // 第一根柱子切断直弦，第二根柱子才真正切断前进圆弧。
            for (x,y) in [(1.85,1.55)].into_iter().chain(block_arc.then_some((2.05,1.25))) {
                for z in 1..8 {points.push((x,y,(z as f32+0.5)*0.1));}
            }
            let base=map(0.0);
            let source=Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
            source.validate_body_pose(body(&start).0,body(&start).1).unwrap();
            assert!(connection_endpoint(&source,body(&start),&target).is_none(),"真实立柱必须挡住直弦");
            assert_eq!(forward_arc_connects(&source,body(&start),&target),!block_arc,
                "完整圆弧扫掠必须区分能绕过的柱子和真实堵路");
            let path=Path3D{poses:vec![start.clone(),target.clone()]};
            let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.2;
            let mut follower=SurfaceFollower::new(config).unwrap();
            follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
            follower.goal=Some(target.clone());follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
            follower.odometry=Some(start.clone());
            let step=follower.follow_body_step(0.2);
            if block_arc {
                assert!(matches!(step,PlannerStep::ReplanRequired(ref reason)if reason.contains("无法从实际站姿")),
                    "完整连接已失效时不能用微速继续追点: {step:?}");
            }else{
                assert!(matches!(step,PlannerStep::Command(command)if command.linear.x>0.0 && command.linear.y==0.0),
                    "可执行前向圆弧不能因直弦受阻被提前拒绝: {step:?}");
            }
        }
    }

    #[test]
    fn clear_chord_does_not_authorize_a_blocked_forward_arc() {
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let start=pose(1.05,1.05,0.0);
        let target=pose(1.05+2.0*1.0_f64.sin(),1.05+2.0*(1.0-1.0_f64.cos()),1.0);
        let mut points:Vec<_>=(0..40).flat_map(|x|(0..40).map(move|y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
        for z in 1..8 {points.push((2.05,1.25,(z as f32+0.5)*0.1));}
        let base=map(0.0);
        let source=Arc::new(SurfaceSnapshot::from_points(&points,base.config().clone(),0.1).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        assert!(connection_endpoint(&source,body(&start),&target).is_some());
        assert!(!forward_arc_connects(&source,body(&start),&target));
        let path=Path3D{poses:vec![start.clone(),target.clone()]};
        let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.2;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
        follower.goal=Some(target);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
        follower.odometry=Some(start);
        let step=follower.follow_body_step(0.2);
        assert!(matches!(step,PlannerStep::Command(command)if command.linear.x==0.0 && command.angular_z>0.0),
            "当前前向圆弧被柱子阻挡，应先对齐可执行方向，不能凭直弦许可直接平移：{step:?}");
    }

    #[test]
    fn explicit_turn_is_completed_before_starting_translation() {
        let make=|x:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,1.05,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let pi=std::f64::consts::PI;
        let source=Arc::new(map(0.0));let target=make(0.7,pi);
        let path=Path3D{poses:vec![make(1.05,0.0),make(1.05,pi),target.clone()]};
        let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.2;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
        follower.goal=Some(target);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
        follower.odometry=Some(make(1.05,pi-0.2));
        let step=follower.follow_body_step(0.2);
        assert!(matches!(step,PlannerStep::Command(c) if c.linear.x==0.0 && c.angular_z>0.0),
            "明确的原地转向尚差约11度时应继续转向，不能提前切入平移：{step:?}");
    }

    #[test]
    fn completed_turn_at_a_permitted_shifted_pivot_advances() {
        let make=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let source=Arc::new(map(0.0));let goal=make(1.8,1.05,0.12);
        let actual=make(1.2,1.1,0.12);
        let mut turned=make(1.2,1.1,0.0);
        assert!(connection_endpoint(&source,body(&turned),&actual).is_some(),"当前偏移处确实允许转向");
        turned.pose.position=Point3::new(1.05,1.05,0.4);
        let path=Path3D{poses:vec![turned,make(1.05,1.05,0.12),goal.clone()]};
        let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.2;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
        follower.goal=Some(goal);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
        follower.odometry=Some(actual);
        let step=follower.follow_body_step(0.2);
        assert!(follower.path_index>1 && matches!(step,PlannerStep::Command(c) if c.linear.x>0.0),
            "允许的偏移支点已转完，不能继续追旧支点而原地摆动：{step:?}");
    }

    #[test]
    fn passed_local_sample_reconnects_instead_of_turning_back_across_tracking_error() {
        let make = |x, y| PoseStamped {
            frame_id: nuwax_nav_types_messages::NavFrameId::Map,
            stamp: nuwax_nav_types_messages::Timestamp::from_unix(1, 0),
            pose: Pose3 { position: Point3::new(x, y, 0.4), orientation: Quaternion::IDENTITY },
        };
        let source = Arc::new(map(0.0));
        let goal = make(1.9, 1.05);
        let path = Path3D { poses: vec![make(1.05, 1.05), make(1.1, 1.05), goal.clone()] };
        let mut config = SurfaceFollowerConfig::new(0.3);
        config.controller.speed = 0.2;
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.cumulative = cumulative_lengths(&path);
        follower.path = Some(path);
        follower.path_index = 1;
        follower.goal = Some(goal);
        follower.map = Some(Arc::clone(&source));
        follower.source_map = Some(source);
        follower.odometry = Some(make(1.5, 1.3));
        let step = follower.follow_body_step(0.2);
        assert!(matches!(step, PlannerStep::ReplanRequired(ref reason) if reason.contains("越过局部采样")),
            "减速位移造成越过起始点并有横差时，须请求实际位姿接续，不能转回旧点：{step:?}");
        // 正常直线上的进度仍按原容差消费，不因采样位于后方就重规划。
        follower.odometry = Some(make(1.5, 1.06));
        let step = follower.follow_body_step(0.2);
        assert!(follower.path_index > 1 && matches!(step, PlannerStep::Command(c) if c.linear.x > 0.0),
            "同一直段的正常采样越过应继续前进：{step:?}");
    }

    #[test]
    fn duplicate_curve_sample_does_not_create_an_in_place_turn() {
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        for real_turn in [false,true] {
            let source=Arc::new(map(0.0));
            let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.3;
            let mut follower=SurfaceFollower::new(config).unwrap();
            let previous=pose(1.2,1.05,0.3);
            let target=if real_turn {pose(1.2,1.05,0.8)}else{previous.clone()};
            let path=Path3D {poses:vec![pose(1.0,1.05,0.2),previous,target,pose(1.4,1.10,0.4),pose(2.0,1.2,0.0)]};
            follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);
            follower.path_index=2;follower.goal=Some(pose(2.0,1.2,0.0));
            follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
            follower.odometry=Some(pose(1.19,1.04,0.1));
            let step=follower.follow_body_step(0.2);
            let PlannerStep::Command(command)=step else {panic!("合法曲线接续应产生控制: {step:?}");};
            if real_turn {
                assert_eq!(follower.path_index,2,"真正原地转向须保留");
                assert_eq!(command.linear.x,0.0);
            }else{
                assert!(follower.path_index>=3,"重复采样不能要求纠正旧路径末方向");
                assert!(command.linear.x>0.0,"实际站姿须继续前向曲线");
            }
            assert_eq!(command.linear.y,0.0);
        }
    }

    #[test]
    fn duplicated_curve_junction_keeps_required_intermediate_motion() {
        let pose=|x:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,0.1649999963119626,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let source=Arc::new(narrow_door());let start=pose(0.305,0.0);
        let junction=pose(0.36,0.0);let mut duplicate=junction.clone();
        duplicate.pose.position.y+=5e-17;
        let mut next=pose(0.395,0.3);
        next.pose.position.y+=0.035*0.15_f64.tan();
        let goal=pose(0.8,0.3);
        assert!(!translation_connects(&source,body(&start),&next),
            "机尾未出门时不能提前转入后面的曲线");
        let arrival=connection_endpoint(&source,body(&start),&junction).unwrap();
        assert!(connection_endpoint(&source,arrival,&next).is_some(),
            "原路线先离开门框再转弯必须可执行");
        let mut config=SurfaceFollowerConfig::new(0.4);config.controller.speed=0.3;
        let mut follower=SurfaceFollower::new(config).unwrap();
        let path=Path3D {poses:vec![start.clone(),junction,duplicate,next,goal.clone()]};
        follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
        follower.goal=Some(goal);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
        follower.odometry=Some(start);
        let step=follower.follow_body_step(0.32);
        let PlannerStep::Command(command)=step else {panic!("重复点不能令合法分段路线重规划: {step:?}");};
        assert_eq!(follower.path_index,1,"必须先执行离开门框的短直段");
        assert!(command.linear.x>0.0 && command.linear.y==0.0);
    }

    #[test]
    fn short_s_bend_keeps_its_progress_and_lookahead_on_the_current_curve() {
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let radius=0.2;let angle=0.6_f64;
        let mut poses:Vec<_>=(0..=10).map(|i| {
            let yaw=angle*i as f64/10.0;
            pose(1.0+radius*yaw.sin(),1.0+radius*(1.0-yaw.cos()),yaw)
        }).collect();
        let inflection=poses.last().unwrap().clone();
        for i in 1..=10 {
            let yaw=angle*(1.0-i as f64/10.0);
            poses.push(pose(inflection.pose.position.x+radius*(angle.sin()-yaw.sin()),
                inflection.pose.position.y+radius*(yaw.cos()-angle.cos()),yaw));
        }
        let goal=pose(2.5,poses.last().unwrap().pose.position.y,0.0);
        poses.push(goal.clone());
        let actual=poses[5].clone();let path=Path3D {poses};
        let source=Arc::new(map(0.0));
        let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=1.0;
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=6;
        follower.goal=Some(goal);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
        follower.odometry=Some(actual);
        let step=follower.follow_body_step(0.2);
        let PlannerStep::Command(command)=step else {panic!("连续S弯应保持可执行前进: {step:?}");};
        assert_eq!(follower.path_index,6,"七厘米到点范围不能提前消耗细分弧段");
        let target=follower.control_target.unwrap();
        assert!(target.x<=inflection.pose.position.x+1e-10,
            "单圆弧前瞻不能跨到反向曲率段");
        assert!((target.yaw-angle).abs()<1e-9,"当前圆弧的末切向必须与参照一致");
        assert!(command.linear.x>0.0 && command.linear.y==0.0);
    }

    #[test]
    fn translation_recovery_requires_the_selected_turn_direction_and_keeps_braking() {
        let source=Arc::new(map(0.0));
        let pose=PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(1.05,1.05,0.4),orientation:Quaternion::IDENTITY}};
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);follower.odometry=Some(pose);
        for angle in [-0.1_f64,0.1,-0.02,0.02] {
            follower.control_target=Some(PlanarPose{x:1.05+0.5*angle.cos(),y:1.05+0.5*angle.sin(),yaw:2.0*angle});
            let preferred=Twist2 {linear:Point2::new(0.2,0.0),angular_z:0.8*angle.sin()};
            let correct=angle.signum()*0.1;
            follower.last_command=Twist2::ZERO;
            assert!(follower.rotation_can_restore_translation(preferred,correct),"必要方向应保留");
            assert!(!follower.rotation_can_restore_translation(preferred,-correct),
                "存在某方向恢复依据不能授权反方向候选");
            follower.last_command=Twist2 {linear:Point2::new(0.0,0.0),angular_z:-correct};
            assert!(follower.rotation_can_restore_translation(preferred,-correct*0.5),
                "已有反向速度仍须允许按原标定减速过零");
            assert!(!follower.rotation_can_restore_translation(preferred,-correct*1.5),
                "不能借制动理由继续加大错误方向转速");
        }
    }

    #[test]
    fn short_curve_tail_is_consumed_after_its_longitudinal_midpoint() {
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        for tail in [0.0003778463496549095,0.00003758324675470232] {
            for fraction in [0.25,0.75] {
                let radius=0.6;let angle=0.04_f64;
                let at=|theta:f64|pose(1.0+radius*theta.sin(),1.0+radius*(1.0-theta.cos()),theta);
                let previous=at(angle);let end=at(angle+tail/radius);
                let mut actual=at(angle+fraction*tail/radius);
                actual.pose.position.x-=0.001*(angle+tail/(2.0*radius)).sin();
                actual.pose.position.y+=0.001*(angle+tail/(2.0*radius)).cos();
                let end_yaw=end.pose.orientation.yaw();let after_yaw=end_yaw-0.04;
                let after=pose(end.pose.position.x+0.25*(end_yaw.sin()-after_yaw.sin()),
                    end.pose.position.y+0.25*(after_yaw.cos()-end_yaw.cos()),after_yaw);
                let goal=pose(2.0,1.0,0.0);
                let path=Path3D {poses:vec![at(0.0),previous,end,after,goal.clone()]};
                let source=Arc::new(map(0.0));let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
                follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=2;
                follower.goal=Some(goal);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
                follower.odometry=Some(actual);
                let step=follower.follow_body_step(0.2);
                assert!(matches!(step,PlannerStep::Command(_)),"同层安全短弧接续应可控制: {step:?}");
                assert_eq!(follower.path_index,if fraction<0.5 {2}else{3},
                    "固定横差不应让微米级末采样永远无法消费，沿向中点之前仍须保留");
            }
        }
    }

    #[test]
    fn multi_point_partial_body_path_consumes_its_safe_end_and_requests_replan() {
        let pose=|x:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,1.0,0.4),orientation:Quaternion::IDENTITY}};
        for offset in [0.01,0.0] {
            let source=Arc::new(map(0.0));
            let mut config=SurfaceFollowerConfig::new(0.3);config.controller.goal_tolerance=0.35;
            let mut follower=SurfaceFollower::new(config).unwrap();
            let path=Path3D {poses:vec![pose(1.0),pose(1.5)]};
            follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
            follower.goal=Some(pose(2.5));follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
            follower.odometry=Some(pose(1.5-offset));
            let step=follower.follow_body_step(0.2);
            assert!(matches!(step,PlannerStep::ReplanRequired(_)),
                "到达远离原目标的安全截断末点后应请求后续路线: {step:?}");
            assert_eq!(follower.path_index,2,"安全截断末点须按原路径点容差消费");
        }
    }

    #[test]
    fn goal_region_does_not_pin_a_passed_internal_sample_outside_the_original_ball() {
        let pose=|x:f64,z:f64,yaw:f64|PoseStamped {frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(x,1.0,z),orientation:Quaternion::from_yaw(yaw)}};
        for terminal_turn in [false,true] {
            let source=Arc::new(map(0.0));
            let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.3;
            config.controller.goal_tolerance=0.35;
            let mut follower=SurfaceFollower::new(config).unwrap();
            let actual=pose(0.6507,0.4,0.0);let goal=pose(1.0,0.43,0.3);
            assert!(actual.pose.position.distance(goal.pose.position)>0.35);
            let mut poses=vec![pose(0.6,0.4,0.0),pose(0.6505,0.4,0.0),pose(0.657,0.4,0.0)];
            if terminal_turn {poses.push(pose(0.657,0.4,0.3));}
            let path=Path3D {poses};
            follower.cumulative=cumulative_lengths(&path);follower.path=Some(path);follower.path_index=1;
            follower.goal=Some(goal);follower.map=Some(Arc::clone(&source));follower.source_map=Some(source);
            follower.odometry=Some(actual);
            let step=follower.follow_body_step(0.2);
            let PlannerStep::Command(command)=step else {panic!("原球外须继续向真实末点前进: {step:?}");};
            assert_eq!(follower.path_index,2,"不能把已越过的内部采样误当作终点锁住；真实末点仍不能提前消费");
            assert!(command.linear.x>0.0 && command.linear.y==0.0);
        }
    }

    #[test]
    fn straight_position_path_uses_translation_tangent_without_erasing_turns() {
        let radius=0.34;
        for heading in [std::f64::consts::FRAC_PI_2,-std::f64::consts::FRAC_PI_2,std::f64::consts::PI] {
            let stored=0.37;
            let a=PlanarPose{x:0.0,y:0.0,yaw:stored};
            let b=PlanarPose{x:heading.cos(),y:heading.sin(),yaw:stored};
            let on_path=PlanarPose{x:b.x*0.5,y:b.y*0.5,yaw:heading};
            let wrong_direction=PlanarPose{yaw:stored,..on_path};
            let aligned=tracking_error(&[on_path],&[(a,b)],radius).unwrap();
            assert!(aligned<1e-12,"位置直段应以实际前进切向评分，不能要求侧向保持旧存储朝向");
            assert!(aligned<tracking_error(&[wrong_direction],&[(a,b)],radius).unwrap());
        }
        let angle=0.4_f64;
        let a=PlanarPose{x:0.0,y:0.0,yaw:0.0};
        let b=PlanarPose{x:angle.sin(),y:1.0-angle.cos(),yaw:angle};
        let curved=PlanarPose{x:b.x*0.5,y:b.y*0.5,yaw:angle*0.5};
        assert!(tracking_error(&[curved],&[(a,b)],radius).unwrap()<1e-12,
            "真实曲线仍保留规划姿态的连续切向");
        assert!(tracking_error(&[a],&[(a,PlanarPose{yaw:angle,..a})],radius).is_none(),
            "同XY显式转向不能作为平移切向重写");
    }

    #[test]
    fn predicted_progress_uses_the_path_tangent_instead_of_the_carrot_end_heading() {
        // 冻结5600中的真实平面片段，隔离旧末朝向评分与实际路径切向的差别。
        let points=[
            (-7.7527007954791,-0.3190792591919361,-0.37554222841535706),
            (-7.741158813811954,-0.3238766614451577,-0.4123035278208432),
            (-7.7298009491655755,-0.32909502515981837,-0.4490648272263291),
            (-7.718642548751804,-0.3347272990691632,-0.4858261266318152),
            (-7.70769869025824,-0.3407658726140322,-0.5225874260373004),
            (-7.696984161474654,-0.3472025862265581,-0.5593487254427867),
            (-7.686513440311124,-0.3540287423557098,-0.5961100248482734)];
        let segments:Vec<_>=points.windows(2).map(|pair|{
            let pose=|(x,y,yaw)|PlanarPose{x,y,yaw};
            (pose(pair[0]),pose(pair[1]))
        }).collect();
        let pose=PlanarPose{x:-7.748640236441645,y:-0.32372501761207123,yaw:-0.48697013073055523};
        let current=Twist2{linear:Point2::new(0.10395200376906219,0.0),angular_z:0.5007321947613922};
        let preferred=Twist2{linear:Point2::new(0.06651956194942318,0.0),angular_z:0.27727242705926103};
        let target=PlanarPose{x:-7.707698690258239,y:-0.34076587261403235,yaw:-0.30185766004473485};
        let limits=MotionLimits{model:MotionModel::Differential,max_forward_mps:1.0,max_reverse_mps:0.0,
            max_lateral_mps:0.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
            angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true};
        let ordinary=choose_control_with_period_check(pose,current,preferred,target,&limits,0.05,0.025,0.34,
            false,|_|true,|_,_,_|true,|_,_,_|true).unwrap();
        let horizon=0.1+current.angular_z.abs()/limits.guaranteed_angular_braking_rps2;
        let tracking=choose_tracking_control_with_period_check(pose,current,preferred,target,&limits,0.05,0.025,0.34,
            false,horizon,Some(-2.9409),|candidate,preview,stopped|tracking_error(&preview,&segments,0.34).map(|error|TrackingScore {
                path_error_m:error,target_cost:(stopped.x-target.x).hypot(stopped.y-target.y)
                    +0.005*(candidate.angular_z-preferred.angular_z).abs()}),
            |_|true,|_,_,_|true,|_,_,_|true).unwrap();
        assert!(ordinary.command.angular_z>current.angular_z,"必须复现原评分增大转弯的前态");
        assert!(tracking.command.angular_z<current.angular_z,
            "短时实际进展应按当前路径切向刹角速度，而非追更远的停车末朝向");
        assert!(tracking.command.linear.x>=0.0 && tracking.command.linear.y==0.0);
    }

    fn map(slope: f64) -> SurfaceSnapshot {
        super::super::test_snapshot(slope,false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,
            max_step_up_m:0.2,max_step_down_m:0.2,max_cross_slope_deg:20.0,
            support_uncertainty_m:0.0,
        }).unwrap()
    }

    #[test]
    fn successful_lookahead_reuse_preserves_control_and_only_removes_exact_duplicate() {
        use nuwax_nav_types_messages::{MotionModel,NavFrameId,Timestamp};
        let pose=|x:f64,y:f64,yaw:f64|PoseStamped {frame_id:NavFrameId::Map,
            stamp:Timestamp::from_unix(1,0),pose:Pose3 {
                position:Point3::new(x,y,0.4),orientation:Quaternion::from_yaw(yaw)}};
        let clear_source=Arc::new(map(0.0));
        let blocked_source=Arc::new(super::super::test_snapshot(0.0,true)
            .with_body_envelope(clear_source.body_envelope().unwrap().clone()).unwrap());
        let mut saved=0;
        for model in [MotionModel::Differential,MotionModel::Omnidirectional,MotionModel::Ackermann] {
            for (x,turn,aligned,restricted,blocked) in [(0.75,false,false,false,false),(1.04,false,false,false,false),
                (0.75,true,false,false,false),(0.75,false,true,false,false),(0.75,false,false,true,false),
                (0.75,false,false,false,true)] {
                let source=if blocked {&blocked_source}else{&clear_source};
                let path=Path3D {poses:if turn {
                    vec![pose(0.75,1.05,0.0),pose(0.95,1.05,0.0),pose(0.95,1.05,0.3),pose(1.55,1.2,0.3)]
                } else {vec![pose(0.75,1.05,0.0),pose(0.95,1.05,0.0),pose(1.15,1.05,0.0),pose(1.35,1.05,0.0),pose(1.55,1.05,0.0)]}};
                let make=|| {
                    let mut config=SurfaceFollowerConfig::new(0.3);config.controller.speed=0.3;
                    config.controller.control_frequency=20.0;
                    config.motion_limits=Some(MotionLimits {model,max_forward_mps:0.3,max_reverse_mps:0.0,
                        max_lateral_mps:if model==MotionModel::Omnidirectional{0.3}else{0.0},max_angular_rps:0.6,
                        linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
                        angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
                        minimum_turning_radius_m:if model==MotionModel::Ackermann{0.5}else{0.0},
                        can_rotate_in_place:model!=MotionModel::Ackermann});
                    let mut follower=SurfaceFollower::new(config).unwrap();
                    follower.cumulative=cumulative_lengths(&path);follower.path=Some(path.clone());follower.path_index=1;
                    follower.goal=path.poses.last().cloned();follower.map=Some(source.clone());follower.source_map=Some(source.clone());
                    follower.odometry=Some(pose(x,1.05,0.0));
                    follower.last_command=Twist2{linear:Point2::new(0.1,0.0),angular_z:0.0};
                    if aligned {follower.alignment_target_index=Some(2);}
                    if restricted {follower.boundary=vec![[0.5,0.7],[0.85,0.7],[0.85,1.4],[0.5,1.4]];}
                    follower
                };
                let mut reference=make();let mut reused=make();
                crate::control_profile::begin();
                // Explicit legacy comparison; the proposed default must not silently
                // turn this historical exact-arc-reuse test into a different policy.
                let old=reference.follow_body_step_policy::<false,false,false,false>(0.2);
                let old_profile=crate::control_profile::end();
                crate::control_profile::begin();
                let new=reused.follow_body_step_policy::<true,false,false,false>(0.2);
                let new_profile=crate::control_profile::end();
                assert_eq!(old,new,"{model:?} x={x} turn={turn} aligned={aligned} restricted={restricted} blocked={blocked}");
                assert_eq!(reference.path_index,reused.path_index);
                assert_eq!(reference.progress_m,reused.progress_m);
                assert_eq!(reference.control_target,reused.control_target);
                assert_eq!(reference.alignment_target_index,reused.alignment_target_index);
                assert_eq!(reference.last_command,reused.last_command);
                let stage=crate::control_profile::Stage::ForwardArcConnector as usize;
                let before=old_profile.calls[stage];let after=new_profile.calls[stage];
                assert!(after<=before&&before-after<=1);
                if aligned||turn||blocked {assert_eq!(before,after,"no successful lookahead probe to reuse");}
                if blocked {assert!(before>0,"missing-ground case must exercise the required fallback probe");}
                for counter in [crate::control_profile::Counter::CandidateEligibility,
                    crate::control_profile::Counter::TrackingScores,crate::control_profile::Counter::MotionMemoHits,
                    crate::control_profile::Counter::MotionMemoMisses] {
                    assert_eq!(old_profile.counters[counter as usize],new_profile.counters[counter as usize],
                        "selected-body guard work cannot be skipped");
                }
                if !turn&&!aligned&&!restricted&&!blocked {
                    assert!(matches!(new,PlannerStep::Command(c)if c.linear.x>0.0));
                    assert!(old_profile.counters[crate::control_profile::Counter::MotionMemoMisses as usize]>0,
                        "positive case must execute physical stopping checks");
                    assert_eq!(before-after,1);saved+=1;
                }
            }
        }
        assert!(saved>=6);
    }

    #[test]
    fn memo_success_dependencies_survive_selection_reset() {
        let anchor=ObservationClockAnchor {monotonic:std::time::Instant::now(),unix_ns:100};
        let proof=ObservationProofAccumulator::new(Some(anchor));
        let cache=RefCell::new(HashMap::new());
        let from=([1.0,1.0,0.4],[0.0,0.0,0.0,1.0]);
        let selected=PlanarPose{x:1.1,y:1.0,yaw:0.0};
        let current_stop=PlanarPose{x:1.2,..selected};
        for (target,latest) in [(selected,200),(current_stop,180)] {
            memoized_body_motion(&cache,&proof,from,target,false,Some(120),||
                Ok(BodyMotionSuccess {pose:from,latest_start_ns:Some(latest)})).unwrap();
        }
        proof.merge(Some(101)); // An unrelated successful prefix of a rejected candidate.
        proof.reset();
        assert_eq!(proof.bound().unwrap(),None);
        memoized_body_motion(&cache,&proof,from,selected,false,Some(120),||
            panic!("selected proof must come from the exact memo entry")).unwrap();
        assert_eq!(proof.bound().unwrap(),Some(crate::CommandDispatchBound::from_anchor(anchor,200)));
        memoized_body_motion(&cache,&proof,from,current_stop,false,Some(120),||
            panic!("current-stop proof must also merge its memo dependency")).unwrap();
        assert_eq!(proof.bound().unwrap(),Some(crate::CommandDispatchBound::from_anchor(anchor,180)));
        proof.reset();
        let error=PlanarPose{x:1.3,..selected};
        assert!(memoized_body_motion(&cache,&proof,from,error,false,Some(120),||
            Err(nuwax_nav_mls_planner::surface::SurfaceError::MissingSupport)).is_err());
        assert!(memoized_body_motion(&cache,&proof,from,error,false,Some(120),||
            panic!("failed full queries stay failed and grant no dependency")).is_err());
        assert_eq!(proof.bound().unwrap(),None);
        memoized_body_motion(&cache,&proof,from,selected,false,Some(121),||
            Ok(BodyMotionSuccess {pose:from,latest_start_ns:None})).unwrap();
        assert_eq!(proof.bound().unwrap(),None,"no finite constraint must remain unbounded");
    }

    #[test]
    fn observed_dependencies_are_restored_from_each_callback_memo() {
        let base=map(0.0);
        let observed_ns=crate::command_dispatch::unix_ns().unwrap();
        let mut blocks:BTreeMap<_,BTreeMap<_,_>>=BTreeMap::new();
        for (x,y,z) in base.cell_keys() {
            let key=[x,y,z];
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
                .insert(key,SurfaceObservation {observed_ns,hits:2});
        }
        let mut observations=SurfaceObservations::new(base.source_voxel_size_m(),32).unwrap();
        for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
        let source=Arc::new(base.with_observations(Some(Arc::new(observations))));
        let start=PlanarPose{x:1.05,y:1.05,yaw:0.0};
        let target=PlanarPose{x:1.45,..start};
        let mut config=SurfaceFollowerConfig::new(0.3);config.max_observation_age_sec=Some(1.0);
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.map=Some(source.clone());follower.source_map=Some(source);
        follower.odometry=Some(PoseStamped {
            frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(start.x,start.y,0.4),orientation:Quaternion::IDENTITY},
        });
        follower.with_body_motion_checks(|clear,period,_,proof| {
            assert!(clear(start,start,0.0));assert!(clear(start,target,0.2));
            let original=proof.bound().unwrap();
            assert!(original.is_some(),"motion beyond the original footprint needs finite evidence");
            proof.reset();
            assert!(period(start,start,0.0));assert!(period(start,target,0.2));
            assert_eq!(proof.bound().unwrap(),original);
            for is_period in [false,true] {
                let mut check=|a,b,t|if is_period {period(a,b,t)}else{clear(a,b,t)};
                proof.reset();
                assert!(check(start,start,0.0));
                assert_eq!(proof.bound().unwrap(),None,"original-footprint exemption has no finite constraint");
                assert!(check(start,target,0.2));
                assert_eq!(proof.bound().unwrap(),original,"each memo restores the full-query dependency");
            }
        }).unwrap();
    }

    #[test]
    fn selected_replay_matches_fresh_current_stop_trace_without_new_geometry_queries() {
        use crate::control_profile::Counter;
        let start=PlanarPose{x:1.05,y:1.05,yaw:0.0};
        let limits=MotionLimits {
            model:MotionModel::Differential,max_forward_mps:1.0,max_reverse_mps:0.0,
            max_lateral_mps:0.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,
            guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        };
        let current=Twist2{linear:Point2::new(0.35,0.0),angular_z:0.0};
        let preferred=Twist2{linear:Point2::new(0.8,0.0),angular_z:0.2};
        let source=Arc::new(map(0.0));
        let step=f64::from(source.source_voxel_size_m())*0.5;
        let radius=source.body_envelope().unwrap().radius();
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        follower.map=Some(source.clone());follower.source_map=Some(source);
        follower.odometry=Some(PoseStamped {
            frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(start.x,start.y,0.4),orientation:Quaternion::IDENTITY},
        });
        follower.last_command=current;
        let (selected,replayed,trace,record)=follower.with_body_motion_checks(|clear,period,_,proof| {
            let trace=RefCell::new(Vec::new());
            let mut geometric=|a,b,t| {let ok=clear(a,b,t);trace.borrow_mut().push((0,a,b,t,ok));ok};
            let mut physical=|a,b,t| {let ok=period(a,b,t);trace.borrow_mut().push((1,a,b,t,ok));ok};
            let selected=choose_tracking_control_with_period_check(start,current,preferred,
                PlanarPose{x:3.0,y:1.05,yaw:0.0},&limits,0.05,step,radius,false,
                0.5,None,|_,_,_|None,|_|true,&mut geometric,&mut physical).unwrap();
            assert_ne!(selected.command,Twist2::ZERO);
            trace.borrow_mut().clear();
            proof.reset();
            crate::control_profile::begin();
            let replayed=nuwax_nav_control::check_stopping_trajectory_with_period_check(
                start,current,selected.command,&limits,0.05,step,radius,&mut geometric,&mut physical).unwrap();
            let record=crate::control_profile::end();
            assert_eq!(proof.bound().unwrap(),None);
            drop(physical);drop(geometric);
            (selected,replayed,trace.into_inner(),record)
        }).unwrap();
        let (fresh,fresh_trace)=follower.with_body_motion_checks(|clear,period,_,_| {
            let trace=RefCell::new(Vec::new());
            let result=nuwax_nav_control::check_stopping_trajectory_with_period_check(
                start,current,selected.command,&limits,0.05,step,radius,
                |a,b,t| {let ok=clear(a,b,t);trace.borrow_mut().push((0,a,b,t,ok));ok},
                |a,b,t| {let ok=period(a,b,t);trace.borrow_mut().push((1,a,b,t,ok));ok}).unwrap();
            (result,trace.into_inner())
        }).unwrap();
        assert_eq!(trace,fresh_trace,"replay must include the fresh standalone current-stop trace");
        assert_eq!(replayed.command,selected.command);
        assert_eq!(replayed.command,fresh.command);
        assert_eq!(replayed.stopped_pose,fresh.stopped_pose);
        assert_eq!(replayed.samples,fresh.samples);
        assert!(trace.iter().any(|(basis,_,_,_,_)|*basis==1),"both callbacks are necessary");
        assert!(record.counters[Counter::MotionMemoHits as usize]>0);
        for counter in [Counter::MotionMemoMisses,Counter::BodyProjectionCalls,Counter::GeometrySweeps,
            Counter::ObservedSweeps,Counter::FutureGeometrySweeps,Counter::FutureViewBuilds] {
            assert_eq!(record.counters[counter as usize],0,"selected replay should reuse exact queries: {counter:?}");
        }
    }

    #[test]
    fn observed_selected_replay_excludes_rejected_prefix_and_keeps_current_stop_deadline() {
        use crate::control_profile::Counter;
        let base=map(0.0);
        let size=f64::from(base.source_voxel_size_m());
        // At .35 m/s the immediate stop travels .07875 m. Braking to .30
        // has candidate-only tails of at most .07625 m. Put cell x=12 in
        // that real 2.5 mm difference, outside the original occupied set.
        let old_cell_edge=12.0*size;
        let start=PlanarPose{x:old_cell_edge-0.1775,y:1.05,yaw:0.0};
        let observed_ns=crate::command_dispatch::unix_ns().unwrap();
        let older_ns=observed_ns-200_000_000;
        let max_age_ns=1_000_000_000;
        let mut blocks:BTreeMap<_,BTreeMap<_,_>>=BTreeMap::new();
        for (x,y,z) in base.cell_keys() {
            let key=[x,y,z];
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation {observed_ns:if x==12 {older_ns}else{observed_ns},hits:2});
        }
        let mut observations=SurfaceObservations::new(base.source_voxel_size_m(),32).unwrap();
        for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
        let source=Arc::new(base.with_observations(Some(Arc::new(observations))));
        let initial_body=([start.x,start.y,0.4],[0.0,0.0,0.0,1.0]);
        let occupied=source.body_observation_footprint(initial_body).unwrap();
        assert!(!occupied.iter().any(|cell|cell[0]==12));
        let limits=MotionLimits {
            model:MotionModel::Differential,max_forward_mps:1.0,max_reverse_mps:0.0,
            max_lateral_mps:0.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,
            guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
            guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true,
        };
        let current=Twist2{linear:Point2::new(0.35,0.0),angular_z:0.0};
        let preferred=Twist2{linear:Point2::new(0.8,0.0),angular_z:0.0};
        let step=size*0.5;
        let radius=source.body_envelope().unwrap().radius();
        let mut config=SurfaceFollowerConfig::new(0.3);
        config.max_observation_age_sec=Some(1.0);
        let mut follower=SurfaceFollower::new(config).unwrap();
        follower.map=Some(source.clone());follower.source_map=Some(source.clone());
        follower.odometry=Some(PoseStamped {
            frame_id:nuwax_nav_types_messages::NavFrameId::Map,
            stamp:nuwax_nav_types_messages::Timestamp::from_unix(1,0),
            pose:Pose3 {position:Point3::new(start.x,start.y,0.4),orientation:Quaternion::IDENTITY},
        });
        follower.last_command=current;
        // This existing task-boundary check applies to the support reference,
        // not the forward footprint edge. The .40 proposal's actual-period
        // tail travels .11875 m; its instantaneous-response tail travels .12 m.
        // The latter fails after successful tighter observation dependencies.
        let right=start.x+0.119375;
        follower.boundary=vec![[0.0,0.0],[right,0.0],[right,3.0],[0.0,3.0]];
        let before=std::time::Instant::now();
        let (selected,replayed,rank_bound,selected_bound,trace,record)=
            follower.with_body_motion_checks(|clear,period,last_failure,proof| {
                let trace=RefCell::new(Vec::new());
                let candidates=RefCell::new(Vec::new());
                let mut geometric=|a,b,t| {let ok=clear(a,b,t);trace.borrow_mut().push((0,a,b,t,ok));ok};
                let mut physical=|a,b,t| {let ok=period(a,b,t);trace.borrow_mut().push((1,a,b,t,ok));ok};
                let selected=choose_tracking_control_with_period_check(start,current,preferred,
                    PlanarPose{x:3.0,y:start.y,yaw:0.0},&limits,0.05,step,radius,false,
                    0.5,None,|candidate,_,_| {
                        let cost=if candidate.angular_z==0.0 && candidate.linear.x>0.399 {0.0}
                            else if candidate.angular_z==0.0 && candidate.linear.x<0.301 {1.0}
                            else {2.0};
                        Some(TrackingScore {path_error_m:cost,target_cost:cost})
                    },|candidate| {candidates.borrow_mut().push(candidate);true},
                    &mut geometric,&mut physical).unwrap();
                let candidates=candidates.into_inner();
                let failures:Vec<_>=trace.borrow().iter().filter(|entry|!entry.4).copied().take(8).collect();
                eprintln!("OBSERVED_SELECTED_RANK selected={:?} candidates={candidates:?} failures={failures:?} last_failure={:?} support_boundary_x={right:.15} old_cell_edge={old_cell_edge:.15}",
                    selected.command,last_failure.borrow());
                assert!((selected.command.linear.x-0.30).abs()<1e-12,"selected={:?}",selected.command);
                assert_eq!(selected.command.angular_z,0.0);
                assert!(candidates.len()>=2,"the tighter proposal must actually reach the guard");
                assert!(candidates[0].linear.x>0.399 && candidates[0].angular_z==0.0);
                assert!(trace.borrow().iter().any(|(_,_,_,_,ok)|!*ok),"ranking must reject a real checked proposal");
                let rank_bound=proof.bound().unwrap().unwrap();
                trace.borrow_mut().clear();
                proof.reset();
                crate::control_profile::begin();
                let replayed=nuwax_nav_control::check_stopping_trajectory_with_period_check(
                    start,current,selected.command,&limits,0.05,step,radius,
                    &mut geometric,&mut physical).unwrap();
                let record=crate::control_profile::end();
                let selected_bound=proof.bound().unwrap().unwrap();
                drop(physical);drop(geometric);
                (selected,replayed,rank_bound,selected_bound,trace.into_inner(),record)
            }).unwrap();
        let (fresh,fresh_bound,fresh_trace)=follower.with_body_motion_checks(|clear,period,_,proof| {
            let trace=RefCell::new(Vec::new());
            let fresh=nuwax_nav_control::check_stopping_trajectory_with_period_check(
                start,current,selected.command,&limits,0.05,step,radius,
                |a,b,t| {let ok=clear(a,b,t);trace.borrow_mut().push((0,a,b,t,ok));ok},
                |a,b,t| {let ok=period(a,b,t);trace.borrow_mut().push((1,a,b,t,ok));ok}).unwrap();
            (fresh,proof.bound().unwrap().unwrap(),trace.into_inner())
        }).unwrap();
        assert_eq!(trace,fresh_trace,"fresh original callbacks must check the same complete selected proof");
        assert_eq!(replayed.command,fresh.command);
        assert_eq!(replayed.stopped_pose,fresh.stopped_pose);
        assert_eq!(replayed.samples,fresh.samples);
        // Independently identify which endpoints need evidence beyond the
        // original footprint. The first geometric trajectory is current-stop;
        // every later zero-time reset begins a selected-command tail.
        let mut geometric_starts=0;
        let mut current_stop_latest_elapsed=None;
        let mut candidate_queries=0;
        for &(basis,a,b,elapsed,ok) in &trace {
            assert!(ok);
            if basis==0 && elapsed==0.0 {geometric_starts+=1;}
            let is_current_stop=basis==0 && geometric_starts==1;
            for p in [a,b] {
                let pose=([p.x,p.y,0.4],[0.0,0.0,(p.yaw*0.5).sin(),(p.yaw*0.5).cos()]);
                let cells=source.body_observation_footprint(pose).unwrap();
                let outside:Vec<_>=cells.difference(&occupied).copied().collect();
                if is_current_stop {
                    assert!(outside.iter().all(|cell|cell[0]==12));
                    if !outside.is_empty() {
                        current_stop_latest_elapsed=Some(current_stop_latest_elapsed
                            .map_or((elapsed*1e9)as u64,|old:u64|old.max((elapsed*1e9)as u64)));
                    }
                } else {
                    assert!(outside.is_empty(),"selected-command tails must not visit the older current-stop-only strip");
                    candidate_queries+=1;
                }
            }
        }
        assert!(candidate_queries>0 && geometric_starts>=4);
        let current_stop_latest_elapsed=current_stop_latest_elapsed.expect("real current-stop must need the old non-exempt cell");
        let expected=older_ns+max_age_ns-current_stop_latest_elapsed;
        eprintln!("OBSERVED_SELECTED_BOUND current_stop_elapsed_ns={current_stop_latest_elapsed} expected_latest_unix_ns={expected} ranking={rank_bound:?} selected={selected_bound:?} fresh={fresh_bound:?} replay_queries={:?}",record.counters);
        // Different invocations capture different clock pairs. Their absolute
        // Unix constraints must nevertheless agree to the nanosecond. The
        // earlier monotonic sample isolates the inclusive Unix qualification.
        for bound in [selected_bound,fresh_bound] {
            assert!(!bound.expired_at(before,Some(expected)),"exact selected/current-stop deadline remains inclusive");
            assert!(bound.expired_at(before,Some(expected+1)),"the next nanosecond must expire");
        }
        assert!(rank_bound.expired_at(before,Some(expected)),
            "rejected proposal's tighter successful-prefix dependency must be absent after selection reset");
        assert!(record.counters[Counter::MotionMemoHits as usize]>0);
        for counter in [Counter::MotionMemoMisses,Counter::BodyProjectionCalls,Counter::GeometrySweeps,
            Counter::ObservedSweeps,Counter::FutureGeometrySweeps,Counter::FutureViewBuilds] {
            assert_eq!(record.counters[counter as usize],0,"observed selected replay reuses exact queries: {counter:?}");
        }
    }

    #[test]
    fn body_witness_current_pose_reaches_final_controller_failure() {
        use nuwax_nav_types_messages::{NavFrameId,Timestamp};
        let source=Arc::new(map(0.0));
        let pose=PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,0),
            pose:Pose3{position:Point3::new(100.0,100.0,0.4),orientation:Quaternion::from_yaw(0.0)}};
        let mut follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
        follower.map=Some(source.clone());follower.source_map=Some(source.clone());
        follower.odometry=Some(pose.clone());follower.goal=Some(pose.clone());
        follower.path=Some(Path3D{poses:vec![pose.clone()]});
        // Fill the old API first: the final message must still recover evidence on a warm hit.
        let old=source.validate_body_pose([100.0,100.0,0.4],[0.0,0.0,0.0,1.0]);
        assert!(old.is_err());
        let step=follower.follow_body_step_policy::<true,false,false,false>(0.2);
        let PlannerStep::ReplanRequired(message)=step else {panic!("expected current-body rejection: {step:?}")};
        assert!(message.contains("CentreSupport"));assert_eq!(message.matches("拒绝样本").count(),1);
        assert!(message.contains("100.0"));
        let current=follower.current_body_error().unwrap();
        assert!(current.contains("CentreSupport"));assert_eq!(current.matches("拒绝样本").count(),1);
        eprintln!("FINAL_CONTROLLER_FAILURE {message}\nFINAL_CURRENT_BODY_FAILURE {current}");
    }

    #[test]
    fn body_witness_controller_memo_size_and_final_message() {
        use nuwax_nav_mls_planner::surface::{BodyFailureWitness,BodyRejection};
        eprintln!("CONTROLLER_CACHE old_value={} new_value={} key={} old_entry={} new_entry={}",
            std::mem::size_of::<BodyMotionResult>(),std::mem::size_of::<Result<BodyMotionSuccess,BodyMotionFailure>>(),
            std::mem::size_of::<BodyMotionQuery>(),std::mem::size_of::<(BodyMotionQuery,BodyMotionResult)>(),
            std::mem::size_of::<(BodyMotionQuery,Result<BodyMotionSuccess,BodyMotionFailure>)>());
        let witness=BodyFailureWitness{pose:([0.05,0.05,0.4],[0.0,0.0,0.0,1.0]),
            rejection:BodyRejection::CoreSupport{column:[1,0]}};
        let message=BodyMotionFailure::Body(witness).to_string();
        assert!(message.contains("no observed support"));assert!(message.contains("拒绝样本"));
        assert!(message.contains("NotChecked"));
    }

    #[test]
    fn exact_query_reuse_preserves_both_stopping_callback_traces() {
        for slope in [0.0,0.08] {
            let map=map(slope);
            let start=PlanarPose{x:1.05,y:1.05,yaw:0.0};
            let body=([start.x,start.y,0.4+slope*start.x],[0.0,0.0,0.0,1.0]);
            let limits=MotionLimits {
                model:MotionModel::Omnidirectional,max_forward_mps:1.0,max_reverse_mps:1.0,
                max_lateral_mps:1.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,
                guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
                guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
                minimum_turning_radius_m:0.0,can_rotate_in_place:true,
            };
            let current=Twist2{linear:Point2::new(0.35,0.04),angular_z:0.0};
            let preferred=Twist2{linear:Point2::new(0.8,0.0),angular_z:0.2};
            let run=|reuse| {
                let queries=Cell::new(0);
                let cache=RefCell::new(HashMap::new());
                let proof=ObservationProofAccumulator::new(None);
                let trace=RefCell::new(Vec::new());
                let make_check=|basis| {
                    let mut previous=body;let mut previous_planar=start;
                    let queries=&queries;let cache=&cache;let proof=&proof;let trace=&trace;let map=&map;
                    move |from:PlanarPose,to:PlanarPose,elapsed:f64| {
                        if elapsed==0.0 || (from.x-previous_planar.x).hypot(from.y-previous_planar.y)>1e-5
                            || angle_diff(from.yaw,previous_planar.yaw).abs()>1e-5 {previous=body;}
                        let a=previous;let preserve=(to.x-from.x).hypot(to.y-from.y)<1e-9;
                        let check=|| {
                            queries.set(queries.get()+1);
                            let mut b=map.body_pose_after_motion(a,[to.x,to.y],to.yaw)?;
                            if preserve {b.0[2]=a.0[2];}
                            map.validate_body_motion(a,b,&[])?;Ok(BodyMotionSuccess {pose:b,latest_start_ns:None})
                        };
                        let result=if reuse {memoized_body_motion(cache,proof,a,to,preserve,None,check)}else{check()};
                        trace.borrow_mut().push((basis,from,to,elapsed,a,result.clone()));
                        match result {
                            Ok(success)=>{previous=success.pose;previous_planar=to;true},
                            Err(_)=>false,
                        }
                    }
                };
                let result=choose_control_with_period_check(start,current,preferred,
                    PlanarPose{x:3.0,y:1.05,yaw:0.0},&limits,0.05,0.025,0.3,true,
                    |_|true,make_check(0),make_check(1));
                (result.map(|r|(r.command,r.stopped_pose,r.samples)),trace.into_inner(),queries.get())
            };
            let raw=run(false);let reused=run(true);
            assert!(raw.0.is_some(),"合成实际站姿必须能推进，坡度{slope}");
            assert_eq!(raw.0,reused.0,"同输入控制结果和样本数量不能变化");
            assert_eq!(raw.1,reused.1,"两个完整回调的起点、时间和结果逐项一致");
            assert!(reused.2<raw.2,"应消除已证明完全相同的实际查询");
            eprintln!("坡度{slope}：{}个回调，实际查询{}→{}",raw.1.len(),raw.2,reused.2);
        }
    }

    #[test]
    fn exact_query_reuse_keeps_real_height_and_observation_expiry() {
        let base=map(0.0);
        let mut blocks:BTreeMap<_,BTreeMap<_,_>>=BTreeMap::new();
        for (x,y,z) in base.cell_keys() {
            let key=[x,y,z];
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
                .insert(key,SurfaceObservation{observed_ns:10_000_000_000,hits:2});
        }
        let mut observations=SurfaceObservations::new(base.source_voxel_size_m(),32).unwrap();
        for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
        let map=base.with_observations(Some(Arc::new(observations)));
        let from=([1.05,1.05,0.4],[0.0,0.0,0.0,1.0]);
        let target=PlanarPose{x:1.45,y:1.05,yaw:0.0};
        let occupied=map.body_observation_footprint(from).unwrap();
        let cache=RefCell::new(HashMap::new());
        let proof=ObservationProofAccumulator::new(None);
        let query=|start:BodyPose,time| memoized_body_motion(&cache,&proof,start,target,false,Some(time),|| {
            let to=map.body_pose_after_motion(start,[target.x,target.y],target.yaw)?;
            map.validate_observed_body_motion(start,to,&occupied,time,1_000_000_000)?;
            Ok(BodyMotionSuccess {pose:to,latest_start_ns:None})
        });
        let fresh=query(from,10_100_000_000).expect("真实新鲜地面应允许前进");
        assert_eq!(query(from,10_100_000_000).unwrap(),fresh);
        let lower=([from.0[0],from.0[1],from.0[2]-0.01],from.1);
        let lower_result=query(lower,10_100_000_000).unwrap();
        assert_ne!(lower_result,fresh,"同平面起点的不同实际高度不得合并");
        let tilted=(from.0,[0.0,(0.01_f64/2.0).sin(),0.0,(0.01_f64/2.0).cos()]);
        assert_ne!(query(tilted,10_100_000_000).unwrap(),fresh,"实际倾斜不能由缓存清零");
        assert!(query(from,12_000_000_000).is_err(),"同一扫掠的观察在停车末期过期，不能复用早期许可");
        assert_eq!(query(from,10_100_000_000).unwrap(),fresh,"过期拒绝也不能污染原时刻结果");
    }
}

#[cfg(test)]
#[path = "surface_body_control_doorway_tests.rs"]
mod dense_doorway_checkpoint_diagnostic;
