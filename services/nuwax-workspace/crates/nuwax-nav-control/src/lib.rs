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
    let x = axis(
        current.linear.x,
        x,
        limits.linear_acceleration_mps2,
        limits.guaranteed_braking_mps2,
    );
    let y = axis(
        current.linear.y,
        y,
        limits.linear_acceleration_mps2,
        limits.guaranteed_braking_mps2,
    );
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

/// 验证响应延迟、一个控制周期和停止尾段。回调须检查相邻样本间完整机体扫掠。
/// 最多1024段、20秒；超过预算返回不可行，不截短制动尾段。
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
    // 目标速度立即生效是加速时更保守的边界；当前速度另有独立停车检验。
    // 不把最大响应延迟解释为每次都恰好延迟，从而低估快速响应执行器的位移。
    if command != Twist2::ZERO {
        stopping_from_velocity(
            pose,
            current,
            limits.reaction_time_sec,
            limits,
            spatial_step_m,
            body_radius_m,
            &mut clear,
        )?;
        let mut choice = stopping_from_velocity(
            pose,
            command,
            control_dt + limits.reaction_time_sec,
            limits,
            spatial_step_m,
            body_radius_m,
            clear,
        )?;
        choice.command = command;
        Some(choice)
    } else {
        let mut choice = stopping_from_velocity(
            pose,
            current,
            limits.reaction_time_sec,
            limits,
            spatial_step_m,
            body_radius_m,
            clear,
        )?;
        choice.command = Twist2::ZERO;
        Some(choice)
    }
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
    mut clear: impl FnMut(PlanarPose, PlanarPose, f64) -> bool,
) -> Option<ControlChoice> {
    if limits.validate().is_err()
        || !finite(current)
        || !finite(preferred)
        || ![target.x, target.y, target.yaw]
            .into_iter()
            .all(f64::is_finite)
    {
        return None;
    }
    if preferred == Twist2::ZERO {
        return check_stopping_trajectory(
            pose,
            current,
            Twist2::ZERO,
            limits,
            control_dt,
            spatial_step_m,
            body_radius_m,
            clear,
        );
    }
    let mut velocities = Vec::new();
    for scale in [1.0, 0.6, 0.3, 0.0] {
        for w in [
            preferred.angular_z,
            0.0,
            preferred.angular_z - limits.angular_acceleration_rps2 * control_dt,
            preferred.angular_z + limits.angular_acceleration_rps2 * control_dt,
            current.angular_z,
        ] {
            let mut candidate = reachable(
                Twist2 {
                    linear: Point2::new(preferred.linear.x * scale, preferred.linear.y * scale),
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
            if allowed(candidate, limits, allow_reverse)
                && !velocities.iter().any(|old: &Twist2| old == &candidate)
            {
                velocities.push(candidate);
            }
        }
    }
    let mut best = None;
    let mut best_score = f64::INFINITY;
    for velocity in velocities {
        if let Some(choice) = check_stopping_trajectory(
            pose,
            current,
            velocity,
            limits,
            control_dt,
            spatial_step_m,
            body_radius_m,
            &mut clear,
        ) {
            let future = integrate(pose, velocity, control_dt.max(0.2));
            let score = (future.x - target.x).hypot(future.y - target.y)
                + 0.1 * angle(future.yaw - target.yaw).abs()
                + 0.005 * (velocity.angular_z - preferred.angular_z).abs();
            if score < best_score {
                best_score = score;
                best = Some(choice);
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
