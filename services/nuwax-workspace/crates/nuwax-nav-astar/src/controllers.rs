//! P / PD 角速度控制器
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/controllers.py`。
//!
//! 行为契约：
//! - `Controller` trait：`advance(lookahead, odom) -> Twist`、`rotate(yaw_error) -> Twist`、
//!   `reset_errors()`、`reset_yaw_error(value)`。
//! - `PController`：纯比例控制器。`angular_velocity = k_angular * yaw_error`，限幅 ±speed，
//!   最小角速度 0.2（保留符号）。当 |yaw_error| > 90° 时只旋转不前进（rotate-then-drive）。
//! - `PdController`：在 P 之上加微分阻尼，并限制角加速度。

use crate::helpers::{make_twist2, make_twist2_zero, pose_xy_yaw, PoseStamped, Twist2};
use std::f64::consts::PI;

/// 控制器 trait
pub trait Controller {
    /// 沿 lookahead 方向前进，输出 twist
    fn advance(&mut self, lookahead_point: (f64, f64), current_odom: PoseStamped) -> Twist2;
    /// 原地旋转
    fn rotate(&mut self, yaw_error: f64) -> Twist2;
    /// 重置内部误差状态
    fn reset_errors(&mut self);
    /// 把内部 yaw 误差初始化为给定值（PD 用）
    fn reset_yaw_error(&mut self, value: f64);
}

/// wrap-around 角度差，结果 ∈ [-π, π]
#[inline]
#[must_use]
pub fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d
}

/// P 控制器
pub struct PController {
    speed: f64,
    control_frequency: f64,
    k_angular: f64,
    min_linear_velocity: f64,
    min_angular_velocity: f64,
    max_angular_accel: f64,
    rotation_threshold: f64,
    simulation: bool,
}

impl PController {
    /// 构造
    #[must_use]
    pub fn new(speed: f64, control_frequency: f64) -> Self {
        Self {
            speed,
            control_frequency,
            k_angular: 0.5,
            min_linear_velocity: 0.2,
            min_angular_velocity: 0.2,
            max_angular_accel: 2.0,
            rotation_threshold: 90.0 * (PI / 180.0),
            simulation: false,
        }
    }

    /// 启用仿真模式（角速度 < 0.8 时取 0.8 * sign）
    pub fn set_simulation(&mut self, on: bool) {
        self.simulation = on;
    }
}

impl Controller for PController {
    fn advance(&mut self, lookahead_point: (f64, f64), current_odom: PoseStamped) -> Twist2 {
        let (lx, ly) = lookahead_point;
        let (ox, oy, oyaw) = pose_xy_yaw(&current_odom);
        let dx = lx - ox;
        let dy = ly - oy;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist < 1e-6 {
            return make_twist2_zero();
        }
        let desired_yaw = dy.atan2(dx);
        let yaw_error = angle_diff(desired_yaw, oyaw);

        let angular_velocity = self.compute_angular_velocity(yaw_error);

        if yaw_error.abs() > self.rotation_threshold {
            return self.angular_twist(angular_velocity);
        }

        // 与 Python 同步：linear = speed * (1 - |yaw_error| / threshold)
        let mut linear = self.speed * (1.0 - yaw_error.abs() / self.rotation_threshold);
        linear = self.apply_min_velocity(linear, self.min_linear_velocity);

        make_twist2(linear, angular_velocity)
    }

    fn rotate(&mut self, yaw_error: f64) -> Twist2 {
        let angular_velocity = self.compute_angular_velocity(yaw_error);
        self.angular_twist(angular_velocity)
    }

    fn reset_errors(&mut self) {}

    fn reset_yaw_error(&mut self, _value: f64) {}
}

impl PController {
    fn compute_angular_velocity(&self, yaw_error: f64) -> f64 {
        let mut v = self.k_angular * yaw_error;
        v = v.clamp(-self.speed, self.speed);
        self.apply_min_velocity(v, self.min_angular_velocity)
    }

    fn apply_min_velocity(&self, velocity: f64, min_velocity: f64) -> f64 {
        if velocity == 0.0 {
            return 0.0;
        }
        if velocity.abs() < min_velocity {
            return if velocity > 0.0 {
                min_velocity
            } else {
                -min_velocity
            };
        }
        velocity
    }

    fn angular_twist(&self, mut angular_velocity: f64) -> Twist2 {
        if self.simulation && angular_velocity.abs() < 0.8 {
            angular_velocity = 0.8 * angular_velocity.signum();
        }
        make_twist2(0.0, angular_velocity)
    }
}

/// PD 控制器
pub struct PdController {
    base: PController,
    k_derivative: f64,
    prev_yaw_error: f64,
    prev_angular_velocity: f64,
}

impl PdController {
    /// 构造
    #[must_use]
    pub fn new(speed: f64, control_frequency: f64) -> Self {
        Self {
            base: PController::new(speed, control_frequency),
            k_derivative: 0.15,
            prev_yaw_error: 0.0,
            prev_angular_velocity: 0.0,
        }
    }
}

impl Controller for PdController {
    fn advance(&mut self, lookahead_point: (f64, f64), current_odom: PoseStamped) -> Twist2 {
        // advance 调用时使用 P 的 advance 行为，但 compute_angular_velocity 由本类覆盖
        // 为了保持 trait 一致，我们手动实现
        let (lx, ly) = lookahead_point;
        let (ox, oy, oyaw) = pose_xy_yaw(&current_odom);
        let dx = lx - ox;
        let dy = ly - oy;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist < 1e-6 {
            return make_twist2_zero();
        }
        let desired_yaw = dy.atan2(dx);
        let yaw_error = angle_diff(desired_yaw, oyaw);

        let angular_velocity = self.compute_pd_angular_velocity(yaw_error);

        if yaw_error.abs() > self.base.rotation_threshold {
            return self.base.angular_twist(angular_velocity);
        }

        let mut linear = self.base.speed * (1.0 - yaw_error.abs() / self.base.rotation_threshold);
        linear = self
            .base
            .apply_min_velocity(linear, self.base.min_linear_velocity);

        make_twist2(linear, angular_velocity)
    }

    fn rotate(&mut self, yaw_error: f64) -> Twist2 {
        let v = self.compute_pd_angular_velocity(yaw_error);
        self.base.angular_twist(v)
    }

    fn reset_errors(&mut self) {
        self.prev_yaw_error = 0.0;
        self.prev_angular_velocity = 0.0;
    }

    fn reset_yaw_error(&mut self, value: f64) {
        self.prev_yaw_error = value;
    }
}

impl PdController {
    fn compute_pd_angular_velocity(&mut self, yaw_error: f64) -> f64 {
        let dt = 1.0 / self.base.control_frequency;
        let yaw_error_derivative = (yaw_error - self.prev_yaw_error) / dt;
        let mut v = self.base.k_angular * yaw_error - self.k_derivative * yaw_error_derivative;
        let max_delta = self.base.max_angular_accel * dt;
        v = v.clamp(
            self.prev_angular_velocity - max_delta,
            self.prev_angular_velocity + max_delta,
        );
        v = v.clamp(-self.base.speed, self.base.speed);
        v = self
            .base
            .apply_min_velocity(v, self.base.min_angular_velocity);
        self.prev_yaw_error = yaw_error;
        self.prev_angular_velocity = v;
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::make_pose_stamped;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn angle_diff_wraps_around_pi() {
        assert!(approx_eq(angle_diff(0.0, 0.0), 0.0));
        let d = angle_diff(0.1, -0.1);
        assert!(approx_eq(d, 0.2));
        let d = angle_diff(3.5, -3.5); // 总差 7, 归一化
        assert!(d.abs() <= PI + 1e-6);
    }

    #[test]
    fn p_controller_advance_returns_zero_when_at_lookahead() {
        let mut c = PController::new(0.5, 10.0);
        let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
        let t = c.advance((0.0, 0.0), odom);
        assert_eq!(t.linear.x, 0.0);
        assert_eq!(t.angular_z, 0.0);
    }

    #[test]
    fn p_controller_advance_drives_forward_when_aligned() {
        let mut c = PController::new(0.5, 10.0);
        let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
        let t = c.advance((1.0, 0.0), odom);
        assert!(t.linear.x > 0.0);
        assert!(t.angular_z.abs() < 0.5);
    }

    #[test]
    fn p_controller_advance_rotates_when_misaligned() {
        let mut c = PController::new(0.5, 10.0);
        let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
        // lookahead 在正后方 (-1, 0)，yaw_error ~ π
        let t = c.advance((-1.0, 0.0), odom);
        assert_eq!(t.linear.x, 0.0, "should rotate in place");
        assert!(t.angular_z.abs() > 0.0);
    }

    #[test]
    fn p_controller_rotate_yaw() {
        let mut c = PController::new(0.5, 10.0);
        let t = c.rotate(0.5);
        assert!(t.linear.x.abs() < 1e-9);
        assert!(t.angular_z > 0.0);
    }

    #[test]
    fn pd_controller_reset_clears_state() {
        let mut c = PdController::new(0.5, 10.0);
        let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
        let _ = c.advance((1.0, 0.0), odom);
        c.reset_errors();
        assert_eq!(c.prev_yaw_error, 0.0);
        assert_eq!(c.prev_angular_velocity, 0.0);
    }

    #[test]
    fn pd_controller_reset_yaw_error() {
        let mut c = PdController::new(0.5, 10.0);
        c.reset_yaw_error(0.3);
        assert!((c.prev_yaw_error - 0.3).abs() < 1e-9);
    }

    #[test]
    fn simulation_boosts_small_angular() {
        let mut c = PController::new(0.5, 10.0);
        c.set_simulation(true);
        // yaw_error 很小导致 angular < 0.8，应被 boost 到 0.8
        let t = c.rotate(0.01);
        assert!((t.angular_z.abs() - 0.8).abs() < 1e-6);
    }
}
