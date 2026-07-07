//! 基础路径跟随器（纯 Rust）。
//!
//! 与 C++ `path_follower` native bridge 不冲突：
//! - 本 crate 是 Rust fallback / 基础控制器；
//! - C++ `path_follower` 是 CMU pure pursuit 实现，由 `nuwax-nav-path-follower-bridge` 包装。
//!
//! 行为对账源：`dimos/navigation/basic_path_follower/module.py`。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};
use thiserror::Error;

// Foundation 协议面 re-export — 跨 crate PoseStamped / Twist2 / ControllerCommand 协议语义对齐
pub use nuwax_nav_base::NavError as FoundationNavError;
pub use nuwax_nav_types_messages::{
    ControllerCommand as FoundationControllerCommand, Pose2 as FoundationPose2,
    PoseStamped as FoundationPoseStamped, Twist2 as FoundationTwist2,
};

pub use nuwax_numeric::Point2;

/// 2D 位置（保持 xyxy 语义）
pub type Position2 = Point2;

/// yaw 偏角 wrap 到 `[-π, π]`
#[must_use]
pub fn wrap_angle(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// 计算 wrap 之后的 yaw 误差 = `target_yaw - current_yaw`
#[must_use]
pub fn angle_diff(target_yaw: f64, current_yaw: f64) -> f64 {
    wrap_angle(target_yaw - current_yaw)
}

/// 计算 lookahead 距离：`clamp(speed * time_s, lo, hi)`
#[must_use]
pub fn lookahead_distance(speed: f64, time_s: f64, lo: f64, hi: f64) -> f64 {
    if hi < lo {
        return speed * time_s;
    }
    let raw = speed * time_s;
    raw.clamp(lo, hi)
}

/// 错误类型
#[derive(Debug, Error)]
pub enum FollowerError {
    /// 配置非法
    #[error("invalid config: {0}")]
    InvalidConfig(String),
}

/// 控制器配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasicPathFollowerConfig {
    /// 前进线速度（m/s）
    pub speed: f64,
    /// 控制循环频率（Hz）
    pub control_frequency: f64,
    /// 到达距离（m）
    pub goal_tolerance: f64,
    /// lookahead 时间（s）
    pub lookahead_time_s: f64,
    /// lookahead 距离下界（m）
    pub min_lookahead_m: f64,
    /// lookahead 距离上界（m）
    pub max_lookahead_m: f64,
    /// heading P 增益
    pub heading_gain: f64,
    /// 最大角速度（rad/s）
    pub max_angular: f64,
}

impl BasicPathFollowerConfig {
    /// 校验配置合法性
    pub fn validate(&self) -> Result<(), FollowerError> {
        if !(self.speed.is_finite() && self.speed >= 0.0) {
            return Err(FollowerError::InvalidConfig(
                "speed must be finite >= 0".into(),
            ));
        }
        if !(self.control_frequency.is_finite() && self.control_frequency > 0.0) {
            return Err(FollowerError::InvalidConfig(
                "control_frequency must be finite > 0".into(),
            ));
        }
        if !(self.goal_tolerance.is_finite() && self.goal_tolerance >= 0.0) {
            return Err(FollowerError::InvalidConfig(
                "goal_tolerance must be finite >= 0".into(),
            ));
        }
        if !(self.lookahead_time_s.is_finite() && self.lookahead_time_s >= 0.0) {
            return Err(FollowerError::InvalidConfig(
                "lookahead_time_s must be finite >= 0".into(),
            ));
        }
        if !(self.min_lookahead_m.is_finite()
            && self.max_lookahead_m.is_finite()
            && self.min_lookahead_m >= 0.0
            && self.max_lookahead_m >= self.min_lookahead_m)
        {
            return Err(FollowerError::InvalidConfig(
                "lookahead bounds must satisfy 0 <= min <= max".into(),
            ));
        }
        if !(self.heading_gain.is_finite() && self.heading_gain >= 0.0) {
            return Err(FollowerError::InvalidConfig(
                "heading_gain must be finite >= 0".into(),
            ));
        }
        if !(self.max_angular.is_finite() && self.max_angular > 0.0) {
            return Err(FollowerError::InvalidConfig(
                "max_angular must be finite > 0".into(),
            ));
        }
        Ok(())
    }
}

impl Default for BasicPathFollowerConfig {
    fn default() -> Self {
        Self {
            speed: 0.5,
            control_frequency: 10.0,
            goal_tolerance: 0.3,
            lookahead_time_s: 1.5,
            min_lookahead_m: 0.4,
            max_lookahead_m: 1.5,
            heading_gain: 1.5,
            max_angular: 1.0,
        }
    }
}

/// 单步控制输出
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControllerCommand {
    /// 线速度（m/s）
    pub linear: f64,
    /// 角速度（rad/s）
    pub angular: f64,
}

impl ControllerCommand {
    /// 零指令
    #[must_use]
    pub fn zero() -> Self {
        Self {
            linear: 0.0,
            angular: 0.0,
        }
    }

    /// 是否为零
    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.linear.abs() < f64::EPSILON && self.angular.abs() < f64::EPSILON
    }
}

/// 路径跟随控制器（纯算法核心）。
///
/// 行为对齐 Python `BasicPathFollower._step`：
/// 1. 空路径 / stop 状态：返回 `zero()`。
/// 2. 终点距离 < `goal_tolerance`：清路径 + 返回 `zero()`（外部通过 `goal_reached` 信号上报）。
/// 3. 否则计算 lookahead target 并产生 angular/linear 指令。
#[derive(Debug, Clone)]
pub struct BasicPathFollowerCore {
    config: BasicPathFollowerConfig,
    waypoints: Option<Vec<Point2>>,
}

impl BasicPathFollowerCore {
    /// 构造
    pub fn new(config: BasicPathFollowerConfig) -> Result<Self, FollowerError> {
        config.validate()?;
        Ok(Self {
            config,
            waypoints: None,
        })
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &BasicPathFollowerConfig {
        &self.config
    }

    /// 当前是否持有路径
    #[must_use]
    pub fn has_path(&self) -> bool {
        self.waypoints.is_some()
    }

    /// 设置路径（waypoint 列表）。
    ///
    /// 空路径会清空当前路径（外部应发布 zero twist）。
    pub fn set_path(&mut self, waypoints: Vec<Point2>) {
        if waypoints.is_empty() {
            self.waypoints = None;
        } else {
            self.waypoints = Some(waypoints);
        }
    }

    /// 清空路径（外部应发布 zero twist）
    pub fn clear_path(&mut self) {
        self.waypoints = None;
    }

    /// 计算一次控制输出。
    ///
    /// `position`：当前世界位置；`yaw`：当前朝向（rad）。
    /// 返回 `Some(command)` 与 `goal_reached` 是否触发。
    pub fn step(&mut self, position: Point2, yaw: f64) -> StepOutput {
        let Some(waypoints) = self.waypoints.as_ref() else {
            return StepOutput::no_path();
        };
        let last = *waypoints.last().expect("non-empty guaranteed by set_path");
        let dist_last = (last - position).norm();
        if dist_last < self.config.goal_tolerance {
            // 清路径并发布 zero twist
            self.waypoints = None;
            return StepOutput::reached();
        }

        let lookahead = lookahead_distance(
            self.config.speed,
            self.config.lookahead_time_s,
            self.config.min_lookahead_m,
            self.config.max_lookahead_m,
        );
        let target = self.lookahead_point(waypoints, position, lookahead);
        let target_yaw = (target.y - position.y).atan2(target.x - position.x);
        let yaw_error = angle_diff(target_yaw, yaw);

        let angular = (self.config.heading_gain * yaw_error)
            .clamp(-self.config.max_angular, self.config.max_angular);
        let linear = self.config.speed * yaw_error.cos().max(0.0);

        StepOutput::moving(ControllerCommand { linear, angular })
    }

    /// 将当前位置投影到路径段，再沿路径插值 lookahead 距离
    fn lookahead_point(&self, waypoints: &[Point2], position: Point2, lookahead: f64) -> Point2 {
        if waypoints.len() == 1 {
            return waypoints[0];
        }
        let (seg_idx, proj) = self.project_onto_path(waypoints, position);
        let mut remaining = lookahead;
        let mut cursor = proj;
        for i in seg_idx..(waypoints.len() - 1) {
            let end = waypoints[i + 1];
            let dx = end.x - cursor.x;
            let dy = end.y - cursor.y;
            let seg_len = (dx * dx + dy * dy).sqrt();
            if seg_len >= remaining {
                let inv = if seg_len > 0.0 {
                    remaining / seg_len
                } else {
                    0.0
                };
                return Point2::new(cursor.x + dx * inv, cursor.y + dy * inv);
            }
            remaining -= seg_len;
            cursor = end;
        }
        *waypoints.last().expect("at least 2 waypoints here")
    }

    /// 把当前位置投影到多段折线，返回 (起始段索引, 投影点)
    fn project_onto_path(&self, waypoints: &[Point2], position: Point2) -> (usize, Point2) {
        let mut best_idx = 0_usize;
        let mut best_point = waypoints[0];
        let mut best_dist = f64::INFINITY;
        for i in 0..(waypoints.len() - 1) {
            let a = waypoints[i];
            let abx = waypoints[i + 1].x - a.x;
            let aby = waypoints[i + 1].y - a.y;
            let denom = abx * abx + aby * aby;
            let t = if denom == 0.0 {
                0.0
            } else {
                let px = position.x - a.x;
                let py = position.y - a.y;
                let raw = (px * abx + py * aby) / denom;
                raw.clamp(0.0, 1.0)
            };
            let proj = Point2::new(a.x + abx * t, a.y + aby * t);
            let dist = {
                let dx = position.x - proj.x;
                let dy = position.y - proj.y;
                (dx * dx + dy * dy).sqrt()
            };
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
                best_point = proj;
            }
        }
        (best_idx, best_point)
    }
}

/// 单步控制输出结构
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StepOutput {
    /// 控制指令
    pub command: ControllerCommand,
    /// 是否到达目标
    pub goal_reached: bool,
    /// 是否处于无路径状态
    pub no_path: bool,
}

impl StepOutput {
    /// 无路径状态
    #[must_use]
    pub fn no_path() -> Self {
        Self {
            command: ControllerCommand::zero(),
            goal_reached: false,
            no_path: true,
        }
    }

    /// 到达目标
    #[must_use]
    pub fn reached() -> Self {
        Self {
            command: ControllerCommand::zero(),
            goal_reached: true,
            no_path: false,
        }
    }

    /// 正常运动
    #[must_use]
    pub fn moving(command: ControllerCommand) -> Self {
        Self {
            command,
            goal_reached: false,
            no_path: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn default_config() -> BasicPathFollowerConfig {
        BasicPathFollowerConfig {
            speed: 0.5,
            control_frequency: 10.0,
            goal_tolerance: 0.3,
            lookahead_time_s: 1.5,
            min_lookahead_m: 0.4,
            max_lookahead_m: 1.5,
            heading_gain: 1.5,
            max_angular: 1.0,
        }
    }

    #[test]
    fn lookahead_distance_clamps_lower() {
        // 0.1 m/s * 1.5 s = 0.15 m < min 0.4 -> clamp to 0.4
        assert_relative_eq!(lookahead_distance(0.1, 1.5, 0.4, 1.5), 0.4);
    }

    #[test]
    fn lookahead_distance_clamps_upper() {
        // 5 m/s * 1.5 s = 7.5 m > max 1.5 -> clamp to 1.5
        assert_relative_eq!(lookahead_distance(5.0, 1.5, 0.4, 1.5), 1.5);
    }

    #[test]
    fn lookahead_distance_no_clamp() {
        // 0.5 m/s * 1.5 s = 0.75 m 介于 0.4..1.5
        assert_relative_eq!(lookahead_distance(0.5, 1.5, 0.4, 1.5), 0.75);
    }

    #[test]
    fn angle_diff_wraps_pi() {
        // wrap(2*PI - 0.2) → -0.2
        assert_relative_eq!(angle_diff(PI - 0.1, -PI + 0.1), -0.2, epsilon = 1e-9);
        // wrap(PI) → PI（边界）
        let diff = angle_diff(PI, 0.0);
        assert!((diff - PI).abs() < 1e-9 || (diff + PI).abs() < 1e-9);
    }

    #[test]
    fn single_point_path_returns_that_point() {
        let cfg = default_config();
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        core.set_path(vec![Point2::new(1.0, 0.0)]);
        let out = core.step(Point2::new(0.0, 0.0), 0.0);
        // 单点 + 距离 > tolerance 不会 reached
        assert!(!out.goal_reached);
        // angular 接近 0（target 在 x 轴正方向）
        assert!(out.command.angular.abs() < 0.1);
        assert!(out.command.linear > 0.0);
    }

    #[test]
    fn multi_segment_path_interpolates_no_corner_cut() {
        let cfg = default_config();
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        // 沿 x 正方向直线路径（避免切角歧义）
        core.set_path(vec![Point2::new(2.0, 0.0), Point2::new(4.0, 0.0)]);
        // 在 (0,0) 朝向 0（x+）
        let out = core.step(Point2::new(0.0, 0.0), 0.0);
        // angular ≈ 0，linear ≈ speed = 0.5
        assert!(out.command.angular.abs() < 0.05);
        assert!(out.command.linear > 0.45 && out.command.linear <= 0.5);
    }

    #[test]
    fn goal_reached_publishes_zero_and_clears_path() {
        let cfg = default_config();
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        core.set_path(vec![Point2::new(0.1, 0.0)]);
        let out = core.step(Point2::new(0.0, 0.0), 0.0);
        assert!(out.goal_reached);
        assert!(out.command.is_zero());
        assert!(!core.has_path());
    }

    #[test]
    fn yaw_wrap_clamped_angular() {
        let cfg = default_config();
        let max_angular = cfg.max_angular;
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        // 目标在 x 正方向，与朝向 -π+0.01，差 ≈ π-0.01 → 接近 π
        core.set_path(vec![Point2::new(2.0, 0.0), Point2::new(2.0, 0.0)]);
        let out = core.step(Point2::new(0.0, 0.0), -PI + 0.01);
        // yaw_error ≈ π - 0.01 -> angular ≈ heading_gain * (π-0.01) -> clamped to max_angular
        assert!(out.command.angular.abs() <= max_angular + 1e-9);
        // cos(π-0.01) ≈ -1 -> linear = speed * max(0, -1) = 0
        // 这就是"大角度时不倒车"的语义
        assert!(out.command.linear >= 0.0);
    }

    #[test]
    fn empty_path_zero_twist() {
        let cfg = default_config();
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        // 从未 set_path
        let out = core.step(Point2::new(0.0, 0.0), 0.0);
        assert!(out.no_path);
        assert!(out.command.is_zero());
    }

    #[test]
    fn clear_path_publishes_zero() {
        let cfg = default_config();
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        core.set_path(vec![Point2::new(1.0, 0.0)]);
        core.clear_path();
        let out = core.step(Point2::new(0.0, 0.0), 0.0);
        assert!(out.no_path);
        assert!(out.command.is_zero());
    }

    #[test]
    fn large_yaw_error_no_reverse_linear() {
        let cfg = default_config();
        let max_angular = cfg.max_angular;
        let mut core = BasicPathFollowerCore::new(cfg).unwrap();
        // 目标在 y+ 方向（PI/2），当前朝向 0 -> yaw_error = π/2 -> cos = 0
        core.set_path(vec![Point2::new(0.0, 2.0)]);
        let out = core.step(Point2::new(0.0, 0.0), 0.0);
        // 不倒车：linear 应 >= 0
        assert!(out.command.linear >= 0.0);
        // angular 应当被夹到 max_angular
        assert_relative_eq!(out.command.angular, max_angular);
    }

    #[test]
    fn config_validation() {
        let mut bad = default_config();
        bad.speed = -1.0;
        assert!(BasicPathFollowerCore::new(bad).is_err());
        let mut bad = default_config();
        bad.min_lookahead_m = 2.0;
        bad.max_lookahead_m = 1.0;
        assert!(BasicPathFollowerCore::new(bad).is_err());
    }
}
