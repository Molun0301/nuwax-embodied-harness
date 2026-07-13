//! MovementManager：点击转发、teleop/nav 速度 mux、teleop 接管时取消导航。
//!
//! 行为对账源：`dimos/navigation/movement_manager/movement_manager.py`。
//!
//! 端口语义：
//! - 入 `clicked_point`：校验有限值和范围，通过后同时发布 `goal` 与 `way_point`
//! - 入 `nav_cmd_vel`：teleop cooldown 未激活时转发到 `cmd_vel`
//! - 入 `tele_cmd_vel`：缩放后发布到 `cmd_vel`，并立即 cancel navigation
//! - 出 `goal` / `way_point` / `cmd_vel` / `stop_movement`

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use thiserror::Error;

// Foundation 协议面 re-export — 跨 crate PointStamped / Twist 语义对齐
pub use nuwax_nav_base::NavError as FoundationNavError;
pub use nuwax_nav_types_messages::Point3 as FoundationPoint3;
pub use nuwax_nav_types_messages::PointStamped;
pub use nuwax_nav_types_messages::Twist2 as FoundationTwist2;
pub use nuwax_nav_types_messages::Twist3 as Twist;

// P0-13 Module 副作用层:把 `cancel_navigation()` 的 CancelNavigation event
// 真副作用化 -- 直接通过 broadcast port 发 `MovementCommand::Stop` 和 NaN goal,
// 而不只是返回 event 让 caller 自己转发。
pub mod module;
pub use module::*;

/// 默认 click 水平范围（米）
pub const DEFAULT_MAX_CLICK_HORIZONTAL_M: f64 = 500.0;
/// 默认 click 垂直范围（米）
pub const DEFAULT_MAX_CLICK_VERTICAL_M: f64 = 50.0;
/// 默认 teleop cooldown（秒）
pub const DEFAULT_TELE_COOLDOWN_SEC: f64 = 1.0;

/// 取消策略
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CancelGoalPolicy {
    /// 同时发布 `stop_movement=true` 和 NaN goal（与现有 Python 行为兼容）
    #[default]
    LegacyStopAndNan,
    /// 仅发布 `stop_movement=true`；typed cancel 由 stack 接管
    TypedStop,
    /// 仅发 NaN goal（不推荐，保留兼容）
    LegacyNanOnly,
}

/// 3D vector scaling
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3Scaling {
    /// x
    pub x: f64,
    /// y
    pub y: f64,
    /// z
    pub z: f64,
}

impl Default for Vec3Scaling {
    fn default() -> Self {
        Self {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        }
    }
}

/// Twist 缩放
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct TwistScaling {
    /// linear scaling
    pub linear: Vec3Scaling,
    /// angular scaling
    pub angular: Vec3Scaling,
}

/// Twist 缩放辅助：返回缩放后的 Foundation `Twist3`
///
/// Foundation `Twist3` 字段为 `linear: Point3 / angular: Point3`，
/// 不能直接挂 `scaled()`；本扩展 trait 在 cfg 内补充扩展方法，
/// 行为与原本地 `Twist::scaled` 完全一致。
pub trait Twist3ScalingExt {
    /// 应用 `TwistScaling`，返回新 `Twist3`
    #[must_use]
    fn scaled(self, s: TwistScaling) -> Self;
}

impl Twist3ScalingExt for Twist {
    fn scaled(self, s: TwistScaling) -> Self {
        Self {
            linear: FoundationPoint3 {
                x: self.linear.x * s.linear.x,
                y: self.linear.y * s.linear.y,
                z: self.linear.z * s.linear.z,
            },
            angular: FoundationPoint3 {
                x: self.angular.x * s.angular.x,
                y: self.angular.y * s.angular.y,
                z: self.angular.z * s.angular.z,
            },
        }
    }
}

// PointStamped 已迁移到 Foundation `nuwax_nav_types_messages::PointStamped`;
// 旧调用 `crate::PointStamped` 自动指向 Foundation 字段形态
// (`frame_id: NavFrameId` + `stamp: Timestamp` + `point: Point3`)。

/// 配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementManagerConfig {
    /// teleop 后 nav 抑制时长（秒）
    pub tele_cooldown_sec: f64,
    /// teleop 缩放
    pub tele_cmd_vel_scaling: TwistScaling,
    /// click 水平范围上限（米）
    pub max_click_horizontal_m: f64,
    /// click 垂直范围上限（米）
    pub max_click_vertical_m: f64,
    /// 取消策略
    pub cancel_goal_policy: CancelGoalPolicy,
}

impl Default for MovementManagerConfig {
    fn default() -> Self {
        Self {
            tele_cooldown_sec: DEFAULT_TELE_COOLDOWN_SEC,
            tele_cmd_vel_scaling: TwistScaling::default(),
            max_click_horizontal_m: DEFAULT_MAX_CLICK_HORIZONTAL_M,
            max_click_vertical_m: DEFAULT_MAX_CLICK_VERTICAL_M,
            cancel_goal_policy: CancelGoalPolicy::default(),
        }
    }
}

/// 事件
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MovementEvent {
    /// 通过校验的 click：转发为 goal + way_point
    ClickAccepted(PointStamped),
    /// click 被拒绝：原因
    ClickRejected {
        /// x
        x: f64,
        /// y
        y: f64,
        /// z
        z: f64,
        /// 拒绝原因
        reason: ClickRejection,
    },
    /// 取消导航
    CancelNavigation {
        /// stop_movement=true ?
        publish_stop: bool,
        /// 是否同时发 NaN goal
        publish_nan_goal: bool,
    },
    /// cmd_vel 输出
    CmdVel(Twist),
}

/// click 拒绝原因
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClickRejection {
    /// 存在 NaN/Inf
    NonFinite,
    /// 水平超界
    HorizontalOutOfRange,
    /// 垂直超界
    VerticalOutOfRange,
}

/// 错误
#[derive(Debug, Error)]
pub enum MovementManagerError {
    /// 配置非法
    #[error("invalid config: {0}")]
    InvalidConfig(String),
}

/// MovementManager 核心（不含 IO 适配）
///
/// 设计要点：
/// - 不直接持有 transport / stream，只提供 `handle_*` 入口 + `MovementEvent` 输出；
/// - 调用方（stack/transport 适配）将 `MovementEvent` 落到具体 topic；
/// - 线程安全由外部决定（核心状态在 `MovementManager` 中通过 `&mut self` 顺序调用）。
#[derive(Debug)]
pub struct MovementManager {
    config: MovementManagerConfig,
    teleop_active: bool,
    last_teleop_time: Option<Instant>,
}

impl MovementManager {
    /// 构造
    pub fn new(config: MovementManagerConfig) -> Result<Self, MovementManagerError> {
        if !(config.tele_cooldown_sec.is_finite() && config.tele_cooldown_sec >= 0.0) {
            return Err(MovementManagerError::InvalidConfig(
                "tele_cooldown_sec must be finite >= 0".into(),
            ));
        }
        if !(config.max_click_horizontal_m.is_finite() && config.max_click_horizontal_m > 0.0) {
            return Err(MovementManagerError::InvalidConfig(
                "max_click_horizontal_m must be finite > 0".into(),
            ));
        }
        if !(config.max_click_vertical_m.is_finite() && config.max_click_vertical_m > 0.0) {
            return Err(MovementManagerError::InvalidConfig(
                "max_click_vertical_m must be finite > 0".into(),
            ));
        }
        Ok(Self {
            config,
            teleop_active: false,
            last_teleop_time: None,
        })
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &MovementManagerConfig {
        &self.config
    }

    /// 处理 click：返回多个 event（成功时 1 个 `ClickAccepted`；失败时 1 个 `ClickRejected`）
    pub fn handle_click(&mut self, point: &PointStamped) -> Vec<MovementEvent> {
        if !point.point.x.is_finite() || !point.point.y.is_finite() || !point.point.z.is_finite() {
            return vec![MovementEvent::ClickRejected {
                x: point.point.x,
                y: point.point.y,
                z: point.point.z,
                reason: ClickRejection::NonFinite,
            }];
        }
        if point.point.x.abs() > self.config.max_click_horizontal_m
            || point.point.y.abs() > self.config.max_click_horizontal_m
        {
            return vec![MovementEvent::ClickRejected {
                x: point.point.x,
                y: point.point.y,
                z: point.point.z,
                reason: ClickRejection::HorizontalOutOfRange,
            }];
        }
        if point.point.z.abs() > self.config.max_click_vertical_m {
            return vec![MovementEvent::ClickRejected {
                x: point.point.x,
                y: point.point.y,
                z: point.point.z,
                reason: ClickRejection::VerticalOutOfRange,
            }];
        }
        vec![MovementEvent::ClickAccepted(point.clone())]
    }

    /// 处理 nav cmd_vel。
    ///
    /// 返回的 `Vec` 通常为空或单元素 `CmdVel`。teleop cooldown 内直接丢弃。
    pub fn handle_nav_cmd_vel(&mut self, twist: Twist) -> Vec<MovementEvent> {
        self.refresh_teleop_cooldown();
        if self.teleop_active {
            // cooldown 未过期
            return Vec::new();
        }
        vec![MovementEvent::CmdVel(twist)]
    }

    /// 处理 teleop cmd_vel。
    ///
    /// 总是先 cancel navigation，再按 scaling 发布到 cmd_vel。
    pub fn handle_tele_cmd_vel(&mut self, twist: Twist) -> Vec<MovementEvent> {
        self.teleop_active = true;
        self.last_teleop_time = Some(Instant::now());
        let cancel = self.build_cancel_event();
        let scaled = twist.scaled(self.config.tele_cmd_vel_scaling);
        vec![cancel, MovementEvent::CmdVel(scaled)]
    }

    /// 主动请求 cancel navigation（用于 RPC / 测试）
    pub fn cancel_navigation(&mut self) -> Vec<MovementEvent> {
        vec![self.build_cancel_event()]
    }

    fn refresh_teleop_cooldown(&mut self) {
        if !self.teleop_active {
            return;
        }
        let Some(t) = self.last_teleop_time else {
            self.teleop_active = false;
            return;
        };
        let cooldown = Duration::from_secs_f64(self.config.tele_cooldown_sec);
        if t.elapsed() >= cooldown {
            self.teleop_active = false;
        }
    }

    fn build_cancel_event(&self) -> MovementEvent {
        let (publish_stop, publish_nan_goal) = match self.config.cancel_goal_policy {
            CancelGoalPolicy::LegacyStopAndNan => (true, true),
            CancelGoalPolicy::TypedStop => (true, false),
            CancelGoalPolicy::LegacyNanOnly => (false, true),
        };
        MovementEvent::CancelNavigation {
            publish_stop,
            publish_nan_goal,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::NavFrameId;
    use pretty_assertions::assert_eq;

    fn default_manager() -> MovementManager {
        MovementManager::new(MovementManagerConfig::default()).unwrap()
    }

    #[test]
    fn click_finite_pass() {
        let mut m = default_manager();
        let ev = m.handle_click(&PointStamped::from_xyz(
            NavFrameId::from_static("map"),
            1.0,
            2.0,
            0.0,
        ));
        assert_eq!(ev.len(), 1);
        assert!(matches!(ev[0], MovementEvent::ClickAccepted(_)));
    }

    #[test]
    fn click_nan_rejected() {
        let mut m = default_manager();
        let ev = m.handle_click(&PointStamped::from_xyz(
            NavFrameId::from_static("map"),
            f64::NAN,
            0.0,
            0.0,
        ));
        assert_eq!(ev.len(), 1);
        match &ev[0] {
            MovementEvent::ClickRejected { reason, .. } => {
                assert_eq!(*reason, ClickRejection::NonFinite);
            }
            _ => panic!("expected rejected"),
        }
    }

    #[test]
    fn click_inf_rejected() {
        let mut m = default_manager();
        let ev = m.handle_click(&PointStamped::from_xyz(
            NavFrameId::from_static("map"),
            f64::INFINITY,
            0.0,
            0.0,
        ));
        assert!(matches!(
            ev[0],
            MovementEvent::ClickRejected {
                reason: ClickRejection::NonFinite,
                ..
            }
        ));
    }

    #[test]
    fn click_horizontal_out_of_range_rejected() {
        let mut m = default_manager();
        let ev = m.handle_click(&PointStamped::from_xyz(
            NavFrameId::from_static("map"),
            1000.0,
            0.0,
            0.0,
        ));
        match &ev[0] {
            MovementEvent::ClickRejected { reason, .. } => {
                assert_eq!(*reason, ClickRejection::HorizontalOutOfRange);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn click_vertical_out_of_range_rejected() {
        let mut m = default_manager();
        let ev = m.handle_click(&PointStamped::from_xyz(
            NavFrameId::from_static("map"),
            0.0,
            0.0,
            100.0,
        ));
        match &ev[0] {
            MovementEvent::ClickRejected { reason, .. } => {
                assert_eq!(*reason, ClickRejection::VerticalOutOfRange);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn teleop_publishes_scaled_and_cancels() {
        let cfg = MovementManagerConfig {
            tele_cmd_vel_scaling: TwistScaling {
                linear: Vec3Scaling {
                    x: 0.5,
                    y: 1.0,
                    z: 1.0,
                },
                angular: Vec3Scaling {
                    x: 1.0,
                    y: 1.0,
                    z: 0.25,
                },
            },
            ..MovementManagerConfig::default()
        };
        let mut m = MovementManager::new(cfg).unwrap();
        let tele = Twist {
            linear: FoundationPoint3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: FoundationPoint3 {
                x: 0.0,
                y: 0.0,
                z: 0.8,
            },
        };
        let evs = m.handle_tele_cmd_vel(tele);
        assert_eq!(evs.len(), 2);
        // 第一条：cancel
        assert!(matches!(
            evs[0],
            MovementEvent::CancelNavigation {
                publish_stop: true,
                publish_nan_goal: true,
            }
        ));
        // 第二条：缩放后的 cmd_vel
        match &evs[1] {
            MovementEvent::CmdVel(t) => {
                assert_eq!(t.linear.x, 0.5);
                assert_eq!(t.angular.z, 0.2);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn nav_cmd_suppressed_during_teleop_cooldown() {
        let mut m = default_manager();
        let tele = Twist {
            linear: FoundationPoint3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: FoundationPoint3::default(),
        };
        let _ = m.handle_tele_cmd_vel(tele);
        let nav = Twist::default();
        let evs = m.handle_nav_cmd_vel(nav);
        // cooldown 内应被抑制
        assert!(evs.is_empty());
    }

    #[test]
    fn nav_cmd_released_after_cooldown() {
        let cfg = MovementManagerConfig {
            tele_cooldown_sec: 0.0, // 立即失效
            ..MovementManagerConfig::default()
        };
        let mut m = MovementManager::new(cfg).unwrap();
        let tele = Twist {
            linear: FoundationPoint3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: FoundationPoint3::default(),
        };
        let _ = m.handle_tele_cmd_vel(tele);
        std::thread::sleep(Duration::from_millis(10));
        let nav = Twist::default();
        let evs = m.handle_nav_cmd_vel(nav);
        assert_eq!(evs.len(), 1);
        assert!(matches!(evs[0], MovementEvent::CmdVel(_)));
    }

    #[test]
    fn cancel_navigation_event_obeys_policy() {
        let cfg = MovementManagerConfig {
            cancel_goal_policy: CancelGoalPolicy::TypedStop,
            ..MovementManagerConfig::default()
        };
        let mut m = MovementManager::new(cfg).unwrap();
        let evs = m.cancel_navigation();
        assert_eq!(evs.len(), 1);
        assert!(matches!(
            evs[0],
            MovementEvent::CancelNavigation {
                publish_stop: true,
                publish_nan_goal: false,
            }
        ));
    }

    #[test]
    fn invalid_config_rejected() {
        let cfg = MovementManagerConfig {
            tele_cooldown_sec: -1.0,
            ..MovementManagerConfig::default()
        };
        assert!(MovementManager::new(cfg).is_err());
        let cfg = MovementManagerConfig {
            max_click_horizontal_m: 0.0,
            ..MovementManagerConfig::default()
        };
        assert!(MovementManager::new(cfg).is_err());
    }
}
