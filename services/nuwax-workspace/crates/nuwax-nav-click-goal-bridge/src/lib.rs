//! Click → start / goal pose 交替路由（纯 Rust）。
//!
//! 行为对账源：`dimos/navigation/cmu_nav/modules/click_start_goal_router/click_start_goal_router.py`。
//!
//! 重要：虽然是 `bridge` 命名，但是**纯 Rust 路由**，与 native bridge runner 不混。
//!
//! 行为：
//! - 初始 `_next_is_start=true`
//! - 第一次 click -> `start_pose`，下一次 click 设置 goal
//! - 第二次 click -> `goal_pose`，下一次 click 回到 start
//! - orientation 固定 identity quaternion
//! - frame 使用 config `world_frame`，默认 `map`
//! - RPC: `reset_next_is_start()`

#![deny(unsafe_code)]
#![warn(missing_docs)]

use serde::{Deserialize, Serialize};

// Foundation 协议面 re-export — 跨 crate PoseStamped / Quaternion 协议语义对齐
pub use nuwax_nav_types_messages::{
    Point3 as FoundationPoint3, Pose3 as FoundationPose3, PoseStamped,
    Quaternion as FoundationQuaternion,
};

// PoseStamped 已迁移到 Foundation `nuwax_nav_types_messages::PoseStamped`;
// 旧 `crate::PoseStamped` 直接指向 Foundation 完整字段形态
// (`frame_id: NavFrameId` + `stamp: Timestamp` + `pose: Pose3`)。

/// PointStamped 输入（自包含）
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClickPoint {
    /// frame_id of source click
    pub frame_id: String,
    /// x
    pub x: f64,
    /// y
    pub y: f64,
    /// z
    pub z: f64,
}

impl ClickPoint {
    /// 构造
    #[must_use]
    pub fn new(frame_id: impl Into<String>, x: f64, y: f64, z: f64) -> Self {
        Self {
            frame_id: frame_id.into(),
            x,
            y,
            z,
        }
    }
}

/// 路由结果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RoutedClick {
    /// 作为 start_pose 发出
    StartPose(PoseStamped),
    /// 作为 goal_pose 发出
    GoalPose(PoseStamped),
}

/// 配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClickStartGoalRouterConfig {
    /// 输出 frame（默认 map）
    pub world_frame: String,
}

impl Default for ClickStartGoalRouterConfig {
    fn default() -> Self {
        Self {
            world_frame: "map".to_string(),
        }
    }
}

/// 路由核心
#[derive(Debug, Clone)]
pub struct ClickStartGoalRouter {
    config: ClickStartGoalRouterConfig,
    next_is_start: bool,
}

impl ClickStartGoalRouter {
    /// 构造
    #[must_use]
    pub fn new(config: ClickStartGoalRouterConfig) -> Self {
        Self {
            config,
            next_is_start: true,
        }
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &ClickStartGoalRouterConfig {
        &self.config
    }

    /// 当前下一次 click 是否指向 start
    #[must_use]
    pub fn next_is_start(&self) -> bool {
        self.next_is_start
    }

    /// 重置序列（下一次 click 重新指 start）
    pub fn reset_next_is_start(&mut self) {
        self.next_is_start = true;
    }

    /// 处理 click：交替返回 Start / Goal。
    pub fn handle_click(&mut self, click: &ClickPoint) -> RoutedClick {
        let pose = PoseStamped::from_pose(
            nuwax_nav_types_messages::Pose3::from_pose2(nuwax_nav_types_messages::Pose2::new(
                click.x, click.y, 0.0,
            )),
            nuwax_nav_types_messages::NavFrameId::from_static(&self.config.world_frame),
        );
        if self.next_is_start {
            self.next_is_start = false;
            RoutedClick::StartPose(pose)
        } else {
            self.next_is_start = true;
            RoutedClick::GoalPose(pose)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    fn router() -> ClickStartGoalRouter {
        ClickStartGoalRouter::new(ClickStartGoalRouterConfig::default())
    }

    #[test]
    fn first_click_is_start_with_identity_quaternion() {
        let mut r = router();
        assert!(r.next_is_start());
        let out = r.handle_click(&ClickPoint::new("map", 1.0, 2.0, 0.0));
        match out {
            RoutedClick::StartPose(p) => {
                assert_eq!(p.frame_id, nuwax_nav_types_messages::NavFrameId::Map);
                assert!((p.pose.position.x - 1.0).abs() < 1e-9);
                assert!((p.pose.position.y - 2.0).abs() < 1e-9);
                assert!((p.pose.orientation.w - 1.0).abs() < 1e-9);
            }
            _ => panic!("expected StartPose"),
        }
    }

    #[test]
    fn alternates_start_goal() {
        let mut r = router();
        assert!(matches!(
            r.handle_click(&ClickPoint::new("map", 1.0, 0.0, 0.0)),
            RoutedClick::StartPose(_)
        ));
        assert!(matches!(
            r.handle_click(&ClickPoint::new("map", 2.0, 0.0, 0.0)),
            RoutedClick::GoalPose(_)
        ));
        assert!(matches!(
            r.handle_click(&ClickPoint::new("map", 3.0, 0.0, 0.0)),
            RoutedClick::StartPose(_)
        ));
        assert!(matches!(
            r.handle_click(&ClickPoint::new("map", 4.0, 0.0, 0.0)),
            RoutedClick::GoalPose(_)
        ));
    }

    #[test]
    fn custom_world_frame() {
        let cfg = ClickStartGoalRouterConfig {
            world_frame: "world".into(),
        };
        let mut r = ClickStartGoalRouter::new(cfg);
        match r.handle_click(&ClickPoint::new("click_source", 0.0, 0.0, 0.0)) {
            RoutedClick::StartPose(p) => {
                assert_eq!(
                    p.frame_id,
                    nuwax_nav_types_messages::NavFrameId::Custom("world".to_string())
                );
            }
            _ => panic!(),
        }
    }

    #[test]
    fn reset_returns_to_start() {
        let mut r = router();
        let _ = r.handle_click(&ClickPoint::new("map", 0.0, 0.0, 0.0));
        // 一次 click 之后下一次 next_is_start 应该是 false（即将是 goal）
        assert!(!r.next_is_start());
        r.reset_next_is_start();
        assert!(r.next_is_start());
        // next click should be start
        match r.handle_click(&ClickPoint::new("map", 0.0, 0.0, 0.0)) {
            RoutedClick::StartPose(_) => {}
            _ => panic!("expected start after reset"),
        }
    }

    #[test]
    fn every_pose_has_identity_quaternion() {
        let mut r = router();
        for _ in 0..6 {
            let out = r.handle_click(&ClickPoint::new("map", 0.0, 0.0, 0.0));
            let p = match out {
                RoutedClick::StartPose(p) => p,
                RoutedClick::GoalPose(p) => p,
            };
            assert!((p.pose.orientation.w - 1.0).abs() < 1e-9);
        }
    }
}
