//! `FrontierGoal` -> 导航 / patrolling 类型 唯一转换入口(方案 §8.3)
//!
//! 跨导航转换统一放在 `nuwax-exploration::conversions`;`nuwax-explore-wavefront`
//! 与 `nuwax-explore-tare-bridge` 都不依赖 `nuwax-nav-patrolling`。
//!
//! 由于 Rust orphan rule,`FrontierGoal` 与 `NavigationGoal` / `FrontierGoalInput`
//! 都定义在 `nuwax-nav-types-messages`,所以**不能**实现 `impl From<FrontierGoal>`。
//! 本模块提供 free function 形式。

use nuwax_nav_types_messages::goal::{GoalSource, NavigationGoal};
#[cfg(feature = "patrolling-conversions")]
use nuwax_nav_types_messages::sensors::FrontierGoalInput;
use nuwax_nav_types_messages::visual::FrontierGoal;

/// `FrontierGoal -> NavigationGoal` 唯一转换入口(方案 §8.3 显式合同)
///
/// 使用 `NavigationGoal::new(id, pose).with_source(GoalSource::Frontier)`,
/// 不实现 `impl From`。容差使用 `NavigationGoal::new` 默认值
/// (position 0.5 m / yaw 0.2 rad / priority 0)。
#[must_use]
pub fn frontier_goal_to_navigation_goal(goal: &FrontierGoal) -> NavigationGoal {
    NavigationGoal::new(goal.id.clone(), goal.pose.clone()).with_source(GoalSource::Frontier)
}

/// `FrontierGoal -> FrontierGoalInput` 唯一转换入口(方案 §8.3 + §8.5)
///
/// 字段映射:`pose=(x, y, 0.0)` / `score` / `information_gain` / `distance_cost` /
/// `frontier_area_m2` / `producer`。`pose` 简化是 patrolling 内部排序的约定
/// (参考 `nuwax-nav-types-messages::sensors::FrontierGoalInput` 文档)。
#[cfg(feature = "patrolling-conversions")]
#[must_use]
pub fn frontier_goal_to_patrolling_input(goal: &FrontierGoal) -> FrontierGoalInput {
    FrontierGoalInput {
        pose: (goal.pose.pose.position.x, goal.pose.pose.position.y, 0.0),
        score: goal.score,
        information_gain: goal.information_gain,
        distance_cost: goal.distance_cost,
        frontier_area_m2: goal.frontier_area_m2,
        producer: goal.producer.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::frames::NavFrameId;
    use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped, Quaternion, Timestamp};
    use std::collections::BTreeMap;

    fn sample_goal() -> FrontierGoal {
        let pose = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1_700_000_000, 0),
            pose: Pose3 {
                position: Point3 {
                    x: 1.5,
                    y: -2.5,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        let mut g = FrontierGoal::new("f-1", pose, 0.7);
        g.information_gain = Some(2.0);
        g.distance_cost = Some(1.25);
        g.frontier_area_m2 = Some(3.0);
        g.producer = "test-producer".to_string();
        g.stamp = Timestamp::from_unix(1_700_000_000, 0);
        g.metadata = BTreeMap::new();
        g
    }

    #[test]
    fn to_navigation_goal_preserves_id_pose_and_source() {
        let g = sample_goal();
        let nav = frontier_goal_to_navigation_goal(&g);
        assert_eq!(nav.id, "f-1");
        assert_eq!(nav.target, g.pose);
        assert!(matches!(nav.source, GoalSource::Frontier));
    }

    #[test]
    fn to_navigation_goal_default_tolerances() {
        let g = sample_goal();
        let nav = frontier_goal_to_navigation_goal(&g);
        assert!((nav.position_tolerance_m - 0.5).abs() < 1e-9);
        assert!((nav.yaw_tolerance_rad - 0.2).abs() < 1e-9);
        assert_eq!(nav.priority, 0);
    }

    #[cfg(feature = "patrolling-conversions")]
    #[test]
    fn to_patrolling_input_copies_field_mapping() {
        let g = sample_goal();
        let inp = frontier_goal_to_patrolling_input(&g);
        assert_eq!(inp.pose, (1.5, -2.5, 0.0));
        assert!((inp.score - 0.7).abs() < 1e-6);
        assert_eq!(inp.information_gain, Some(2.0));
        assert_eq!(inp.distance_cost, Some(1.25));
        assert_eq!(inp.frontier_area_m2, Some(3.0));
        assert_eq!(inp.producer, "test-producer");
    }
}
