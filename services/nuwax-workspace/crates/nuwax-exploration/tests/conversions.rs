//! `conversions` 集成测试(方案 §8.3)
//!
//! 显式验证:不使用 `From` impl,而是 free function;
//! 不依赖 wavefront crate;默认 build 启用 `patrolling-conversions` 必须单独 feature。

use nuwax_exploration::conversions::frontier_goal_to_navigation_goal;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped, Quaternion, Timestamp};
use nuwax_nav_types_messages::goal::GoalSource;
use nuwax_nav_types_messages::visual::FrontierGoal;

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
    g.producer = "nuwax-explore-wavefront/test".to_string();
    g.stamp = Timestamp::from_unix(1_700_000_000, 0);
    g
}

#[test]
fn to_navigation_goal_uses_free_function() {
    let g = sample_goal();
    let nav = frontier_goal_to_navigation_goal(&g);
    assert_eq!(nav.id, "f-1");
    assert_eq!(nav.target.pose.position.x, 1.5);
    assert_eq!(nav.target.pose.position.y, -2.5);
    assert!(matches!(nav.source, GoalSource::Frontier));
}

#[cfg(feature = "patrolling-conversions")]
#[test]
fn to_patrolling_input_is_feature_gated() {
    use nuwax_exploration::conversions::frontier_goal_to_patrolling_input;
    let g = sample_goal();
    let inp = frontier_goal_to_patrolling_input(&g);
    assert_eq!(inp.pose, (1.5, -2.5, 0.0));
    assert!((inp.score - 0.7).abs() < 1e-6);
    assert_eq!(inp.information_gain, Some(2.0));
    assert_eq!(inp.distance_cost, Some(1.25));
    assert_eq!(inp.frontier_area_m2, Some(3.0));
    assert_eq!(inp.producer, "nuwax-explore-wavefront/test");
}
