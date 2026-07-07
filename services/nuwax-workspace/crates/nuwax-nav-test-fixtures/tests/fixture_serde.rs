//! Fixture serde roundtrip 与稳定性测试

use nuwax_nav_base::diagnostics::NavDiagnostics;
use nuwax_nav_test_fixtures::*;

#[test]
fn fixtures_serde_roundtrip_pose() {
    let p = pose(1.0, 2.0, 0.5);
    let s = serde_json::to_string(&p).unwrap();
    let back: nuwax_nav_types_messages::PoseStamped = serde_json::from_str(&s).unwrap();
    assert!((back.pose.position.x - 1.0).abs() < 1e-9);
}

#[test]
fn fixtures_serde_roundtrip_grid() {
    let g = grid_4x4_with_obstacle();
    let s = serde_json::to_string(&g).unwrap();
    let back: nuwax_nav_types_messages::grid::OccupancyGrid2D = serde_json::from_str(&s).unwrap();
    assert_eq!(
        back.get(1, 1),
        Some(nuwax_nav_types_messages::grid::OccupancyCell::Occupied)
    );
    assert_eq!(
        back.get(0, 0),
        Some(nuwax_nav_types_messages::grid::OccupancyCell::Free)
    );
}

#[test]
fn fixtures_serde_roundtrip_navigation_goal() {
    let g = goal("goal-A", 1.5, 2.5);
    let s = serde_json::to_string(&g).unwrap();
    let back: nuwax_nav_types_messages::goal::NavigationGoal = serde_json::from_str(&s).unwrap();
    assert_eq!(back.id, "goal-A");
    assert_eq!(
        back.source,
        nuwax_nav_types_messages::goal::GoalSource::Click
    );
}

#[test]
fn fixtures_serde_roundtrip_bbox() {
    let b = bbox(10.0, 20.0, 110.0, 220.0);
    let s = serde_json::to_string(&b).unwrap();
    let back: nuwax_nav_types_messages::visual::BoundingBox2D = serde_json::from_str(&s).unwrap();
    assert!(back.is_valid());
}

#[test]
fn fixtures_serde_roundtrip_planner_request() {
    let r = planner_request((0.0, 0.0), (3.0, 3.0));
    let s = serde_json::to_string(&r).unwrap();
    let back: nuwax_nav_types_messages::goal::PlannerRequest = serde_json::from_str(&s).unwrap();
    assert_eq!(back.options.cost_threshold, 100);
}

#[test]
fn fixtures_serde_roundtrip_frontier_goal() {
    let g = frontier_goal("f-1", 1.0, 2.0, 0.7);
    let s = serde_json::to_string(&g).unwrap();
    let back: nuwax_nav_types_messages::visual::FrontierGoal = serde_json::from_str(&s).unwrap();
    assert_eq!(back.id, "f-1");
    assert!(back.validate().is_ok());
    assert_eq!(
        back.metadata.get("cell_count").map(String::as_str),
        Some("5")
    );
}

#[test]
fn fixtures_navigation_diagnostics_roundtrip() {
    let d = nav_diagnostics();
    let s = serde_json::to_string(&d).unwrap();
    let back: NavDiagnostics = serde_json::from_str(&s).unwrap();
    assert_eq!(back.component, "nuwax-nav-stack");
    assert_eq!(back.state, "Moving");
}

#[test]
fn fixtures_statuses_all_roundtrip() {
    for st in all_statuses() {
        let s = serde_json::to_string(&st).unwrap();
        let back: nuwax_nav_types_messages::goal::NavigationStatus =
            serde_json::from_str(&s).unwrap();
        assert_eq!(back, st);
    }
}

#[test]
fn fixtures_levels_all_roundtrip() {
    for lv in all_levels() {
        let s = serde_json::to_string(&lv).unwrap();
        let back: nuwax_nav_types_messages::diagnostics::DiagnosticLevel =
            serde_json::from_str(&s).unwrap();
        assert_eq!(back, lv);
    }
}
