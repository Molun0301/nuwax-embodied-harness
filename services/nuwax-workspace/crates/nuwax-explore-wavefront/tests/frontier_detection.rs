//! 集成测试:frontier 判定 / cluster / 边界情况。
//!
//! 通过 `test_helpers::detect_kind(...)` 摘要访问 internal detector state,
//! 不直接接触 crate-private `DetectionOutcome` / `FrontierCandidate`(满足 AC-2)。

use nuwax_explore_wavefront::{
    config::WavefrontConfig,
    frontier_goal::{GoalSelection, StopReason},
    session::ExplorationSession,
    test_helpers::{detect_kind, DetectionOutcomeKind, WavefrontDetector},
    GridPoint, PointClassification, WavefrontGoalSelector,
};
use nuwax_nav_types_messages::geometry::{Point2, Timestamp};
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

mod fixtures;

fn empty_grid(w: u32, h: u32) -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: w,
        height: h,
        cells: vec![OccupancyCell::Unknown; (w * h) as usize],
    }
}

#[test]
fn detector_handles_fully_occupied_grid() {
    let g = fixtures::grid_40x40_fully_occupied();
    let mut det = WavefrontDetector::new(WavefrontConfig::default());
    let session = ExplorationSession::new();
    let kind = detect_kind(&mut det, Point2 { x: 1.0, y: 1.0 }, &g, &session).unwrap();
    assert!(matches!(kind, DetectionOutcomeKind::NoReachableFreeSpace));
}

#[test]
fn detector_handles_fully_free_grid_as_no_frontier() {
    let g = fixtures::grid_40x40_free();
    let mut det = WavefrontDetector::new(WavefrontConfig::default());
    let session = ExplorationSession::new();
    let kind = detect_kind(&mut det, Point2 { x: 1.0, y: 1.0 }, &g, &session).unwrap();
    if let DetectionOutcomeKind::Candidates(n) = kind {
        assert_eq!(n, 0, "fully-free grid should yield 0 candidates");
    } else {
        panic!("expected Candidates(0)");
    }
}

#[test]
fn detector_out_of_bounds_returns_error() {
    let g = empty_grid(8, 8);
    let mut det = WavefrontDetector::new(WavefrontConfig::default());
    let session = ExplorationSession::new();
    let err = detect_kind(&mut det, Point2 { x: 999.0, y: 999.0 }, &g, &session).unwrap_err();
    let s = format!("{err}");
    assert!(
        s.contains("999"),
        "error should mention out-of-bounds coords: {s}"
    );
}

#[test]
fn classification_default_is_empty() {
    let f = PointClassification::default();
    assert!(f.is_empty());
}

#[test]
fn grid_point_constructs() {
    let p = GridPoint::new(3, 5);
    assert_eq!(p.x, 3);
    assert_eq!(p.y, 5);
}

#[test]
fn selector_reports_no_reachable_as_stop() {
    let g = fixtures::grid_40x40_fully_occupied();
    let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
    let mut session = ExplorationSession::new();
    let mut ids = TestIdSrc;
    let r = sel
        .select_goal_at(
            Point2 { x: 1.0, y: 1.0 },
            &g,
            &mut session,
            Timestamp::default(),
            &mut ids,
        )
        .unwrap();
    assert!(matches!(
        r,
        GoalSelection::Stop(StopReason::NoReachableFreeSpace)
    ));
}

struct TestIdSrc;
impl nuwax_explore_wavefront::frontier_goal::FrontierGoalIdSource for TestIdSrc {
    fn next_frontier_goal_id(
        &mut self,
        _seed: &nuwax_explore_wavefront::frontier_goal::FrontierGoalIdSeed,
    ) -> String {
        "f-test".to_string()
    }
}
