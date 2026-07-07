//! 集成测试:同输入 100 次结果完全一致(determinism)。

use nuwax_explore_wavefront::{
    config::WavefrontConfig,
    frontier_goal::{FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection},
    session::ExplorationSession,
    WavefrontGoalSelector,
};
use nuwax_nav_types_messages::geometry::{Point2, Timestamp};
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

mod fixtures;

struct FixedIdSrc;
impl FrontierGoalIdSource for FixedIdSrc {
    fn next_frontier_goal_id(&mut self, _seed: &FrontierGoalIdSeed) -> String {
        "fixed".to_string()
    }
}

#[test]
fn determinism_100_iterations() {
    let g = fixtures::grid_20x20_mixed();
    let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
    let mut first: Option<GoalSelection> = None;
    for _ in 0..100 {
        let mut session = ExplorationSession::new();
        let mut ids = FixedIdSrc;
        let r = sel
            .select_goal_at(
                Point2 { x: 1.0, y: 1.0 },
                &g,
                &mut session,
                Timestamp::default(),
                &mut ids,
            )
            .unwrap();
        match &first {
            None => first = Some(r),
            Some(prev) => assert_eq!(prev, &r, "selection must be deterministic across runs"),
        }
    }
}

#[test]
fn determinism_classification_grid_clear() {
    // detector 多次调用必须清空 classification
    let g = OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 6,
        height: 6,
        cells: vec![OccupancyCell::Unknown; 36],
    };
    let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
    let mut ids = FixedIdSrc;
    for _ in 0..3 {
        let mut session = ExplorationSession::new();
        let r = sel
            .select_goal_at(
                Point2 { x: 0.5, y: 0.5 },
                &g,
                &mut session,
                Timestamp::default(),
                &mut ids,
            )
            .unwrap();
        // 不应 panic;drain internal state
        let _ = r;
    }
    // 摘要 diagnostics 仍可访问
    let _ = sel.diagnostics();
}
