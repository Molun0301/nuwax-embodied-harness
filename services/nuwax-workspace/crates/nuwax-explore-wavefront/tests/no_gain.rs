//! 集成测试:no-gain counter 增长 / 触发 / 清零。

use nuwax_explore_wavefront::{
    config::WavefrontConfig,
    frontier_goal::{FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection, StopReason},
    session::ExplorationSession,
    WavefrontGoalSelector,
};
use nuwax_nav_types_messages::geometry::{Point2, Timestamp};

mod fixtures;

struct FixedIdSrc;
impl FrontierGoalIdSource for FixedIdSrc {
    fn next_frontier_goal_id(&mut self, _seed: &FrontierGoalIdSeed) -> String {
        "fixed".to_string()
    }
}

#[test]
fn no_gain_triggers_stop_after_threshold() {
    let g = fixtures::grid_40x40_free();
    let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
    let mut session = ExplorationSession::new();
    // seed 信息计数
    let _ = sel.select_goal_at(
        Point2 { x: 1.0, y: 1.0 },
        &g,
        &mut session,
        Timestamp::default(),
        &mut FixedIdSrc,
    );
    // 推到 min_goals_before_no_gain_check
    for _ in 0..WavefrontConfig::default().min_goals_before_no_gain_check {
        session.mark_explored_goal(Point2 { x: 0.0, y: 0.0 });
    }
    // 连续多次 select,应触发 NoInformationGain
    let mut triggered = false;
    for _ in 0..10 {
        let r = sel
            .select_goal_at(
                Point2 { x: 1.0, y: 1.0 },
                &g,
                &mut session,
                Timestamp::default(),
                &mut FixedIdSrc,
            )
            .unwrap();
        if matches!(r, GoalSelection::Stop(StopReason::NoInformationGain)) {
            triggered = true;
            break;
        }
    }
    assert!(triggered, "NoInformationGain should be triggered");
}

#[test]
fn free_grid_with_no_explored_does_not_trigger_no_gain() {
    let g = fixtures::grid_40x40_free();
    let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
    let mut session = ExplorationSession::new();
    for _ in 0..20 {
        let r = sel
            .select_goal_at(
                Point2 { x: 1.0, y: 1.0 },
                &g,
                &mut session,
                Timestamp::default(),
                &mut FixedIdSrc,
            )
            .unwrap();
        // explored_goals 始终为 0,不应触发 no-gain
        assert!(!matches!(
            r,
            GoalSelection::Stop(StopReason::NoInformationGain)
        ));
    }
}
