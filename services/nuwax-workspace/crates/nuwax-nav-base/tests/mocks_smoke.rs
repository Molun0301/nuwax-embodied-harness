//! Foundation `nuwax-nav-base` mock sanity 测试
//!
//! 每个 mock 至少一个构造 + 字段访问 sanity 测试，证明 trait + data shape 都可工作。
//! Mock 内部 trait 行为由 Foundation crate 自带的 `#[cfg(test)]` 模块覆盖。

use nuwax_nav_base::mocks::{
    MockBoundsGoalValidator, MockFifoGoalManager, MockFixedPhraseVisualQuery,
    MockStraightLinePlanner, MockZeroController,
};
use nuwax_nav_base::types::GoalId;

#[test]
fn mock_straight_line_planner_constructible() {
    // 单元 struct 直接构造
    let _p = MockStraightLinePlanner;
}

#[test]
fn mock_bounds_goal_validator_constructible() {
    // 默认构造 + buffer_m 字段访问
    let v = MockBoundsGoalValidator::default();
    let _ = v.buffer_m;
}

#[test]
fn mock_zero_controller_constructible() {
    let _c = MockZeroController;
}

#[test]
fn mock_fifo_goal_manager_constructible_and_queryable() {
    let m = MockFifoGoalManager::new();
    // 初始 queue 为空
    assert!(m.is_empty());
    assert_eq!(m.len(), 0);
}

#[test]
fn mock_fixed_phrase_visual_query_constructible() {
    // MockFixedPhraseVisualQuery 是单元 struct,直接构造
    let _q = MockFixedPhraseVisualQuery;
}

#[test]
fn foundation_data_types_constructible() {
    let _ = GoalId::new("test-goal");
}
