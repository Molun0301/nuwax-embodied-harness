//! 500x500 release benchmark(方案 §4.5 / AC-4 要求 < 200ms)。
//!
//! 共享 helper 抽到 `benches/common.rs`(单文件,避免重复 `mod common`)。

use criterion::{criterion_group, criterion_main, Criterion};
use nuwax_explore_wavefront::{
    config::WavefrontConfig,
    frontier_goal::{FrontierGoalIdSeed, FrontierGoalIdSource},
    session::ExplorationSession,
    WavefrontGoalSelector,
};
use nuwax_nav_types_messages::geometry::{Point2, Timestamp};

mod common;
use common::bench_grid_500x500;

struct FixedIdSrc;
impl FrontierGoalIdSource for FixedIdSrc {
    fn next_frontier_goal_id(&mut self, _seed: &FrontierGoalIdSeed) -> String {
        "fixed".to_string()
    }
}

fn bench_select_goal_at_500x500(c: &mut Criterion) {
    let grid = bench_grid_500x500();
    let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
    let mut session = ExplorationSession::new();
    c.bench_function("select_goal_at_500x500", |b| {
        b.iter(|| {
            let _ = sel
                .select_goal_at(
                    Point2 { x: 50.0, y: 50.0 },
                    &grid,
                    &mut session,
                    Timestamp::default(),
                    &mut FixedIdSrc,
                )
                .unwrap();
        });
    });
}

criterion_group!(benches, bench_select_goal_at_500x500);
criterion_main!(benches);
