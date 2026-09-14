//! 前沿查询的全量、重复与局部更新对照；地图构建耗时不计入查询数字。
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceMapBuilder, SurfaceSnapshot};
use std::collections::BTreeSet;
use std::time::Instant;
fn config() -> Config {
    Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.45,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.1,
        wall_buffer_m: 0.1,
        wall_buffer_weight: 0.0,
        step_threshold_m: 0.2,
        max_slope_deg: 25.0,
        min_step_tread_depth_m: 0.2,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    }
}
fn points(cells: &BTreeSet<(i32, i32, i32)>) -> Vec<(f32, f32, f32)> {
    cells
        .iter()
        .map(|&(x, y, z)| {
            (
                (x as f32 + 0.5) * 0.1,
                (y as f32 + 0.5) * 0.1,
                (z as f32 + 0.5) * 0.1,
            )
        })
        .collect()
}
fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}
fn main() {
    let mut cells: BTreeSet<_> = [0, 30]
        .into_iter()
        .flat_map(|z| (0..120).flat_map(move |x| (0..120).map(move |y| (x, y, z))))
        .collect();
    let start = [2.05, 2.05, 0.1];
    let mut builder = SurfaceMapBuilder::new(config());
    let initial = builder.build(&points(&cells), 0.1).unwrap().snapshot;
    initial.reachable_frontiers(start, &[]).unwrap();
    let mut cold = Vec::new();
    let mut warm = Vec::new();
    let mut incremental = Vec::new();
    let mut evaluated = Vec::new();
    for i in 0..7 {
        let cell = (50 + i, 60, 0);
        cells.remove(&cell);
        let input = points(&cells);
        let updated = builder.build(&input, 0.1).unwrap().snapshot;
        let full = SurfaceSnapshot::from_points(&input, config(), 0.1).unwrap();
        let time = Instant::now();
        let expected = full.reachable_frontiers(start, &[]).unwrap();
        cold.push(time.elapsed().as_secs_f64() * 1000.0);
        let time = Instant::now();
        let actual = updated.reachable_frontiers(start, &[]).unwrap();
        incremental.push(time.elapsed().as_secs_f64() * 1000.0);
        assert_eq!(actual.len(), expected.len());
        for (a, b) in actual.iter().zip(&expected) {
            assert_eq!((a.cell, a.gain), (b.cell, b.gain));
            assert!((a.travel_cost - b.travel_cost).abs() < 1e-8);
        }
        evaluated.push(updated.frontier_cache_stats().unwrap().evaluated_cells);
        let time = Instant::now();
        let repeat = updated.reachable_frontiers(start, &[]).unwrap();
        warm.push(time.elapsed().as_secs_f64() * 1000.0);
        assert_eq!(actual, repeat);
    }
    println!("两层约28800支撑，7次同输入查询对照；全量索引中位={:.3}ms，局部索引中位={:.3}ms，复用索引中位={:.3}ms，局部重算单元={evaluated:?}",median(&mut cold),median(&mut incremental),median(&mut warm));
    println!("查询包含当前起点可达成本搜索，不包含地图构建或生产线程排队；所有候选与全量相同。");
}
