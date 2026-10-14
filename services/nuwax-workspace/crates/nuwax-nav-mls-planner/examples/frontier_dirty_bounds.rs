//! 小范围前沿失效的查询成本；构图、更新与全量参考校验不计入计时。
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceColumnUpdate, SurfaceMapBuilder, SurfaceSnapshot,
};
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
fn main() {
    for side in [64, 256] {
        let points: Vec<_> = [0, 30]
            .into_iter()
            .flat_map(|z| {
                (0..side).flat_map(move |x| {
                    (0..side).map(move |y| {
                        (
                            (x as f32 + 0.5) * 0.1,
                            (y as f32 + 0.5) * 0.1,
                            (z as f32 + 0.5) * 0.1,
                        )
                    })
                })
            })
            .collect();
        let mut builder = SurfaceMapBuilder::new(config());
        let first = builder.build(&points, 0.1).unwrap().snapshot;
        let first_stats = first.prepare_frontier_geometry(&[]).unwrap();
        let center = side / 2;
        for iteration in 0..20 {
            let removed = iteration % 2 == 0;
            let update = SurfaceColumnUpdate {
                column: [center, center],
                cells: if removed {
                    vec![(30, [3.0, 3.1])]
                } else {
                    vec![(0, [0.0, 0.1]), (30, [3.0, 3.1])]
                },
            };
            let next = builder
                .build_column_updates(&[update], 0.1, false, 1)
                .unwrap()
                .snapshot;
            let start = Instant::now();
            let stats = next.prepare_frontier_geometry(&[]).unwrap();
            let us = start.elapsed().as_secs_f64() * 1e6;
            assert!(stats.incremental);
            assert!(stats.evaluated_cells < first_stats.evaluated_cells);
            // 每种输入与独立冷构图核对全部可达候选、收益和代价。
            if iteration < 2 {
                let input: Vec<_> = points
                    .iter()
                    .copied()
                    .filter(|&(x, y, z)| {
                        !(removed
                            && (x / 0.1).floor() as i32 == center
                            && (y / 0.1).floor() as i32 == center
                            && z < 1.0)
                    })
                    .collect();
                let full = SurfaceSnapshot::from_points(&input, config(), 0.1).unwrap();
                for height in [0.1, 3.1] {
                    assert_eq!(
                        next.reachable_frontiers([1.05, 1.05, height], &[]).unwrap(),
                        full.reachable_frontiers([1.05, 1.05, height], &[]).unwrap()
                    );
                }
            }
            println!(
                "{}",
                serde_json::json!({"side": side, "iteration": iteration, "us": us,
                "evaluated_cells":stats.evaluated_cells,"frontiers":stats.frontier_count})
            );
        }
        assert_eq!(first.frontier_cache_stats(), Some(first_stats));
    }
}
