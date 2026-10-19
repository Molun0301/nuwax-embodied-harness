//! 固定一列变化扩大远场，独立报告首建、更新和近场路径查询。
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceBodyEnvelope, SurfaceColumnUpdate, SurfaceMapBuilder,
};
use std::time::Instant;
fn config() -> Config {
    Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.6,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.15,
        wall_buffer_m: 0.3,
        wall_buffer_weight: 2.0,
        step_threshold_m: 0.2,
        max_slope_deg: 25.0,
        min_step_tread_depth_m: 0.2,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    }
}
fn main() {
    for side in [64, 256, 512] {
        let points: Vec<_> = (0..side)
            .flat_map(|x| {
                (0..side).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.0125))
            })
            .collect();
        let mut builder = SurfaceMapBuilder::configured(
            config(),
            Some(SurfaceBodyEnvelope {
                fixed_collision: None,
                footprint_xy_m: vec![[-0.15, -0.1], [0.15, -0.1], [0.15, 0.1], [-0.15, 0.1]],
                z_bounds_m: [-0.1, 0.2],
                body_to_support_z_m: 0.35,
                max_step_up_m: 0.2,
                max_step_down_m: 0.25,
                max_cross_slope_deg: 15.0,
                support_uncertainty_m: 0.0,
            }),
        );
        let now = Instant::now();
        let first = builder.build(&points, 0.025).unwrap();
        let first_ms = now.elapsed().as_secs_f64() * 1000.0;
        for iteration in 0..8 {
            let mut cells = vec![(0, [0.0, 0.025])];
            if iteration % 2 == 0 {
                cells.push((4, [0.4, 0.425]));
            }
            let update = [SurfaceColumnUpdate {
                column: [20, 20],
                cells,
            }];
            let now = Instant::now();
            let next = builder
                .build_column_updates(&update, 0.025, false, 2)
                .unwrap();
            let update_ms = now.elapsed().as_secs_f64() * 1000.0;
            let now = Instant::now();
            let path = next
                .snapshot
                .plan([0.85, 0.85, 0.025], [1.85, 0.85, 0.025])
                .unwrap();
            let near_ms = now.elapsed().as_secs_f64() * 1000.0;
            println!(
                "{}",
                serde_json::json!({"side":side,"cells":points.len(),"iteration":iteration,"first_ms":first_ms,"update_ms":update_ms,"near_ms":near_ms,"near_points":path.len(),"checked_cells":next.stats.checked_cells,"global_refresh":next.stats.global_cost_refresh})
            );
        }
        assert!(first
            .snapshot
            .plan([0.85, 0.85, 0.025], [1.85, 0.85, 0.025])
            .is_ok());
    }
}
