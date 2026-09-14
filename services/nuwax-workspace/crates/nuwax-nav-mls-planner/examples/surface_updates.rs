//! 同输入比较全量与共享表面更新；输出原始样本，不把开发机结果当目标板承诺。

use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceMapBuilder, SurfaceSnapshot};
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
    let args: Vec<_> = std::env::args().collect();
    let side: usize = args
        .get(1)
        .map_or(128, |value| value.parse().expect("边长必须为整数"));
    let iterations: usize = args
        .get(2)
        .map_or(5, |value| value.parse().expect("次数必须为整数"));
    assert!(
        (32..=512).contains(&side) && (1..=20).contains(&iterations),
        "边长32—512，次数1—20"
    );
    let base: Vec<_> = (0..side)
        .flat_map(|x| {
            (0..side).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))
        })
        .collect();
    for name in [
        "相同点云",
        "同体素扰动",
        "支撑以下占据变化",
        "局部障碍增删",
        "全量变化",
    ] {
        let mut builder = SurfaceMapBuilder::new(config());
        builder.build(&base, 0.05).unwrap();
        for sample in 0..iterations {
            let mut points = base.clone();
            match name {
                "同体素扰动" => {
                    if sample % 2 == 0 {
                        for point in &mut points {
                            point.0 += 0.01;
                            point.1 += 0.01;
                        }
                        points.reverse();
                    }
                }
                "支撑以下占据变化" | "局部障碍增删" if sample % 2 == 0 => {
                    let center = (side as f32 * 0.5 + 0.5) * 0.1;
                    points.push((
                        center,
                        center,
                        if name == "局部障碍增删" {
                            0.45
                        } else {
                            -0.05
                        },
                    ));
                }
                "全量变化" if sample % 2 == 0 => {
                    for point in &mut points {
                        point.2 += 0.2;
                    }
                }
                _ => {}
            }
            let measure_full = || {
                let start = Instant::now();
                let value = SurfaceSnapshot::from_points(&points, config(), 0.05).unwrap();
                (value, start.elapsed().as_secs_f64() * 1e3)
            };
            let mut measure_update = || {
                let start = Instant::now();
                let value = builder.build(&points, 0.05).unwrap();
                (value, start.elapsed().as_secs_f64() * 1e3)
            };
            // 交替测量顺序，避免总让同一路径先预热输入缓存。
            let (full, full_ms, update, update_ms) = if sample % 2 == 0 {
                let (update, update_ms) = measure_update();
                let (full, full_ms) = measure_full();
                (full, full_ms, update, update_ms)
            } else {
                let (full, full_ms) = measure_full();
                let (update, update_ms) = measure_update();
                (full, full_ms, update, update_ms)
            };
            assert_eq!(
                full.cell_keys().count(),
                update.snapshot.cell_keys().count()
            );
            println!(
                "{}",
                serde_json::json!({
                    "场景":name,"边长格数":side,"样本":sample,"点数":points.len(),
                    "先测":if sample%2==0 {"更新"} else {"全量"},
                    "全量毫秒":full_ms,"更新毫秒":update_ms,"路径":format!("{:?}",update.stats.kind),
                    "变化列":update.stats.changed_columns,"检查单元":update.stats.checked_cells,
                    "全局代价刷新":update.stats.global_cost_refresh,
                    "架构":std::env::consts::ARCH,"优化构建":!cfg!(debug_assertions),
                })
            );
        }
    }
}
