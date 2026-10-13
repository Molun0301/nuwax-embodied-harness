//! 二维返航查询的只读诊断；不替换生产算法，不统计或猜测总分配器流量。
//! 仅在独占测量时段手动运行：
//! cargo test -p nuwax-nav-simple-planner --release --test mission_route_copy_probe -- --ignored --nocapture --test-threads=1
use std::hint::black_box;
use std::mem::size_of;
use std::time::{Duration, Instant};

use nuwax_nav_simple_planner::{Costmap, Point2, Point3, Pose3, Quaternion, SimplePlanner, SimplePlannerConfig};
use nuwax_nav_types_messages::{GoalOptions, OccupancyCell, OccupancyGrid2D, RobotGeometry};

const REPETITIONS: usize = 32;

fn geometry() -> RobotGeometry {
    RobotGeometry {
        footprint_xy_m: vec![[-0.25, -0.15], [0.25, -0.15], [0.25, 0.15], [-0.25, 0.15]],
        z_min_m: -0.2,
        z_max_m: 0.4,
        source: "mission-copy-diagnostic".into(),
    }
}

fn fixture(side: u32, sparse: bool) -> OccupancyGrid2D {
    let mut grid = OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.2, side, side, OccupancyCell::Free);
    if sparse {
        // 外圈未知，远离短测试路线的障碍；保留真实非空高度表的复制成本。
        for y in 0..side {
            for x in 0..side {
                if x < 5 || y < 5 || x >= side - 5 || y >= side - 5 {
                    grid.set(x, y, OccupancyCell::Unknown);
                } else if x % 7 == 0 && y > side / 2 && y % 7 == 0 {
                    grid.set(x, y, OccupancyCell::Occupied);
                }
            }
        }
    }
    grid
}

fn pose(x: f64, y: f64) -> Pose3 {
    Pose3 { position: Point3::new(x, y, 0.0), orientation: Quaternion::IDENTITY }
}

fn query(map: &Costmap, config: &SimplePlannerConfig, from: Pose3, to: Pose3, options: GoalOptions)
    -> Result<(Vec<Point2>, f64), String>
{
    let mut planner = SimplePlanner::new(config.clone()).unwrap();
    // 对齐 MissionRoutes::query 的真实第一层复制；第二层由生产 planning_view 完成。
    planner.replace_costmap(map.clone());
    planner.query_route_with_end_yaw(from, to, options).map_err(|error| error.to_string())
}

// 对齐 ordered_mission_query 的顺序及短路行为；返程起点必须采用外出路线实际末端。
// 本诊断不模拟调度器取消或 MissionRouteAdvice，只测这里的三段同步路线查询。
fn ordered_queries(map: &Costmap, config: &SimplePlannerConfig, from: Pose3, candidate: Pose3, home: Pose3)
    -> Vec<Result<(Vec<Point2>, f64), String>>
{
    // 与 explore_mission::home_goal_options 一致；平面 frontier 使用默认选项。
    let home_options = GoalOptions { allow_reverse: false, position_tolerance_m: 0.1, yaw_tolerance_rad: 0.2 };
    let mut results = vec![query(map, config, from, home, home_options)];
    if results[0].is_err() { return results; }
    results.push(query(map, config, from, candidate, GoalOptions::default()));
    let outward_end = match &results[1] {
        Ok((points, yaw)) => {
            let last = points.last().expect("成功路线包含末端");
            Pose3 { position: Point3::new(last.x, last.y, from.position.z), orientation: Quaternion::from_yaw(*yaw) }
        }
        Err(_) => return results,
    };
    results.push(query(map, config, outward_end, home, home_options));
    results
}

fn samples<T>(mut f: impl FnMut() -> T, mut verify: impl FnMut(&T)) -> Vec<Duration> {
    let mut times = Vec::with_capacity(REPETITIONS);
    for _ in 0..REPETITIONS {
        let start = Instant::now();
        let value = black_box(f());
        times.push(start.elapsed());
        // 保留外层返回对象至计时结束；断言、外层结果析构和输出不计时。
        // 完整查询内部的规划器/地图析构仍在查询调用内，包含在完整查询阶段计时中。
        verify(&value);
        drop(value);
    }
    times.sort_unstable();
    times
}

fn report(side: u32, sparse: bool, phase: &str, times: &[Duration], copied_grid_bytes: usize,
          counted_grid_allocations: usize, height_entries: usize) {
    println!("MISSION_COPY_PROBE side={side} sparse={sparse} phase={phase} samples={} p50_us={:.3} p95_us={:.3} copied_grid_bytes_per_sample={copied_grid_bytes} counted_grid_allocations_per_sample={counted_grid_allocations} source_height_entries={height_entries} total_allocator_bytes=not_measured",
        times.len(), times[times.len()/2].as_secs_f64()*1e6,
        times[(times.len()*95/100).min(times.len()-1)].as_secs_f64()*1e6);
}

#[test]
#[ignore = "独占CPU时段的二维复制成本诊断；不属于常规回归"]
fn mission_route_copy_probe() {
    for side in [100, 500] {
        for sparse in [false, true] {
            let grid = fixture(side, sparse);
            let original = grid.clone();
            let config = SimplePlannerConfig { robot_geometry: Some(geometry()), ..Default::default() };
            let map = Costmap::from_occupancy(grid, config.inflation_radius).unwrap();
            let source = map.occupancy_grid().unwrap();
            let bytes = source.cells.len() * size_of::<OccupancyCell>();
            let from = pose(4.1, 4.1);
            let candidate = pose(6.1, 4.1);
            let home = pose(4.1, 6.1);
            let times = samples(|| black_box(source).clone(), |copy: &OccupancyGrid2D| {
                assert_eq!(copy, source);
                assert_ne!(copy.cells.as_ptr(), source.cells.as_ptr());
            });
            report(side, sparse, "route_preparation_grid_clone_only", &times, bytes, 1, map.heights().len());
            let times = samples(|| black_box(&map).clone(), |copy: &Costmap| {
                let copied = copy.occupancy_grid().unwrap();
                assert_eq!(copied, source);
                assert_ne!(copied.cells.as_ptr(), source.cells.as_ptr());
            });
            report(side, sparse, "query_owned_costmap_clone_only", &times, bytes, 1, map.heights().len());
            let times = samples(|| black_box(&map).planning_view(Point2::new(4.1, 4.1), 0.0,
                config.robot_geometry.as_ref().unwrap()).unwrap(), |view: &Costmap| {
                let copied = view.occupancy_grid().unwrap();
                assert_eq!(copied, source);
                assert_ne!(copied.cells.as_ptr(), source.cells.as_ptr());
            });
            report(side, sparse, "body_planning_view_clone_and_clear", &times, bytes, 1, map.heights().len());

            // 完整三段生产查询，逐次比较精确路径、末端朝向和错误；不声称这是优化A/B。
            let expected = ordered_queries(&map, &config, from, candidate, home);
            assert_eq!(expected.len(), 3);
            assert!(expected.iter().all(Result::is_ok));
            let times = samples(|| ordered_queries(&map, &config, from, candidate, home),
                |actual| assert_eq!(actual, &expected));
            // 以下字节/次数来自明确的六次Vec复制调用，仅计占据数组，不计路径/索引/配置分配。
            report(side, sparse, "three_route_queries_full_source_counted_copies", &times, bytes*6, 6, map.heights().len());
            let outside = pose(-1.0, -1.0);
            let error = query(&map, &config, from, outside, GoalOptions::default());
            assert!(error.is_err());
            assert_eq!(query(&map, &config, from, outside, GoalOptions::default()), error);
            let home_error = ordered_queries(&map, &config, from, candidate, outside);
            assert_eq!(home_error.len(), 1);
            assert!(home_error[0].is_err());
            assert_eq!(ordered_queries(&map, &config, from, candidate, outside), home_error);
            let outward_error = ordered_queries(&map, &config, from, outside, home);
            assert_eq!(outward_error.len(), 2);
            assert!(outward_error[0].is_ok() && outward_error[1].is_err());
            assert_eq!(ordered_queries(&map, &config, from, outside, home), outward_error);
            assert_eq!(map.occupancy_grid().unwrap(), &original);
        }
    }
}
