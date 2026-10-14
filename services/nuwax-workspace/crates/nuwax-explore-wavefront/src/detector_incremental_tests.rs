use super::*;
use crate::session::ExplorationSession;

fn grid() -> OccupancyGrid2D {
    let mut grid =
        OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.2, 64, 40, OccupancyCell::Unknown);
    for y in 3..36 {
        for x in (3..23).chain(42..61) {
            grid.set(x, y, OccupancyCell::Free);
        }
    }
    grid
}
fn signature(
    detector: &mut WavefrontDetector,
    grid: &OccupancyGrid2D,
    robot: Point2,
) -> Vec<(f64, f64, usize, f64, f64, f64)> {
    match detector
        .detect_candidates(robot, grid, &ExplorationSession::new())
        .unwrap()
    {
        DetectionOutcome::NoReachableFreeSpace => Vec::new(),
        DetectionOutcome::Candidates(candidates) => candidates
            .iter()
            .map(|c| {
                (
                    c.cluster.centroid.x,
                    c.cluster.centroid.y,
                    c.cluster.cell_count,
                    c.goal_position.x,
                    c.goal_position.y,
                    c.travel_cost,
                )
            })
            .collect(),
    }
}
#[test]
fn incremental_frontiers_match_fresh_queries_through_doors_splits_and_layout_changes() {
    let mut grid = grid();
    let mut incremental = WavefrontDetector::new(WavefrontConfig::default());
    let left = Point2::new(2.1, 3.1);
    let right = Point2::new(10.1, 3.1);
    for cycle in 0..24 {
        if cycle % 4 == 1 {
            for x in 23..42 {
                for y in 15..25 {
                    grid.set(x, y, OccupancyCell::Free);
                }
            }
        }
        if cycle % 4 == 2 {
            for y in 0..40 {
                grid.set(32, y, OccupancyCell::Occupied);
            }
        }
        if cycle % 4 == 3 {
            for y in 0..40 {
                grid.set(32, y, OccupancyCell::Unknown);
            }
        }
        let robot = if cycle % 2 == 0 { left } else { right };
        let actual = signature(&mut incremental, &grid, robot);
        let expected = signature(
            &mut WavefrontDetector::new(WavefrontConfig::default()),
            &grid,
            robot,
        );
        assert_eq!(actual, expected, "门与前沿拆合 cycle={cycle}");
    }
    grid.origin = Point2::new(1.3, -2.7);
    let moved = Point2::new(left.x + grid.origin.x, left.y + grid.origin.y);
    assert_eq!(
        signature(&mut incremental, &grid, moved),
        signature(
            &mut WavefrontDetector::new(WavefrontConfig::default()),
            &grid,
            moved
        )
    );
    assert!(incremental.diagnostics().incremental.layout_reset);
}
#[test]
fn local_grid_change_reuses_untouched_components_and_footprint_changes_rebuild_safety() {
    let mut grid = grid();
    let mut detector = WavefrontDetector::new(WavefrontConfig::default());
    let robot = Point2::new(2.1, 3.1);
    signature(&mut detector, &grid, robot);
    signature(&mut detector, &grid, robot);
    let stats = &detector.diagnostics().incremental;
    assert_eq!(stats.safe_cells_recomputed, 0);
    assert_eq!(stats.frontier_cells_recomputed, 0);
    assert_eq!(stats.components_rebuilt, 0);
    assert!(stats.components_retained > 0);
    grid.set(60, 10, OccupancyCell::Occupied);
    signature(&mut detector, &grid, robot);
    let stats = &detector.diagnostics().incremental;
    assert_eq!(stats.changed_cells, 1);
    assert!(stats.safe_cells_recomputed < 100);
    assert!(stats.frontier_cells_recomputed <= 9);
    let geometry=nuwax_nav_types_messages::RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.45,-0.15],[0.45,-0.15],[0.45,0.15],[-0.45,0.15]],"z_min_m":-0.3,"z_max_m":0.3}"#).unwrap();
    for yaw in [0.0, std::f64::consts::FRAC_PI_2] {
        let footprint = geometry
            .grid_footprint(grid.resolution, yaw, grid.width, grid.height)
            .unwrap();
        detector.set_grid_footprint(Some(footprint.clone()));
        let mut full = WavefrontDetector::new(WavefrontConfig::default());
        full.set_grid_footprint(Some(footprint));
        assert_eq!(
            signature(&mut detector, &grid, robot),
            signature(&mut full, &grid, robot)
        );
        assert_eq!(
            detector.diagnostics().incremental.safe_cells_recomputed,
            grid.cells.len()
        );
    }
}
#[test]
fn incremental_safe_view_matches_original_disk_implementation_under_cell_edits() {
    let mut grid = grid();
    let radius = 0.4;
    let cells = 2i64;
    let offsets: Vec<_> = (-cells..=cells)
        .flat_map(|y| (-cells..=cells).map(move |x| (x, y)))
        .filter(|(x, y)| x * x + y * y <= cells * cells)
        .collect();
    let mut cache = crate::incremental::PlanarFrontierCache::default();
    for i in 0..80u32 {
        grid.set(
            4 + (i * 17) % 55,
            4 + (i * 7) % 30,
            match i % 3 {
                0 => OccupancyCell::Unknown,
                1 => OccupancyCell::Occupied,
                _ => OccupancyCell::Free,
            },
        );
        let actual = cache.update_grid(&grid, &offsets, |_| false);
        assert_eq!(actual, footprint_safe_map(&grid, radius));
    }
}
