//! 集成测试:5 项分值 + tie-break 稳定(走公共 test_helpers)。

use nuwax_explore_wavefront::{config::WavefrontConfig, test_helpers::FrontierScorer};
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

fn empty_grid() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.1,
        width: 20,
        height: 20,
        cells: vec![OccupancyCell::Free; 400],
    }
}

#[test]
fn total_score_in_plausible_range() {
    let g = empty_grid();
    let cfg = WavefrontConfig::default();
    let geom = nuwax_explore_wavefront::test_helpers::CandidateGeometry {
        centroid: Point2 { x: 1.0, y: 1.0 },
        cell_count: 10,
        perimeter_m: 1.0,
        area_m2: 0.1,
    };
    let b = FrontierScorer::new().score(&geom, Point2 { x: 0.0, y: 0.0 }, &g, &[], None, &cfg);
    assert!(b.total >= 0.0, "total must be non-negative: {}", b.total);
    assert!(b.total <= 1.0 + 1e-6, "total must be <= 1.0: {}", b.total);
}
