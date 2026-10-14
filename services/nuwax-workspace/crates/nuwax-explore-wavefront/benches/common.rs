//! 共享 bench helper(由 benches/*.rs 引用)。

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

/// 500x500 全 free 栅格(与方案 §4.5 benchmark fixture 一致)
#[must_use]
pub fn bench_grid_500x500() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 500,
        height: 500,
        cells: vec![OccupancyCell::Free; 250_000],
    }
}
