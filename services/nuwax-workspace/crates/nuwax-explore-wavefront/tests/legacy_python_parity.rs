//! legacy-parity fixture(只在 `--features legacy-parity` 跑)
//! 验证历史 Python `origin + index * resolution`(左下角)语义仍然可计算。

#![cfg(feature = "legacy-parity")]

use nuwax_explore_wavefront::legacy::grid_to_world_origin;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

#[test]
fn legacy_origin_math() {
    let g = OccupancyGrid2D {
        origin: Point2 { x: 1.0, y: 2.0 },
        resolution: 0.5,
        width: 4,
        height: 4,
        cells: vec![OccupancyCell::Free; 16],
    };
    let p = grid_to_world_origin(&g, 2, 3);
    assert!((p.x - (1.0 + 2.0 * 0.5)).abs() < 1e-9);
    assert!((p.y - (2.0 + 3.0 * 0.5)).abs() < 1e-9);
}
