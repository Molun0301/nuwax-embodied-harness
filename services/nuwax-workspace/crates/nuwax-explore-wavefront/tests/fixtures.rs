//! 共享测试夹具(20x20 / 40x40 / 60x60 / 500x500)
//! 提供 free / partial-free / partially-occupied / mixed 等基础栅格。

#![allow(dead_code)]

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

/// 4m x 4m,0.2m/cell 的全 free 栅格(20x20)
#[must_use]
pub fn grid_20x20_free() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 20,
        height: 20,
        cells: vec![OccupancyCell::Free; 400],
    }
}

/// 8m x 8m,0.2m/cell 的全 free 栅格(40x40)
#[must_use]
pub fn grid_40x40_free() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 40,
        height: 40,
        cells: vec![OccupancyCell::Free; 1600],
    }
}

/// 8m x 8m,0.2m/cell 的全 occupied 栅格(40x40)
#[must_use]
pub fn grid_40x40_fully_occupied() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 40,
        height: 40,
        cells: vec![OccupancyCell::Occupied; 1600],
    }
}

/// 12m x 12m,0.2m/cell 的全 free 栅格(60x60)
#[must_use]
pub fn grid_60x60_free() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 60,
        height: 60,
        cells: vec![OccupancyCell::Free; 3600],
    }
}

/// 100m x 100m,0.2m/cell 的全 free 栅格(500x500)
#[must_use]
pub fn grid_500x500_free() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 500,
        height: 500,
        cells: vec![OccupancyCell::Free; 250_000],
    }
}

/// mixed:中心 10x10 自由 + 周边 unknown + 角落 1x1 occupied
#[must_use]
pub fn grid_20x20_mixed() -> OccupancyGrid2D {
    let mut g = grid_20x20_free();
    for cell in g.cells.iter_mut() {
        *cell = OccupancyCell::Unknown;
    }
    // 中心 10x10
    for y in 5..15 {
        for x in 5..15 {
            g.cells[(y * 20 + x) as usize] = OccupancyCell::Free;
        }
    }
    // 角落 1x1 occupied
    g.cells[0] = OccupancyCell::Occupied;
    g
}
