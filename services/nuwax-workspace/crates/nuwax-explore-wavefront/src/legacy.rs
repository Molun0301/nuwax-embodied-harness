//! legacy-parity fixture / 转换(只在 `legacy-parity` feature 启用时编译)。
//!
//! 历史 Python wavefront 使用 `origin + index * resolution`(左下角)语义;
//! Rust `OccupancyGrid2D::cell_center` 返回格子中心。本模块只用于对账测试,
//! 不进入生产 scorer / detector。

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;

/// 历史 Python `origin + index * resolution` 语义的世界坐标(左下角)
#[must_use]
pub fn grid_to_world_origin(grid: &OccupancyGrid2D, ix: u32, iy: u32) -> Point2 {
    Point2 {
        x: grid.origin.x + (ix as f64) * grid.resolution,
        y: grid.origin.y + (iy as f64) * grid.resolution,
    }
}
