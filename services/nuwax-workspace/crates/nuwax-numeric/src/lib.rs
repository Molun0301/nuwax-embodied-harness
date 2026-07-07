//! nuwax 数值计算基础
//! 提供：
//! - 栅格容器（基于 `ndarray`）
//! - 线性代数封装（基于 `nalgebra`）
//! - KD-tree 加速（基于 `kiddo`）

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod grid;
pub mod point;

pub use grid::OccupancyGridLike;
pub use point::{Point2, Point3, PointCloud2Like};

pub mod prelude {
    //! 常用类型 re-export
    pub use crate::grid::{OccupancyGrid, OccupancyValue};
    pub use crate::point::{Point2, Point3};
}
