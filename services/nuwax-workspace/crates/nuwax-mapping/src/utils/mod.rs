//! 工具模块
//! 提供基于 kiddo KD-tree 的空间距离查询。

pub mod distance;

pub use distance::{nearest_neighbor, pairwise_distance, DistanceMetric};
