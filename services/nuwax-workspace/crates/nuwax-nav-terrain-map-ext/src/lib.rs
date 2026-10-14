//! 扩展地形图(Extended Terrain Map)处理器(对应原 Python `terrain_map_ext.py`)。
//!
//! 维护一个滚动 voxel grid 的"持久世界视图",并叠加 local terrain;输出经过
//! 衰减/连接性过滤的远场地形点云。
//!
//! 不使用 Open3D / SciPy / NumPy;点云以 `&[Point3]` 切片传入。
//!
//! Foundation 集成:基础类型 `Point2 / Point3 / Pose3 / Quaternion / Timestamp` 通过
//! `pub use` 转发 `nuwax-nav-types-messages`,与 simple-planner / a-star / 全部
//! 下游 nav crate 共享同一份定义。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod config;
pub mod connectivity;
pub mod crop;
pub mod decay;
pub mod ground;
pub mod merge;
pub mod voxel_grid;

#[cfg(feature = "module")]
pub mod module;

pub use config::TerrainMapExtConfig;
pub use voxel_grid::VoxelPoint;
pub use voxel_grid::{
    voxel_index, RollingTerrainGrid, PLANAR_VOXEL_HALF_WIDTH, PLANAR_VOXEL_NUM, PLANAR_VOXEL_WIDTH,
    TERRAIN_VOXEL_HALF_WIDTH, TERRAIN_VOXEL_NUM, TERRAIN_VOXEL_WIDTH,
};

// ============================================================================
// Foundation re-export(单一真实来源)
// ============================================================================
//
// 基础类型 `Point2 / Point3 / Pose3 / Quaternion / Timestamp` 通过
// `pub use` 转发 `nuwax-nav-types-messages`,与 simple-planner / a-star / 全部
// 下游 nav crate 共享同一份定义。本 crate 不再重复定义任何 `pub struct`。
pub use nuwax_nav_types_messages::{Point2, Point3, Pose3, Quaternion, Timestamp};
// sensors:点云 / 里程计
pub use nuwax_nav_types_messages::{Odometry, PointCloud};

/// 带时间的点云输入(`registered_scan` 形态)
///
/// 借用语义 + 显式时间分量(秒,浮点);与 Foundation `PointCloud`(拥有语义 + 纳秒 stamp)
/// 是不同的形态:`TimedPointCloud` 适合"批次扫描 + per-point timing"输入场景,
/// `PointCloud` 适合"输出 topic / 序列化"场景。
#[derive(Debug, Clone)]
pub struct TimedPointCloud<'a> {
    /// 点
    pub points: &'a [Point3],
    /// 每个点的 timestamp(秒,浮点)
    pub times: &'a [f64],
}

/// 统一错误类型(Foundation 转发)。
pub use nuwax_nav_base::NavError;

pub use voxel_grid::TerrainMapExtProcessor;
