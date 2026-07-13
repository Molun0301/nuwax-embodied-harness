//! nuwax 建图板块根模块
//! - `voxels` — 体素地图
//! - `pointclouds` — 点云与占据
//! - `relocalization` — 重定位（ICP）
//! - `loop_closure` — 回环检测 + PGO（自研 Gauss-Newton）
//! - `models` — Costmap 数据类
//! - `utils` — 工具（distance 等）

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod cost_mapper;
pub mod loop_closure;
pub mod mapping_session;
pub mod models;
pub mod occupancy;
pub mod pointclouds;
pub mod relocalization;
pub mod utils;
pub mod voxels;

// 重导出最常用类型
pub use cost_mapper::CostMapper;
pub use loop_closure::PgoOptimizer;
pub use mapping_session::{
    FeedStats, MappingSession, ResourceExhaustedError, SessionError, SessionStats,
    StaleRevisionError, DEFAULT_BOUNDED_CAPACITY, DEFAULT_STALE_THRESHOLD_NS, ODOM_QUANTIZE_MM,
};
pub use models::{Costmap, MapMeta};
pub use occupancy::OccupancyGrid;
pub use pointclouds::PointCloudAccumulator;
