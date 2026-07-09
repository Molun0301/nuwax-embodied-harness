//! 重定位
//! 包含：
//! - relocalize — ICP 算法（kiddo KD-tree + SVD）
//! - module — Module trait 包装
//!
//! H-9 stubs（gravity filter + wall subset）以 free fn 暴露在 `relocalize` 模块。

pub mod module;
pub mod relocalize;

pub use module::RelocalizationModule;
pub use relocalize::{
    filter_upright, gravity_tilt_deg, is_upright, wall_subset_mask, IcpConfig, IcpResult,
    IcpSolver, GRAVITY_TILT_MAX_DEG, WALL_NORMAL_Z_MAX,
};
