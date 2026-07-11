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
    compute_fpfh_stub, filter_upright, gravity_tilt_deg, is_upright, multiscale_icp,
    ransac_align_stub, relocation_pipeline, wall_subset_mask, IcpConfig, IcpResult, IcpSolver,
    GRAVITY_TILT_MAX_DEG, WALL_NORMAL_Z_MAX,
};
