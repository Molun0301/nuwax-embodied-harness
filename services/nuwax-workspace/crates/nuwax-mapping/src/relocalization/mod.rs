//! 重定位
//! 包含：
//! - relocalize — multi-scale FPFH + deterministic RANSAC + seeded ICP
//! - module — Module trait 包装

mod features;
pub mod module;
pub(crate) mod nearest;
mod ransac;
pub mod relocalize;
pub mod runtime;

pub use features::{
    compute_fpfh, estimate_normals, match_fpfh, voxel_downsample, FeatureConfig, FeatureMatch,
    FpfhDescriptor, FPFH_BINS, FPFH_DIM,
};
pub use module::RelocalizationModule;
pub use ransac::{evaluate_alignment, ransac_align, AlignmentScore, RansacConfig, RansacResult};
pub use relocalize::{
    compute_fpfh_stub, filter_upright, gravity_tilt_deg, is_upright, multiscale_icp,
    ransac_align_stub, relocation_pipeline, relocation_pipeline_with_config, wall_subset_mask,
    IcpConfig, IcpResult, IcpSolver, RelocalizationConfig, RelocalizationResult,
    GRAVITY_TILT_MAX_DEG, WALL_NORMAL_Z_MAX,
};
pub use runtime::{
    RelocalizationRuntime, RelocalizationRuntimeConfig, RelocalizationRuntimeError,
    RelocalizationStatus, RelocalizationUpdate,
};
