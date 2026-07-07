//! 重定位
//! 包含：
//! - relocalize — ICP 算法（kiddo KD-tree + SVD）
//! - module — Module trait 包装

pub mod module;
pub mod relocalize;

pub use module::RelocalizationModule;
pub use relocalize::{IcpConfig, IcpResult, IcpSolver};
