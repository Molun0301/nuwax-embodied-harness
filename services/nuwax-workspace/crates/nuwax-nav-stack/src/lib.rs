// Copyright 2026 Nuwax.
// SPDX-License-Identifier: GPL-2.0-or-later

//! 固定三维与二维导航组合；规划、控制算法由各自的上游库提供。
#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod bin_support;
pub mod error;
pub mod production_pipeline;
mod topics;
mod task;
#[cfg(feature = "lcm-runtime")]
mod lcm_dataflow;
#[cfg(feature = "lcm-runtime")]
mod map_pose;
#[cfg(all(feature = "mls", feature = "simple-2d"))]
mod runtime_owners;
#[cfg(feature = "mls")]
pub mod surface_config;
#[cfg(feature = "mls")]
pub mod surface_map;
#[cfg(feature = "mls")]
pub mod surface_oriented;
#[cfg(feature = "mls")]
mod surface_chunks;

pub use error::{NavStackError, NavStackResult};
pub use production_pipeline::ProductionPipeline;

/// 程序版本。
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
/// 程序名称。
pub const CRATE_NAME: &str = env!("CARGO_PKG_NAME");

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn version_metadata_is_set() {
        assert!(VERSION.starts_with(|c: char| c.is_ascii_digit()));
        assert!(CRATE_NAME.starts_with("nuwax-"));
    }
}

#[cfg(test)]
pub(crate) fn static_map_follower(config:nuwax_nav_runtime::SurfaceFollowerConfig)->Result<nuwax_nav_runtime::SurfaceFollower,nuwax_nav_runtime::RuntimeError>{
    nuwax_nav_runtime::SurfaceFollower::with_map_initializer(config,Some(|map,obstacles,ground,boundary,forbidden,body|{
        map.seed_occupied(&obstacles.iter().chain(ground).copied().collect::<Vec<_>>(),body);
        map.update_surface(&[],&[],boundary,forbidden)
    }))
}

#[cfg(test)]
mod scan_numerical_fixture;
