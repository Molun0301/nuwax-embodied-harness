//! `nuwax-explore-terrain-map-bridge` — 极薄 adapter / profile
//!
//! **adapter-only**:`terrain_map_ext` 算法唯一 owner 是 `nuwax-nav-terrain-map-ext`;
//! 探索侧只提供 `ExplorationTerrainMapExtProfile` 与 re-export,不重新实现 voxel/crop/ground/merge。
//!
//! 禁止:`src/voxel_grid.rs` / `src/crop.rs` / `src/ground.rs` / `src/merge.rs`
//! 与 typed point-cloud 独立类型(只用 `PointCloud`/`TimedPointCloud` re-export)。

#![deny(unsafe_code)]
#![warn(missing_docs)]

mod profile;

pub use nuwax_nav_terrain_map_ext::{
    PointCloud, TerrainMapExtConfig, TerrainMapExtProcessor, TimedPointCloud,
};
pub use profile::ExplorationTerrainMapExtProfile;
