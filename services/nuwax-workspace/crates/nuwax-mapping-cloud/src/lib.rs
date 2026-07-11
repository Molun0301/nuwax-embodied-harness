//! nuwax 地图云服务
//!
//! **P1 #5 (rework22 Lane D)**:cloud 服务(Google Maps / OSM)依赖外部 API key 与 aarch64 验证,
//! Go2 单机部署场景下不需要,本 crate **默认禁用**。
//! 启用时需要在 `Cargo.toml` 加 `--features cloud-enabled`。
//!
//! 当 feature off 时,模块导出 fallback 类型 + 错误码 `CloudMapError::Disabled`,
//! 替代之前的"返 placeholder + 空 bytes"行为（避免调用方误以为云地图可用）。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use thiserror::Error;

#[cfg(feature = "cloud-enabled")]
pub mod google_maps;
#[cfg(feature = "cloud-enabled")]
pub mod osm;

#[cfg(feature = "cloud-enabled")]
pub use google_maps::{GoogleMapsClient, Place};
#[cfg(feature = "cloud-enabled")]
pub use osm::{OsmTile, OsmTileServer};

/// Cloud map 错误（始终可用）
#[derive(Debug, Error)]
pub enum CloudMapError {
    /// Feature 默认关闭时所有 cloud map 操作返此错误
    #[error("cloud map disabled (compile with --features cloud-enabled)")]
    Disabled,
    /// 占位：底层 client 调用未实装（仅占位 stub）
    #[error("not implemented: {0}")]
    NotImplemented(&'static str),
}

/// 返回 `Err(CloudMapError::Disabled)`（默认 feature off 时）
#[must_use]
pub fn google_maps_search(_query: &str) -> Result<(), CloudMapError> {
    Err(CloudMapError::Disabled)
}

/// 返回 `Err(CloudMapError::Disabled)`（默认 feature off 时）
#[must_use]
pub fn osm_fetch_tile(_z: u8, _x: u32, _y: u32) -> Result<(), CloudMapError> {
    Err(CloudMapError::Disabled)
}