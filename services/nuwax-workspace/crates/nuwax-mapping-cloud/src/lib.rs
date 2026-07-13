//! nuwax 地图云服务
//!
//! **P1 #5 (rework22 Lane D)**：cloud 服务算法实装。
//!
//! 默认开启 `cloud-enabled` feature（产品决策：cloud 必须能工作）。Live HTTPS
//! 路径需显式注入 `NUWAX_GOOGLE_MAPS_API_KEY`，否则走 Mock 分支（本地 fixture
//! + parser）保证 dev/CI 可用。
//!
//! 模块：
//! - `google_maps`: `geocode` / `places_nearby` / `GoogleMapsClient`
//! - `osm`: `nominatim_search` / `download_tile` / `stitch_tiles` / `OsmClient`
//! - `coords`: WGS84 ↔ Web Mercator ↔ slippy tile 坐标转换
//!
//! 错误类型：
//! - `CloudError`: 统一 cloud 错误（含 `ApiKeyMissing` / `Http` / `Parse` / `Disabled`）
//! - `CloudMode`: `Mock` / `Live` 切换开关
//! - `CloudMapError`: 兼容旧 API 的轻量错误（仅 `Disabled` / `NotImplemented`）

#![deny(unsafe_code)]
#![warn(missing_docs)]

use thiserror::Error;

#[cfg(feature = "cloud-enabled")]
pub mod coords;
#[cfg(feature = "cloud-enabled")]
pub mod google_maps;
#[cfg(feature = "cloud-enabled")]
pub mod osm;

#[cfg(feature = "cloud-enabled")]
pub use coords::{web_mercator_to_wgs84, wgs84_to_tile, wgs84_to_web_mercator};
#[cfg(feature = "cloud-enabled")]
pub use google_maps::{
    CloudError, CloudMode, GeocodeComponents, GeocodeResult, GoogleMapsClient, LatLng, Place,
    geocode, places_nearby,
};
#[cfg(feature = "cloud-enabled")]
pub use osm::{
    OsmClient, OsmPlace, OsmTile, OsmTileServer, download_tile, nominatim_search, stitch_tiles,
};

/// Cloud map 错误（始终可用，feature off 时返 `Disabled`）
#[derive(Debug, Error)]
pub enum CloudMapError {
    /// Feature 关闭时所有 cloud map 操作返此错误
    #[error("cloud map disabled (compile with --features cloud-enabled)")]
    Disabled,
    /// 占位：底层 client 调用未实装（仅占位 stub）
    #[error("not implemented: {0}")]
    NotImplemented(&'static str),
}

/// 返回 `Err(CloudMapError::Disabled)`（feature off 时）
#[must_use]
pub fn google_maps_search(_query: &str) -> Result<(), CloudMapError> {
    Err(CloudMapError::Disabled)
}

/// 返回 `Err(CloudMapError::Disabled)`（feature off 时）
#[must_use]
pub fn osm_fetch_tile(_z: u8, _x: u32, _y: u32) -> Result<(), CloudMapError> {
    Err(CloudMapError::Disabled)
}