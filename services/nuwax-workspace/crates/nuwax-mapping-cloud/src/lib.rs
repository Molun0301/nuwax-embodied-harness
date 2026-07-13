//! nuwax 地图云服务
//!
//! The default `cloud-enabled` feature exposes explicit live HTTP clients and
//! offline raster utilities. Google requests require `GOOGLE_MAPS_API_KEY`;
//! OSM endpoints and user-agent values can be overridden with `NUWAX_OSM_*`
//! environment variables. Offline mode never fabricates network results.
//!
//! 模块：
//! - `google_maps`: `geocode` / `places_nearby` / `GoogleMapsClient`
//! - `osm`: `nominatim_search` / `download_tile` / `stitch_tiles` / `OsmClient`
//! - `coords`: WGS84 ↔ Web Mercator ↔ slippy tile 坐标转换
//!
//! 错误类型：
//! - `CloudError`: 统一 cloud 错误（含 `ApiKeyMissing` / `Http` / `Parse` / `Disabled`）
//! - `CloudMode`: explicit offline/live network policy
//! - `CloudMapError`: feature-off compatibility error (`Disabled`)

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
    geocode, places_nearby, CloudError, CloudMode, GeocodeComponents, GeocodeResult,
    GoogleMapsClient, LatLng, Place,
};
#[cfg(feature = "cloud-enabled")]
pub use osm::{
    download_tile, nominatim_search, stitch_tiles, OsmClient, OsmPlace, OsmTile, OsmTileServer,
};

/// Cloud map 错误（始终可用，feature off 时返 `Disabled`）
#[derive(Debug, Error)]
pub enum CloudMapError {
    /// Feature 关闭时所有 cloud map 操作返此错误
    #[error("cloud map disabled (compile with --features cloud-enabled)")]
    Disabled,
}

/// 返回 `Err(CloudMapError::Disabled)`（feature off 时）
#[cfg(not(feature = "cloud-enabled"))]
pub fn google_maps_search(_query: &str) -> Result<(), CloudMapError> {
    Err(CloudMapError::Disabled)
}

/// 返回 `Err(CloudMapError::Disabled)`（feature off 时）
#[cfg(not(feature = "cloud-enabled"))]
pub fn osm_fetch_tile(_z: u8, _x: u32, _y: u32) -> Result<(), CloudMapError> {
    Err(CloudMapError::Disabled)
}
