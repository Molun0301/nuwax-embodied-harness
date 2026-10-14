//! OpenStreetMap HTTP and offline raster utilities.

use std::collections::BTreeMap;
use std::io::Cursor;

use image::{DynamicImage, ImageFormat, RgbaImage};
use serde::{Deserialize, Serialize};

use crate::google_maps::{CloudError, CloudMode};

const DEFAULT_TILE_TEMPLATE: &str = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
const DEFAULT_NOMINATIM_URL: &str = "https://nominatim.openstreetmap.org/search";
const DEFAULT_USER_AGENT: &str = "nuwax-mapping-cloud/0.1";

/// Compatibility alias retained for existing callers.
pub type OsmError = CloudError;

/// Slippy-map tile URL template.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OsmTileServer {
    /// Template containing `{z}`, `{x}`, and `{y}` placeholders.
    pub template: String,
}

impl OsmTileServer {
    /// Construct the public OpenStreetMap tile endpoint.
    #[must_use]
    pub fn default_osm() -> Self {
        Self {
            template: DEFAULT_TILE_TEMPLATE.to_string(),
        }
    }

    /// Expand the template for one tile coordinate.
    #[must_use]
    pub fn url(&self, z: u8, x: u32, y: u32) -> String {
        self.template
            .replace("{z}", &z.to_string())
            .replace("{x}", &x.to_string())
            .replace("{y}", &y.to_string())
    }
}

/// A validated PNG tile and its slippy-map coordinate.
#[derive(Debug, Clone)]
pub struct OsmTile {
    /// Zoom level.
    pub z: u8,
    /// Tile column.
    pub x: u32,
    /// Tile row.
    pub y: u32,
    /// Original, standards-compliant PNG bytes.
    pub bytes: Vec<u8>,
}

/// One Nominatim search result.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OsmPlace {
    /// Latitude.
    #[serde(deserialize_with = "deserialize_f64_from_string")]
    pub lat: f64,
    /// Longitude.
    #[serde(rename = "lon", deserialize_with = "deserialize_f64_from_string")]
    pub lng: f64,
    /// Full display name.
    pub display_name: String,
    /// OSM element type such as `node`, `way`, or `relation`.
    pub osm_type: String,
}

/// OSM tile and Nominatim client.
pub struct OsmClient {
    /// Tile server configuration.
    pub tile_server: OsmTileServer,
    /// Shared HTTP client.
    pub http: reqwest::Client,
    nominatim_url: String,
    user_agent: String,
}

impl OsmClient {
    /// Construct from optional `NUWAX_OSM_TILE_URL_TEMPLATE`,
    /// `NUWAX_NOMINATIM_URL`, and `NUWAX_OSM_USER_AGENT` configuration.
    #[must_use]
    pub fn new() -> Self {
        let tile_template = std::env::var("NUWAX_OSM_TILE_URL_TEMPLATE")
            .unwrap_or_else(|_| DEFAULT_TILE_TEMPLATE.to_string());
        let nominatim_url = std::env::var("NUWAX_NOMINATIM_URL")
            .unwrap_or_else(|_| DEFAULT_NOMINATIM_URL.to_string());
        let user_agent = std::env::var("NUWAX_OSM_USER_AGENT")
            .unwrap_or_else(|_| DEFAULT_USER_AGENT.to_string());
        Self::with_endpoints(tile_template, nominatim_url, user_agent)
    }

    /// Construct with explicit endpoints for a proxy or local integration
    /// server.
    #[must_use]
    pub fn with_endpoints(
        tile_template: impl Into<String>,
        nominatim_url: impl Into<String>,
        user_agent: impl Into<String>,
    ) -> Self {
        Self {
            tile_server: OsmTileServer {
                template: tile_template.into(),
            },
            http: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(60))
                .build()
                .expect("reqwest client configuration is valid"),
            nominatim_url: nominatim_url.into(),
            user_agent: user_agent.into(),
        }
    }

    /// Download and validate one PNG tile.
    ///
    /// # Errors
    /// Returns coordinate, HTTP, or PNG validation errors.
    pub async fn fetch_tile(&self, z: u8, x: u32, y: u32) -> Result<OsmTile, OsmError> {
        validate_tile_coordinate(z, x, y)?;
        let response = self
            .http
            .get(self.tile_server.url(z, x, y))
            .header(reqwest::header::USER_AGENT, &self.user_agent)
            .send()
            .await
            .map_err(|error| CloudError::Http(error.to_string()))?;
        let status = response.status();
        let bytes = response
            .bytes()
            .await
            .map_err(|error| CloudError::Http(error.to_string()))?;
        if !status.is_success() {
            return Err(CloudError::HttpStatus {
                status: status.as_u16(),
                message: String::from_utf8_lossy(&bytes).into_owned(),
            });
        }
        decode_png(&bytes)?;
        Ok(OsmTile {
            z,
            x,
            y,
            bytes: bytes.to_vec(),
        })
    }

    /// Execute a real Nominatim text search.
    ///
    /// # Errors
    /// Returns HTTP or response-decoding errors.
    pub async fn nominatim_search(&self, query: &str) -> Result<Vec<OsmPlace>, OsmError> {
        let response = self
            .http
            .get(&self.nominatim_url)
            .header(reqwest::header::USER_AGENT, &self.user_agent)
            .query(&[("q", query), ("format", "jsonv2")])
            .send()
            .await
            .map_err(|error| CloudError::Http(error.to_string()))?;
        let status = response.status();
        let body = response
            .text()
            .await
            .map_err(|error| CloudError::Http(error.to_string()))?;
        if !status.is_success() {
            return Err(CloudError::HttpStatus {
                status: status.as_u16(),
                message: body,
            });
        }
        serde_json::from_str(&body).map_err(|error| CloudError::Parse(error.to_string()))
    }
}

impl Default for OsmClient {
    fn default() -> Self {
        Self::new()
    }
}

/// Search Nominatim in live mode.
///
/// # Errors
/// Returns [`CloudError::Offline`] in offline mode, or propagates live HTTP
/// and decoding errors.
pub async fn nominatim_search(query: &str, mode: CloudMode) -> Result<Vec<OsmPlace>, CloudError> {
    match mode {
        CloudMode::Live => OsmClient::new().nominatim_search(query).await,
        CloudMode::Offline | CloudMode::Mock => Err(CloudError::Offline),
    }
}

/// Download an OSM tile in live mode.
///
/// # Errors
/// Returns [`CloudError::Offline`] in offline mode, or propagates live HTTP
/// and PNG validation errors.
pub async fn download_tile(z: u8, x: u32, y: u32, mode: CloudMode) -> Result<Vec<u8>, CloudError> {
    match mode {
        CloudMode::Live => Ok(OsmClient::new().fetch_tile(z, x, y).await?.bytes),
        CloudMode::Offline | CloudMode::Mock => Err(CloudError::Offline),
    }
}

/// Decode and stitch a complete rectangular tile set into one PNG raster.
///
/// Tile `(min_x, min_y)` is placed at the top left. Increasing `x` moves
/// right and increasing `y` moves down, matching the slippy-map convention.
/// Every coordinate in the bounding rectangle must be present exactly once.
///
/// # Errors
/// Returns an error for an empty set, invalid coordinates, duplicate or
/// missing cells, corrupt PNG data, inconsistent tile dimensions, overflow,
/// or output encoding failure.
pub fn stitch_tiles(tiles: &[(u32, u32, Vec<u8>)], z: u8) -> Result<Vec<u8>, CloudError> {
    if tiles.is_empty() {
        return Err(CloudError::Parse("cannot stitch an empty tile set".into()));
    }

    let mut decoded = BTreeMap::new();
    for (x, y, bytes) in tiles {
        validate_tile_coordinate(z, *x, *y)?;
        let image = decode_png(bytes)?.to_rgba8();
        if decoded.insert((*x, *y), image).is_some() {
            return Err(CloudError::Parse(format!(
                "duplicate tile coordinate ({x}, {y})"
            )));
        }
    }

    let min_x = decoded.keys().map(|(x, _)| *x).min().expect("not empty");
    let max_x = decoded.keys().map(|(x, _)| *x).max().expect("not empty");
    let min_y = decoded.keys().map(|(_, y)| *y).min().expect("not empty");
    let max_y = decoded.keys().map(|(_, y)| *y).max().expect("not empty");
    let columns = max_x - min_x + 1;
    let rows = max_y - min_y + 1;
    let expected = usize::try_from(u64::from(columns) * u64::from(rows))
        .map_err(|_| CloudError::Parse("tile rectangle is too large".into()))?;
    if decoded.len() != expected {
        let missing = (min_y..=max_y)
            .flat_map(|y| (min_x..=max_x).map(move |x| (x, y)))
            .filter(|coordinate| !decoded.contains_key(coordinate))
            .map(|(x, y)| format!("({x},{y})"))
            .collect::<Vec<_>>()
            .join(", ");
        return Err(CloudError::Parse(format!(
            "missing tile coordinates: {missing}"
        )));
    }

    let (tile_width, tile_height) = decoded.values().next().expect("not empty").dimensions();
    if tile_width == 0 || tile_height == 0 {
        return Err(CloudError::Parse("tile dimensions must be non-zero".into()));
    }
    for ((x, y), image) in &decoded {
        if image.dimensions() != (tile_width, tile_height) {
            return Err(CloudError::Parse(format!(
                "tile ({x}, {y}) dimensions {:?} differ from ({tile_width}, {tile_height})",
                image.dimensions()
            )));
        }
    }

    let output_width = tile_width
        .checked_mul(columns)
        .ok_or_else(|| CloudError::Parse("stitched width overflow".into()))?;
    let output_height = tile_height
        .checked_mul(rows)
        .ok_or_else(|| CloudError::Parse("stitched height overflow".into()))?;
    let mut output = RgbaImage::new(output_width, output_height);
    for ((x, y), tile) in decoded {
        let left = i64::from((x - min_x) * tile_width);
        let top = i64::from((y - min_y) * tile_height);
        image::imageops::replace(&mut output, &tile, left, top);
    }

    let mut bytes = Cursor::new(Vec::new());
    DynamicImage::ImageRgba8(output)
        .write_to(&mut bytes, ImageFormat::Png)
        .map_err(|error| CloudError::Parse(error.to_string()))?;
    Ok(bytes.into_inner())
}

fn validate_tile_coordinate(z: u8, x: u32, y: u32) -> Result<(), CloudError> {
    if z > 31 {
        return Err(CloudError::Parse(format!("unsupported tile zoom {z}")));
    }
    let edge = 1_u64 << z;
    if u64::from(x) >= edge || u64::from(y) >= edge {
        return Err(CloudError::Parse(format!(
            "tile ({x}, {y}) is outside zoom {z} bounds 0..{}",
            edge - 1
        )));
    }
    Ok(())
}

fn decode_png(bytes: &[u8]) -> Result<DynamicImage, CloudError> {
    image::load_from_memory_with_format(bytes, ImageFormat::Png)
        .map_err(|error| CloudError::Parse(format!("invalid PNG tile: {error}")))
}

fn deserialize_f64_from_string<'de, D>(deserializer: D) -> Result<f64, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = String::deserialize(deserializer)?;
    value.parse().map_err(serde::de::Error::custom)
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{Rgba, RgbaImage};

    fn png(color: [u8; 4]) -> Vec<u8> {
        let image = RgbaImage::from_pixel(2, 2, Rgba(color));
        let mut bytes = Cursor::new(Vec::new());
        DynamicImage::ImageRgba8(image)
            .write_to(&mut bytes, ImageFormat::Png)
            .unwrap();
        bytes.into_inner()
    }

    #[test]
    fn default_tile_url() {
        assert_eq!(
            OsmTileServer::default_osm().url(5, 10, 20),
            "https://tile.openstreetmap.org/5/10/20.png"
        );
    }

    #[tokio::test]
    async fn offline_mode_does_not_download() {
        assert!(matches!(
            download_tile(5, 10, 20, CloudMode::Offline).await,
            Err(CloudError::Offline)
        ));
        assert!(matches!(
            nominatim_search("Springfield", CloudMode::Mock).await,
            Err(CloudError::Offline)
        ));
    }

    #[test]
    fn stitch_places_tiles_by_coordinate() {
        let output = stitch_tiles(
            &[
                (11, 21, png([4, 0, 0, 255])),
                (10, 20, png([1, 0, 0, 255])),
                (10, 21, png([3, 0, 0, 255])),
                (11, 20, png([2, 0, 0, 255])),
            ],
            6,
        )
        .unwrap();
        let decoded = decode_png(&output).unwrap().to_rgba8();
        assert_eq!(decoded.dimensions(), (4, 4));
        assert_eq!(decoded.get_pixel(0, 0).0, [1, 0, 0, 255]);
        assert_eq!(decoded.get_pixel(3, 0).0, [2, 0, 0, 255]);
        assert_eq!(decoded.get_pixel(0, 3).0, [3, 0, 0, 255]);
        assert_eq!(decoded.get_pixel(3, 3).0, [4, 0, 0, 255]);
    }

    #[test]
    fn stitch_rejects_missing_and_corrupt_tiles() {
        let missing = stitch_tiles(
            &[(10, 20, png([1, 2, 3, 255])), (11, 21, png([4, 5, 6, 255]))],
            6,
        );
        assert!(matches!(missing, Err(CloudError::Parse(message)) if message.contains("missing")));

        let corrupt = stitch_tiles(&[(10, 20, b"not a png".to_vec())], 6);
        assert!(matches!(corrupt, Err(CloudError::Parse(message)) if message.contains("PNG")));
    }
}
