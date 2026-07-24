//! Cached current-location OpenStreetMap workflow.
//!
//! This module owns tile selection, raster assembly, position-marker drawing,
//! cache refresh, pixel/geographic conversion, and the visual-query boundary.
//! The model implementation remains an injected owner rather than being
//! duplicated inside the mapping crate.

use std::io::Cursor;
use std::path::Path;

use image::{DynamicImage, ImageFormat, Rgba, RgbaImage};

use crate::google_maps::{CloudError, LatLng};
use crate::osm::{stitch_tiles, OsmClient};

const MAX_MERCATOR_LATITUDE: f64 = 85.051_128_78;

/// Runtime configuration for a cached current-location map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurrentLocationConfig {
    /// Slippy-map zoom level.
    pub zoom: u8,
    /// Number of tiles on each side of the square raster.
    pub tiles_per_side: u32,
    /// Central width ratio in which the current raster remains reusable.
    pub center_width_ratio: f64,
}

impl Default for CurrentLocationConfig {
    fn default() -> Self {
        Self {
            zoom: 15,
            tiles_per_side: 6,
            center_width_ratio: 0.4,
        }
    }
}

impl CurrentLocationConfig {
    fn validate(self) -> Result<Self, CloudError> {
        if self.zoom > 31 {
            return Err(CloudError::Parse(format!(
                "unsupported current-location zoom {}",
                self.zoom
            )));
        }
        if self.tiles_per_side == 0 || self.tiles_per_side > 32 {
            return Err(CloudError::Parse("tiles_per_side must be in 1..=32".into()));
        }
        if !self.center_width_ratio.is_finite()
            || !(0.0..=1.0).contains(&self.center_width_ratio)
            || self.center_width_ratio == 0.0
        {
            return Err(CloudError::Parse(
                "center_width_ratio must be finite and in (0, 1]".into(),
            ));
        }
        Ok(self)
    }
}

/// A stitched map raster and the slippy-map geometry needed for conversion.
#[derive(Debug, Clone)]
pub struct MapImage {
    png: Vec<u8>,
    center: LatLng,
    zoom: u8,
    tiles_per_side: u32,
    start_tile_x: u32,
    start_tile_y: u32,
    tile_size: u32,
    width: u32,
    height: u32,
}

impl MapImage {
    /// Encoded PNG bytes, including the current-position marker.
    #[must_use]
    pub fn png(&self) -> &[u8] {
        &self.png
    }

    /// Geographic center used to fetch this raster.
    #[must_use]
    pub const fn center(&self) -> LatLng {
        self.center
    }

    /// Raster width in pixels.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Raster height in pixels.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }

    /// Zoom level used for this raster.
    #[must_use]
    pub const fn zoom(&self) -> u8 {
        self.zoom
    }

    /// Number of tiles on each side of this raster.
    #[must_use]
    pub const fn tiles_per_side(&self) -> u32 {
        self.tiles_per_side
    }

    /// Convert a geographic location to raster pixel coordinates.
    ///
    /// Coordinates outside the raster deliberately produce pixels outside the
    /// image, matching the Dimos cache-refresh calculation.
    pub fn lat_lng_to_pixel(&self, position: LatLng) -> Result<(i32, i32), CloudError> {
        validate_position(position)?;
        let (tile_x, tile_y) = lat_lng_to_fractional_tile(position, self.zoom);
        let x = (tile_x - f64::from(self.start_tile_x)) * f64::from(self.tile_size);
        let y = (tile_y - f64::from(self.start_tile_y)) * f64::from(self.tile_size);
        if !x.is_finite() || !y.is_finite() {
            return Err(CloudError::Parse(
                "geographic-to-pixel conversion was non-finite".into(),
            ));
        }
        Ok((x.trunc() as i32, y.trunc() as i32))
    }

    /// Convert raster pixel coordinates to WGS84 latitude/longitude.
    #[must_use]
    pub fn pixel_to_lat_lng(&self, pixel: (u32, u32)) -> LatLng {
        let tile_x = f64::from(self.start_tile_x) + f64::from(pixel.0) / f64::from(self.tile_size);
        let tile_y = f64::from(self.start_tile_y) + f64::from(pixel.1) / f64::from(self.tile_size);
        fractional_tile_to_lat_lng(tile_x, tile_y, self.zoom)
    }

    /// Persist the currently encoded PNG for offline inspection.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), CloudError> {
        std::fs::write(path, &self.png).map_err(|error| CloudError::Http(error.to_string()))
    }
}

/// Pixel-space match returned by a visual map-query owner.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MapPixelMatch {
    /// Horizontal pixel coordinate.
    pub x: u32,
    /// Vertical pixel coordinate.
    pub y: u32,
    /// Optional model-provided description.
    pub description: Option<String>,
}

/// Geographic match produced after validating and converting a pixel match.
#[derive(Debug, Clone, PartialEq)]
pub struct MapPositionMatch {
    /// WGS84 result.
    pub position: LatLng,
    /// Optional model-provided description.
    pub description: Option<String>,
}

/// Explicit boundary implemented by the visual-model owner.
pub trait MapImageQuery {
    /// Query one map PNG. `None` means the requested feature was not found.
    fn query_map(&mut self, png: &[u8], prompt: &str) -> Result<Option<MapPixelMatch>, CloudError>;
}

/// Stateful owner for the cached map around the latest robot position.
pub struct CurrentLocationMap {
    client: OsmClient,
    config: CurrentLocationConfig,
    position: Option<LatLng>,
    cached: Option<MapImage>,
}

impl CurrentLocationMap {
    /// Construct a workflow using an explicit OSM client and configuration.
    pub fn new(client: OsmClient, config: CurrentLocationConfig) -> Result<Self, CloudError> {
        Ok(Self {
            client,
            config: config.validate()?,
            position: None,
            cached: None,
        })
    }

    /// Update the current robot position.
    pub fn update_position(&mut self, position: LatLng) -> Result<(), CloudError> {
        validate_position(position)?;
        self.position = Some(position);
        Ok(())
    }

    /// Drop the cached raster while preserving configuration and position.
    pub fn invalidate(&mut self) {
        self.cached = None;
    }

    /// Return the cached raster, fetching a new tile grid when needed.
    pub async fn current_map(&mut self) -> Result<&MapImage, CloudError> {
        let position = self
            .position
            .ok_or_else(|| CloudError::Parse("current position has not been set".into()))?;
        let refresh = match &self.cached {
            Some(map) => position_is_outside_center(map, position, self.config.center_width_ratio)?,
            None => true,
        };
        if refresh {
            self.cached = Some(fetch_map(&self.client, self.config, position).await?);
        }
        self.cached
            .as_ref()
            .ok_or_else(|| CloudError::Parse("current map cache was not populated".into()))
    }

    /// Query the current map through the injected model owner and convert its
    /// pixel result to WGS84 coordinates.
    pub async fn query_one_position<Q: MapImageQuery>(
        &mut self,
        model: &mut Q,
        prompt: &str,
    ) -> Result<Option<MapPositionMatch>, CloudError> {
        let map = self.current_map().await?;
        let Some(result) = model.query_map(map.png(), prompt)? else {
            return Ok(None);
        };
        if result.x >= map.width || result.y >= map.height {
            return Err(CloudError::Parse(format!(
                "map-query pixel ({}, {}) is outside {}x{}",
                result.x, result.y, map.width, map.height
            )));
        }
        Ok(Some(MapPositionMatch {
            position: map.pixel_to_lat_lng((result.x, result.y)),
            description: result.description,
        }))
    }
}

async fn fetch_map(
    client: &OsmClient,
    config: CurrentLocationConfig,
    position: LatLng,
) -> Result<MapImage, CloudError> {
    let (center_x, center_y) = lat_lng_to_fractional_tile(position, config.zoom);
    let half = f64::from(config.tiles_per_side) / 2.0;
    let start_x = (center_x - half).floor() as i64;
    let start_y = (center_y - half).floor() as i64;
    let edge = 1_i64 << config.zoom;
    let end_x = start_x + i64::from(config.tiles_per_side);
    let end_y = start_y + i64::from(config.tiles_per_side);
    if start_x < 0 || start_y < 0 || end_x > edge || end_y > edge {
        return Err(CloudError::Parse(
            "requested current-location tile grid crosses the slippy-map boundary".into(),
        ));
    }

    let mut tiles = Vec::with_capacity(
        usize::try_from(config.tiles_per_side * config.tiles_per_side)
            .map_err(|_| CloudError::Parse("tile count overflow".into()))?,
    );
    for row in 0..config.tiles_per_side {
        for column in 0..config.tiles_per_side {
            let x = u32::try_from(start_x + i64::from(column))
                .map_err(|_| CloudError::Parse("tile x conversion failed".into()))?;
            let y = u32::try_from(start_y + i64::from(row))
                .map_err(|_| CloudError::Parse("tile y conversion failed".into()))?;
            let tile = client.fetch_tile(config.zoom, x, y).await?;
            tiles.push((x, y, tile.bytes));
        }
    }

    let stitched = stitch_tiles(&tiles, config.zoom)?;
    let mut raster = image::load_from_memory_with_format(&stitched, ImageFormat::Png)
        .map_err(|error| CloudError::Parse(error.to_string()))?
        .to_rgba8();
    let (width, height) = raster.dimensions();
    if width == 0
        || height == 0
        || width % config.tiles_per_side != 0
        || height % config.tiles_per_side != 0
    {
        return Err(CloudError::Parse(
            "stitched tile raster has incompatible dimensions".into(),
        ));
    }
    let tile_size = width / config.tiles_per_side;
    if tile_size != height / config.tiles_per_side {
        return Err(CloudError::Parse("OSM tiles must be square".into()));
    }
    let start_tile_x =
        u32::try_from(start_x).map_err(|_| CloudError::Parse("tile x conversion failed".into()))?;
    let start_tile_y =
        u32::try_from(start_y).map_err(|_| CloudError::Parse("tile y conversion failed".into()))?;
    let marker_x = ((center_x - f64::from(start_tile_x)) * f64::from(tile_size)).round() as i32;
    let marker_y = ((center_y - f64::from(start_tile_y)) * f64::from(tile_size)).round() as i32;
    draw_position_marker(&mut raster, marker_x, marker_y);
    let png = encode_png(raster)?;

    Ok(MapImage {
        png,
        center: position,
        zoom: config.zoom,
        tiles_per_side: config.tiles_per_side,
        start_tile_x,
        start_tile_y,
        tile_size,
        width,
        height,
    })
}

fn position_is_outside_center(
    map: &MapImage,
    position: LatLng,
    center_width_ratio: f64,
) -> Result<bool, CloudError> {
    let (x, y) = map.lat_lng_to_pixel(position)?;
    let x = f64::from(x);
    let y = f64::from(y);
    let min_x = f64::from(map.width) * (0.5 - center_width_ratio / 2.0);
    let max_x = f64::from(map.width) * (0.5 + center_width_ratio / 2.0);
    let min_y = f64::from(map.height) * (0.5 - center_width_ratio / 2.0);
    let max_y = f64::from(map.height) * (0.5 + center_width_ratio / 2.0);
    Ok(x < min_x || x > max_x || y < min_y || y > max_y)
}

fn validate_position(position: LatLng) -> Result<(), CloudError> {
    if !position.lat.is_finite()
        || !position.lng.is_finite()
        || !(-90.0..=90.0).contains(&position.lat)
        || !(-180.0..=180.0).contains(&position.lng)
    {
        return Err(CloudError::Parse(
            "latitude/longitude must be finite WGS84 coordinates".into(),
        ));
    }
    Ok(())
}

fn lat_lng_to_fractional_tile(position: LatLng, zoom: u8) -> (f64, f64) {
    let edge = 2_f64.powi(i32::from(zoom));
    let latitude = position
        .lat
        .clamp(-MAX_MERCATOR_LATITUDE, MAX_MERCATOR_LATITUDE);
    let x = (position.lng + 180.0) / 360.0 * edge;
    let latitude_radians = latitude.to_radians();
    let y = (1.0
        - (latitude_radians.tan() + 1.0 / latitude_radians.cos()).ln() / std::f64::consts::PI)
        / 2.0
        * edge;
    (x, y)
}

fn fractional_tile_to_lat_lng(tile_x: f64, tile_y: f64, zoom: u8) -> LatLng {
    let edge = 2_f64.powi(i32::from(zoom));
    let lng = tile_x / edge * 360.0 - 180.0;
    let latitude_radians = (std::f64::consts::PI * (1.0 - 2.0 * tile_y / edge))
        .sinh()
        .atan();
    LatLng {
        lat: latitude_radians.to_degrees(),
        lng,
    }
}

fn draw_position_marker(image: &mut RgbaImage, center_x: i32, center_y: i32) {
    let radius = 20_i32
        .min((image.width().min(image.height()) / 8) as i32)
        .max(2);
    for y in center_y - radius..=center_y + radius {
        for x in center_x - radius..=center_x + radius {
            if x < 0 || y < 0 || x >= image.width() as i32 || y >= image.height() as i32 {
                continue;
            }
            let distance_squared = (x - center_x).pow(2) + (y - center_y).pow(2);
            if distance_squared <= radius.pow(2) {
                let outline = distance_squared >= (radius - 2).pow(2);
                image.put_pixel(
                    x as u32,
                    y as u32,
                    if outline {
                        Rgba([0, 0, 0, 255])
                    } else {
                        Rgba([255, 0, 0, 255])
                    },
                );
            }
        }
    }
}

fn encode_png(image: RgbaImage) -> Result<Vec<u8>, CloudError> {
    let mut output = Cursor::new(Vec::new());
    DynamicImage::ImageRgba8(image)
        .write_to(&mut output, ImageFormat::Png)
        .map_err(|error| CloudError::Parse(error.to_string()))?;
    Ok(output.into_inner())
}
