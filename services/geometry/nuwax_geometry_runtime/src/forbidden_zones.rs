//! Persistent map-scoped forbidden-zone loading and conservative rasterization.
//!
//! The physical occupancy grid remains authoritative.  This module only composes an
//! effective grid for navigation/exploration consumers.  A malformed configured file
//! fails closed; a missing file or a map without an entry means that no virtual zones
//! have been configured for that map yet.

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::lcm_runtime::MapState;

pub const FORBIDDEN_ZONES_ENV: &str = "NUWAX_FORBIDDEN_ZONES_PATH";
pub const DEFAULT_FORBIDDEN_ZONES_PATH: &str = "runtime_data/navigation/forbidden_zones.json";

const SUPPORTED_SCHEMA_VERSION: u32 = 1;
const MAX_ZONES: usize = 256;
const MAX_VERTICES: usize = 256;
const GEOMETRY_EPSILON: f64 = 1.0e-9;

pub fn path_from_env() -> PathBuf {
    std::env::var(FORBIDDEN_ZONES_ENV)
        .ok()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_FORBIDDEN_ZONES_PATH))
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq)]
pub struct ZonePoint {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ZoneShape {
    Line,
    Rectangle,
    Circle,
    #[serde(alias = "custom")]
    Polygon,
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ZoneMode {
    Inside,
    Outside,
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
pub struct ForbiddenZone {
    pub id: String,
    pub shape: ZoneShape,
    pub mode: ZoneMode,
    #[serde(default = "default_frame_id")]
    pub frame_id: String,
    #[serde(default, alias = "points")]
    pub vertices: Vec<ZonePoint>,
    #[serde(default)]
    pub center: Option<ZonePoint>,
    #[serde(default)]
    pub radius_m: Option<f64>,
    #[serde(default, alias = "line_width_m")]
    pub stroke_width_m: Option<f64>,
    #[serde(default = "default_enabled")]
    pub enabled: bool,
    #[serde(default)]
    pub metadata: serde_json::Value,
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
pub struct ForbiddenZoneSet {
    #[serde(default)]
    pub map_name: String,
    #[serde(default)]
    pub map_version_id: String,
    pub generation: u64,
    #[serde(default)]
    pub signature: String,
    #[serde(default)]
    pub updated_at: String,
    #[serde(default)]
    pub zones: Vec<ForbiddenZone>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct ForbiddenZoneDocument {
    schema_version: u32,
    #[serde(default)]
    maps: BTreeMap<String, ForbiddenZoneSet>,
}

fn default_frame_id() -> String {
    "map".to_string()
}

const fn default_enabled() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ZoneLoadStatus {
    Empty,
    Valid,
    FailClosed(String),
}

#[derive(Debug, Clone)]
pub struct LoadedForbiddenZones {
    pub signature: String,
    pub generation: u64,
    pub zones: Vec<ForbiddenZone>,
    pub status: ZoneLoadStatus,
}

impl LoadedForbiddenZones {
    fn empty(signature: String) -> Self {
        Self {
            signature,
            generation: 0,
            zones: Vec::new(),
            status: ZoneLoadStatus::Empty,
        }
    }

    fn fail_closed(signature: String, reason: impl Into<String>) -> Self {
        Self {
            signature,
            generation: 0,
            zones: Vec::new(),
            status: ZoneLoadStatus::FailClosed(reason.into()),
        }
    }
}

#[cfg(test)]
pub fn load_for_map(path: &Path, map_version_id: &str) -> LoadedForbiddenZones {
    load_for_map_binding(path, "", map_version_id)
}

pub fn load_for_map_binding(
    path: &Path,
    map_name: &str,
    map_version_id: &str,
) -> LoadedForbiddenZones {
    let map_name = map_name.trim();
    let map_version_id = map_version_id.trim();
    if map_name.is_empty() && map_version_id.is_empty() {
        return LoadedForbiddenZones::fail_closed(
            "invalid:empty-map-binding".to_string(),
            "active map_name is empty",
        );
    }
    let binding_id = if map_name.is_empty() {
        map_version_id
    } else {
        map_name
    };

    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            return LoadedForbiddenZones::empty(format!("missing:{binding_id}"));
        }
        Err(error) => {
            return LoadedForbiddenZones::fail_closed(
                format!("io-error:{binding_id}:{error}"),
                format!("cannot read {}: {error}", path.display()),
            );
        }
    };
    let file_hash = crate::map_checksum::sha256_hex(&bytes);
    let document: ForbiddenZoneDocument = match serde_json::from_slice(&bytes) {
        Ok(document) => document,
        Err(error) => {
            return LoadedForbiddenZones::fail_closed(
                format!("invalid-json:{file_hash}"),
                format!("invalid forbidden-zone JSON: {error}"),
            );
        }
    };
    if document.schema_version != SUPPORTED_SCHEMA_VERSION {
        return LoadedForbiddenZones::fail_closed(
            format!("unsupported-schema:{file_hash}"),
            format!(
                "unsupported forbidden-zone schema_version {}; expected {}",
                document.schema_version, SUPPORTED_SCHEMA_VERSION
            ),
        );
    }
    let selected = if map_name.is_empty() {
        document
            .maps
            .get(map_version_id)
            .map(|zone_set| (map_version_id, zone_set))
    } else {
        document
            .maps
            .get(map_name)
            .map(|zone_set| (map_name, zone_set))
            .or_else(|| {
                if map_version_id.is_empty() {
                    None
                } else {
                    document
                        .maps
                        .get(map_version_id)
                        .map(|zone_set| (map_version_id, zone_set))
                }
            })
    };
    let Some((storage_key, zone_set)) = selected else {
        return LoadedForbiddenZones::empty(format!(
            "absent:{}:{map_version_id}",
            if map_name.is_empty() {
                "legacy"
            } else {
                map_name
            }
        ));
    };
    let stored_map_name = zone_set.map_name.trim();
    if storage_key == map_name && stored_map_name != map_name {
        return LoadedForbiddenZones::fail_closed(
            format!("map-name-mismatch:{file_hash}:{map_name}"),
            format!(
                "forbidden-zone key {map_name:?} contains map_name {:?}",
                zone_set.map_name
            ),
        );
    }
    if !stored_map_name.is_empty() && !map_name.is_empty() && stored_map_name != map_name {
        return LoadedForbiddenZones::fail_closed(
            format!("map-name-mismatch:{file_hash}:{map_name}"),
            format!(
                "forbidden-zone entry for {map_version_id:?} belongs to map {:?}",
                zone_set.map_name
            ),
        );
    }
    if storage_key != map_name && zone_set.map_version_id.trim() != map_version_id {
        return LoadedForbiddenZones::fail_closed(
            format!("map-version-mismatch:{file_hash}:{map_version_id}"),
            format!(
                "forbidden-zone key {map_version_id:?} contains map_version_id {:?}",
                zone_set.map_version_id
            ),
        );
    }
    if let Err(error) = validate_zone_set(zone_set) {
        return LoadedForbiddenZones::fail_closed(
            format!("invalid-zone-set:{file_hash}:{map_version_id}"),
            error,
        );
    }
    let entry_bytes = serde_json::to_vec(zone_set).unwrap_or_default();
    let signature = if zone_set.signature.trim().is_empty() {
        format!(
            "legacy:{storage_key}:{}",
            crate::map_checksum::sha256_hex(&entry_bytes)
        )
    } else {
        zone_set.signature.trim().to_ascii_lowercase()
    };
    LoadedForbiddenZones {
        signature,
        generation: zone_set.generation,
        zones: zone_set.zones.clone(),
        status: if zone_set.zones.iter().any(|zone| zone.enabled) {
            ZoneLoadStatus::Valid
        } else {
            ZoneLoadStatus::Empty
        },
    }
}

fn validate_zone_set(zone_set: &ForbiddenZoneSet) -> Result<(), String> {
    if zone_set.generation == 0 {
        return Err("forbidden-zone generation must be positive".to_string());
    }
    if zone_set.zones.len() > MAX_ZONES {
        return Err(format!(
            "forbidden-zone count {} exceeds limit {MAX_ZONES}",
            zone_set.zones.len()
        ));
    }
    let signature = zone_set.signature.trim();
    if !signature.is_empty()
        && (signature.len() != 64 || !signature.bytes().all(|byte| byte.is_ascii_hexdigit()))
    {
        return Err("forbidden-zone signature must be 64 hexadecimal characters".to_string());
    }
    let mut ids = HashSet::with_capacity(zone_set.zones.len());
    for zone in &zone_set.zones {
        validate_zone(zone)?;
        if !ids.insert(zone.id.trim()) {
            return Err(format!("duplicate forbidden-zone id {:?}", zone.id));
        }
    }
    Ok(())
}

fn validate_zone(zone: &ForbiddenZone) -> Result<(), String> {
    if zone.id.trim().is_empty() {
        return Err("forbidden-zone id must not be empty".to_string());
    }
    if zone.frame_id.trim().trim_start_matches('/') != "map" {
        return Err(format!(
            "forbidden-zone {:?} must use frame_id=map",
            zone.id
        ));
    }
    if zone.vertices.len() > MAX_VERTICES {
        return Err(format!(
            "forbidden-zone {:?} vertex count exceeds {MAX_VERTICES}",
            zone.id
        ));
    }
    if zone
        .vertices
        .iter()
        .any(|point| !point.x.is_finite() || !point.y.is_finite())
        || zone
            .center
            .is_some_and(|point| !point.x.is_finite() || !point.y.is_finite())
        || zone.radius_m.is_some_and(|value| !value.is_finite())
        || zone.stroke_width_m.is_some_and(|value| !value.is_finite())
    {
        return Err(format!(
            "forbidden-zone {:?} contains non-finite geometry",
            zone.id
        ));
    }
    if zone
        .stroke_width_m
        .is_some_and(|value| value <= GEOMETRY_EPSILON)
    {
        return Err(format!(
            "forbidden-zone {:?} stroke_width_m must be positive",
            zone.id
        ));
    }

    match zone.shape {
        ZoneShape::Line => {
            if zone.mode != ZoneMode::Inside {
                return Err(format!(
                    "line forbidden-zone {:?} only supports inside mode",
                    zone.id
                ));
            }
            if zone.vertices.len() != 2 || zone.stroke_width_m.is_none() {
                return Err(format!(
                    "line forbidden-zone {:?} requires two vertices and stroke_width_m",
                    zone.id
                ));
            }
            if squared_distance(zone.vertices[0], zone.vertices[1]) <= GEOMETRY_EPSILON {
                return Err(format!(
                    "line forbidden-zone {:?} endpoints must differ",
                    zone.id
                ));
            }
        }
        ZoneShape::Rectangle => {
            if zone.vertices.len() != 2 {
                return Err(format!(
                    "rectangle forbidden-zone {:?} requires two opposite corners",
                    zone.id
                ));
            }
            let a = zone.vertices[0];
            let b = zone.vertices[1];
            if (a.x - b.x).abs() <= GEOMETRY_EPSILON || (a.y - b.y).abs() <= GEOMETRY_EPSILON {
                return Err(format!(
                    "rectangle forbidden-zone {:?} must have non-zero area",
                    zone.id
                ));
            }
        }
        ZoneShape::Circle => {
            if zone.center.is_none()
                || zone
                    .radius_m
                    .is_none_or(|radius| radius <= GEOMETRY_EPSILON)
            {
                return Err(format!(
                    "circle forbidden-zone {:?} requires center and positive radius_m",
                    zone.id
                ));
            }
        }
        ZoneShape::Polygon => {
            if zone.vertices.len() < 3 {
                return Err(format!(
                    "polygon forbidden-zone {:?} requires at least three vertices",
                    zone.id
                ));
            }
            if polygon_signed_area(&zone.vertices).abs() <= GEOMETRY_EPSILON {
                return Err(format!(
                    "polygon forbidden-zone {:?} must have non-zero area",
                    zone.id
                ));
            }
            if polygon_self_intersects(&zone.vertices) {
                return Err(format!(
                    "polygon forbidden-zone {:?} must not self-intersect",
                    zone.id
                ));
            }
        }
    }
    Ok(())
}

pub fn compose_effective_map(
    raw: &MapState,
    loaded: &LoadedForbiddenZones,
) -> Result<MapState, String> {
    let expected = raw.width as usize * raw.height as usize;
    if raw.width == 0
        || raw.height == 0
        || !raw.resolution.is_finite()
        || raw.resolution <= 0.0
        || raw.data.len() != expected
    {
        return Err("raw occupancy layout is invalid".to_string());
    }
    let mut effective = raw.clone();
    if matches!(loaded.status, ZoneLoadStatus::FailClosed(_)) {
        effective.data.fill(100);
        return Ok(effective);
    }
    let enabled: Vec<&ForbiddenZone> = loaded.zones.iter().filter(|zone| zone.enabled).collect();
    if enabled.is_empty() {
        return Ok(effective);
    }

    let cell_count = effective.data.len();
    let mut forbidden = vec![false; cell_count];
    let outside_zones: Vec<_> = enabled
        .iter()
        .copied()
        .filter(|zone| zone.mode == ZoneMode::Outside)
        .collect();
    if !outside_zones.is_empty() {
        for row in 0..raw.height {
            for col in 0..raw.width {
                let rect = cell_rect(raw, col, row);
                let allowed = outside_zones
                    .iter()
                    .all(|zone| cell_fully_inside_closed_shape(rect, zone, raw.resolution as f64));
                if !allowed {
                    forbidden[cell_index(raw, col, row)] = true;
                }
            }
        }
    }

    for zone in enabled {
        if zone.mode == ZoneMode::Outside {
            continue;
        }
        for row in 0..raw.height {
            for col in 0..raw.width {
                let rect = cell_rect(raw, col, row);
                if cell_forbidden_by_inside_zone(rect, zone, raw.resolution as f64) {
                    forbidden[cell_index(raw, col, row)] = true;
                }
            }
        }
    }

    for (index, is_forbidden) in forbidden.into_iter().enumerate() {
        if is_forbidden {
            effective.data[index] = 100;
        }
    }
    Ok(effective)
}

#[derive(Debug, Clone, Copy)]
struct Rect {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl Rect {
    fn corners(self) -> [ZonePoint; 4] {
        [
            ZonePoint {
                x: self.min_x,
                y: self.min_y,
            },
            ZonePoint {
                x: self.max_x,
                y: self.min_y,
            },
            ZonePoint {
                x: self.max_x,
                y: self.max_y,
            },
            ZonePoint {
                x: self.min_x,
                y: self.max_y,
            },
        ]
    }

    fn center(self) -> ZonePoint {
        ZonePoint {
            x: (self.min_x + self.max_x) * 0.5,
            y: (self.min_y + self.max_y) * 0.5,
        }
    }

    fn contains(self, point: ZonePoint) -> bool {
        point.x >= self.min_x - GEOMETRY_EPSILON
            && point.x <= self.max_x + GEOMETRY_EPSILON
            && point.y >= self.min_y - GEOMETRY_EPSILON
            && point.y <= self.max_y + GEOMETRY_EPSILON
    }
}

fn cell_index(map: &MapState, col: u32, row: u32) -> usize {
    row as usize * map.width as usize + col as usize
}

fn cell_rect(map: &MapState, col: u32, row: u32) -> Rect {
    let resolution = map.resolution as f64;
    let min_x = map.origin_x + col as f64 * resolution;
    let min_y = map.origin_y + row as f64 * resolution;
    Rect {
        min_x,
        min_y,
        max_x: min_x + resolution,
        max_y: min_y + resolution,
    }
}

fn boundary_half_width(zone: &ForbiddenZone, resolution: f64) -> f64 {
    zone.stroke_width_m.unwrap_or(resolution).max(resolution) * 0.5
}

fn cell_forbidden_by_inside_zone(rect: Rect, zone: &ForbiddenZone, resolution: f64) -> bool {
    match zone.shape {
        ZoneShape::Line => {
            segment_rect_distance(zone.vertices[0], zone.vertices[1], rect)
                <= zone.stroke_width_m.unwrap_or(resolution) * 0.5 + GEOMETRY_EPSILON
        }
        ZoneShape::Rectangle => {
            let shape = rectangle_from_corners(zone.vertices[0], zone.vertices[1]);
            rects_intersect(
                rect,
                expand_rect(shape, boundary_half_width(zone, resolution)),
            )
        }
        ZoneShape::Circle => {
            let center = zone.center.expect("validated circle center");
            let radius = zone.radius_m.expect("validated circle radius")
                + boundary_half_width(zone, resolution);
            point_rect_distance(center, rect) <= radius + GEOMETRY_EPSILON
        }
        ZoneShape::Polygon => {
            cell_intersects_polygon(rect, &zone.vertices, boundary_half_width(zone, resolution))
        }
    }
}

fn cell_fully_inside_closed_shape(rect: Rect, zone: &ForbiddenZone, resolution: f64) -> bool {
    let margin = boundary_half_width(zone, resolution);
    match zone.shape {
        ZoneShape::Line => false,
        ZoneShape::Rectangle => {
            let shape = rectangle_from_corners(zone.vertices[0], zone.vertices[1]);
            rect.min_x > shape.min_x + margin
                && rect.max_x < shape.max_x - margin
                && rect.min_y > shape.min_y + margin
                && rect.max_y < shape.max_y - margin
        }
        ZoneShape::Circle => {
            let center = zone.center.expect("validated circle center");
            let inner_radius = zone.radius_m.expect("validated circle radius") - margin;
            inner_radius > 0.0
                && rect
                    .corners()
                    .iter()
                    .all(|point| squared_distance(*point, center) < inner_radius * inner_radius)
        }
        ZoneShape::Polygon => {
            let points = rect.corners();
            points
                .iter()
                .all(|point| point_in_polygon(*point, &zone.vertices))
                && polygon_boundary_distance_to_rect(&zone.vertices, rect) > margin
        }
    }
}

fn rectangle_from_corners(a: ZonePoint, b: ZonePoint) -> Rect {
    Rect {
        min_x: a.x.min(b.x),
        min_y: a.y.min(b.y),
        max_x: a.x.max(b.x),
        max_y: a.y.max(b.y),
    }
}

fn expand_rect(rect: Rect, amount: f64) -> Rect {
    Rect {
        min_x: rect.min_x - amount,
        min_y: rect.min_y - amount,
        max_x: rect.max_x + amount,
        max_y: rect.max_y + amount,
    }
}

fn rects_intersect(a: Rect, b: Rect) -> bool {
    a.min_x <= b.max_x + GEOMETRY_EPSILON
        && a.max_x >= b.min_x - GEOMETRY_EPSILON
        && a.min_y <= b.max_y + GEOMETRY_EPSILON
        && a.max_y >= b.min_y - GEOMETRY_EPSILON
}

fn point_rect_distance(point: ZonePoint, rect: Rect) -> f64 {
    let dx = if point.x < rect.min_x {
        rect.min_x - point.x
    } else if point.x > rect.max_x {
        point.x - rect.max_x
    } else {
        0.0
    };
    let dy = if point.y < rect.min_y {
        rect.min_y - point.y
    } else if point.y > rect.max_y {
        point.y - rect.max_y
    } else {
        0.0
    };
    dx.hypot(dy)
}

fn segment_rect_distance(a: ZonePoint, b: ZonePoint, rect: Rect) -> f64 {
    if rect.contains(a)
        || rect.contains(b)
        || rect_edges(rect)
            .iter()
            .any(|(c, d)| segments_intersect(a, b, *c, *d))
    {
        return 0.0;
    }
    let mut distance = point_rect_distance(a, rect).min(point_rect_distance(b, rect));
    for corner in rect.corners() {
        distance = distance.min(point_segment_distance(corner, a, b));
    }
    distance
}

fn rect_edges(rect: Rect) -> [(ZonePoint, ZonePoint); 4] {
    let [a, b, c, d] = rect.corners();
    [(a, b), (b, c), (c, d), (d, a)]
}

fn cell_intersects_polygon(rect: Rect, polygon: &[ZonePoint], margin: f64) -> bool {
    if rect
        .corners()
        .iter()
        .chain(std::iter::once(&rect.center()))
        .any(|point| point_in_polygon(*point, polygon))
        || polygon.iter().any(|point| rect.contains(*point))
    {
        return true;
    }
    polygon_boundary_distance_to_rect(polygon, rect) <= margin + GEOMETRY_EPSILON
}

fn polygon_boundary_distance_to_rect(polygon: &[ZonePoint], rect: Rect) -> f64 {
    polygon
        .iter()
        .copied()
        .zip(polygon.iter().copied().cycle().skip(1))
        .take(polygon.len())
        .map(|(a, b)| segment_rect_distance(a, b, rect))
        .fold(f64::INFINITY, f64::min)
}

fn point_in_polygon(point: ZonePoint, polygon: &[ZonePoint]) -> bool {
    let mut inside = false;
    for (a, b) in polygon
        .iter()
        .copied()
        .zip(polygon.iter().copied().cycle().skip(1))
        .take(polygon.len())
    {
        if point_segment_distance(point, a, b) <= GEOMETRY_EPSILON {
            return true;
        }
        let crosses = (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
        if crosses {
            inside = !inside;
        }
    }
    inside
}

fn polygon_signed_area(points: &[ZonePoint]) -> f64 {
    points
        .iter()
        .copied()
        .zip(points.iter().copied().cycle().skip(1))
        .take(points.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f64>()
        * 0.5
}

fn polygon_self_intersects(points: &[ZonePoint]) -> bool {
    let edge_count = points.len();
    for first in 0..edge_count {
        let first_next = (first + 1) % edge_count;
        for second in (first + 1)..edge_count {
            let second_next = (second + 1) % edge_count;
            if first == second
                || first_next == second
                || second_next == first
                || (first == 0 && second_next == 0)
            {
                continue;
            }
            if segments_intersect(
                points[first],
                points[first_next],
                points[second],
                points[second_next],
            ) {
                return true;
            }
        }
    }
    false
}

fn orientation(a: ZonePoint, b: ZonePoint, c: ZonePoint) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn segments_intersect(a: ZonePoint, b: ZonePoint, c: ZonePoint, d: ZonePoint) -> bool {
    let o1 = orientation(a, b, c);
    let o2 = orientation(a, b, d);
    let o3 = orientation(c, d, a);
    let o4 = orientation(c, d, b);
    if ((o1 > GEOMETRY_EPSILON && o2 < -GEOMETRY_EPSILON)
        || (o1 < -GEOMETRY_EPSILON && o2 > GEOMETRY_EPSILON))
        && ((o3 > GEOMETRY_EPSILON && o4 < -GEOMETRY_EPSILON)
            || (o3 < -GEOMETRY_EPSILON && o4 > GEOMETRY_EPSILON))
    {
        return true;
    }
    (o1.abs() <= GEOMETRY_EPSILON && point_on_segment(c, a, b))
        || (o2.abs() <= GEOMETRY_EPSILON && point_on_segment(d, a, b))
        || (o3.abs() <= GEOMETRY_EPSILON && point_on_segment(a, c, d))
        || (o4.abs() <= GEOMETRY_EPSILON && point_on_segment(b, c, d))
}

fn point_on_segment(point: ZonePoint, a: ZonePoint, b: ZonePoint) -> bool {
    point.x >= a.x.min(b.x) - GEOMETRY_EPSILON
        && point.x <= a.x.max(b.x) + GEOMETRY_EPSILON
        && point.y >= a.y.min(b.y) - GEOMETRY_EPSILON
        && point.y <= a.y.max(b.y) + GEOMETRY_EPSILON
}

fn point_segment_distance(point: ZonePoint, a: ZonePoint, b: ZonePoint) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let length_squared = dx * dx + dy * dy;
    if length_squared <= GEOMETRY_EPSILON {
        return squared_distance(point, a).sqrt();
    }
    let t = (((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared).clamp(0.0, 1.0);
    let projection = ZonePoint {
        x: a.x + t * dx,
        y: a.y + t * dy,
    };
    squared_distance(point, projection).sqrt()
}

fn squared_distance(a: ZonePoint, b: ZonePoint) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

#[cfg(test)]
mod tests {
    use super::*;

    fn free_map(width: u32, height: u32) -> MapState {
        MapState {
            width,
            height,
            resolution: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "map".to_string(),
            revision: 7,
            timestamp_ns: 11,
            data: vec![0; width as usize * height as usize],
        }
    }

    fn loaded(zones: Vec<ForbiddenZone>) -> LoadedForbiddenZones {
        LoadedForbiddenZones {
            signature: "test".to_string(),
            generation: 1,
            status: ZoneLoadStatus::Valid,
            zones,
        }
    }

    fn zone(shape: ZoneShape, mode: ZoneMode, vertices: Vec<ZonePoint>) -> ForbiddenZone {
        ForbiddenZone {
            id: "zone".to_string(),
            shape,
            mode,
            frame_id: "map".to_string(),
            vertices,
            center: None,
            radius_m: None,
            stroke_width_m: None,
            enabled: true,
            metadata: serde_json::json!({}),
        }
    }

    #[test]
    fn diagonal_line_uses_conservative_supercover_cells() {
        let mut line = zone(
            ZoneShape::Line,
            ZoneMode::Inside,
            vec![ZonePoint { x: 0.1, y: 0.1 }, ZonePoint { x: 3.9, y: 3.9 }],
        );
        line.stroke_width_m = Some(0.2);
        let map = compose_effective_map(&free_map(4, 4), &loaded(vec![line])).unwrap();
        for index in [0_usize, 5, 10, 15] {
            assert_eq!(map.data[index], 100);
        }
    }

    #[test]
    fn rectangle_inside_marks_interior_and_boundary() {
        let rectangle = zone(
            ZoneShape::Rectangle,
            ZoneMode::Inside,
            vec![ZonePoint { x: 2.0, y: 2.0 }, ZonePoint { x: 4.0, y: 4.0 }],
        );
        let map = compose_effective_map(&free_map(6, 6), &loaded(vec![rectangle])).unwrap();
        assert_eq!(map.data[3 + 3 * 6], 100);
        assert_eq!(map.data[0], 0);
    }

    #[test]
    fn outside_zones_intersect_their_allowed_interiors() {
        let left = zone(
            ZoneShape::Rectangle,
            ZoneMode::Outside,
            vec![ZonePoint { x: 0.0, y: 0.0 }, ZonePoint { x: 5.0, y: 5.0 }],
        );
        let right = zone(
            ZoneShape::Rectangle,
            ZoneMode::Outside,
            vec![ZonePoint { x: 2.0, y: 0.0 }, ZonePoint { x: 7.0, y: 5.0 }],
        );
        let map = compose_effective_map(&free_map(7, 5), &loaded(vec![left, right])).unwrap();
        assert_eq!(map.data[3 + 2 * 7], 0);
        assert_eq!(map.data[0], 100);
        assert_eq!(map.data[6], 100);
    }

    #[test]
    fn circle_outside_keeps_only_cells_fully_inside() {
        let mut circle = zone(ZoneShape::Circle, ZoneMode::Outside, Vec::new());
        circle.center = Some(ZonePoint { x: 2.5, y: 2.5 });
        circle.radius_m = Some(2.25);
        let map = compose_effective_map(&free_map(5, 5), &loaded(vec![circle])).unwrap();
        assert_eq!(map.data[2 + 2 * 5], 0);
        assert_eq!(map.data[0], 100);
    }

    #[test]
    fn concave_polygon_inside_is_rasterized_without_filling_the_notch() {
        let polygon = zone(
            ZoneShape::Polygon,
            ZoneMode::Inside,
            vec![
                ZonePoint { x: 0.0, y: 0.0 },
                ZonePoint { x: 4.0, y: 0.0 },
                ZonePoint { x: 4.0, y: 1.0 },
                ZonePoint { x: 1.0, y: 1.0 },
                ZonePoint { x: 1.0, y: 4.0 },
                ZonePoint { x: 0.0, y: 4.0 },
            ],
        );
        let map = compose_effective_map(&free_map(5, 5), &loaded(vec![polygon])).unwrap();
        assert_eq!(map.data[0], 100);
        assert_eq!(map.data[3 + 3 * 5], 0);
    }

    #[test]
    fn malformed_document_fails_closed() {
        let path = std::env::temp_dir().join(format!(
            "nuwax-forbidden-zones-{}.json",
            uuid::Uuid::new_v4()
        ));
        std::fs::write(&path, b"{not-json").unwrap();
        let loaded = load_for_map(&path, "map-v1");
        assert!(matches!(loaded.status, ZoneLoadStatus::FailClosed(_)));
        let map = compose_effective_map(&free_map(3, 2), &loaded).unwrap();
        assert!(map.data.iter().all(|value| *value == 100));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn persisted_signature_is_the_runtime_ack_identity() {
        let path = std::env::temp_dir().join(format!(
            "nuwax-forbidden-zone-signature-{}-{}.json",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let expected = "0123456789abcdef".repeat(4);
        std::fs::write(
            &path,
            format!(
                r#"{{"schema_version":1,"maps":{{"map-v1":{{"map_version_id":"map-v1","generation":7,"signature":"{expected}","updated_at":"2026-07-20T00:00:00Z","zones":[]}}}}}}"#
            ),
        )
        .expect("write signed forbidden-zone document");

        let loaded = load_for_map(&path, "map-v1");

        assert_eq!(loaded.generation, 7);
        assert_eq!(loaded.signature, expected);
        assert_eq!(loaded.status, ZoneLoadStatus::Empty);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn map_binding_separates_equal_version_ids() {
        let path = std::env::temp_dir().join(format!(
            "nuwax-forbidden-zone-map-binding-{}-{}.json",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let warehouse_signature = "a".repeat(64);
        let office_signature = "b".repeat(64);
        std::fs::write(
            &path,
            format!(
                r#"{{"schema_version":1,"maps":{{"warehouse":{{"map_name":"warehouse","generation":1,"signature":"{warehouse_signature}","updated_at":"2026-07-20T00:00:00Z","zones":[]}},"office":{{"map_name":"office","generation":1,"signature":"{office_signature}","updated_at":"2026-07-20T00:00:00Z","zones":[]}}}}}}"#
            ),
        )
        .expect("write map-bound forbidden-zone document");

        let warehouse = load_for_map_binding(&path, "warehouse", "mapver_000009");
        let office = load_for_map_binding(&path, "office", "mapver_000009");

        assert_eq!(warehouse.signature, warehouse_signature);
        assert_eq!(office.signature, office_signature);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn invalid_persisted_signature_fails_closed() {
        let path = std::env::temp_dir().join(format!(
            "nuwax-forbidden-zone-bad-signature-{}-{}.json",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        std::fs::write(
            &path,
            br#"{"schema_version":1,"maps":{"map-v1":{"map_version_id":"map-v1","generation":7,"signature":"wrong-content","updated_at":"2026-07-20T00:00:00Z","zones":[]}}}"#,
        )
        .expect("write invalid signed forbidden-zone document");

        let loaded = load_for_map(&path, "map-v1");

        assert!(matches!(loaded.status, ZoneLoadStatus::FailClosed(_)));
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn self_intersecting_polygon_is_rejected() {
        let polygon = zone(
            ZoneShape::Polygon,
            ZoneMode::Inside,
            vec![
                ZonePoint { x: 0.0, y: 0.0 },
                ZonePoint { x: 2.0, y: 2.0 },
                ZonePoint { x: 0.0, y: 2.0 },
                ZonePoint { x: 2.0, y: 0.0 },
            ],
        );
        assert!(validate_zone(&polygon).is_err());
    }
}
