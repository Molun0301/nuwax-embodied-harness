//! 每张地图的禁行区文件与原保守栅格化算法。
use serde::{Deserialize, Serialize};
use std::{collections::HashSet, path::Path};
const MAX_ZONES: usize = 256;
const MAX_VERTICES: usize = 256;
const GEOMETRY_EPSILON: f64 = 1e-9;
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

#[derive(Deserialize)]
struct Document {
    #[serde(default)]
    zones: Vec<ForbiddenZone>,
}
#[derive(Clone)]
struct MapState {
    width: u32,
    height: u32,
    resolution: f32,
    origin_x: f64,
    origin_y: f64,
    data: Vec<i8>,
}
#[derive(Clone)]
struct LoadedForbiddenZones {
    zones: Vec<ForbiddenZone>,
}
fn default_frame_id() -> String {
    "map".into()
}
const fn default_enabled() -> bool {
    true
}
pub fn read(path: &Path) -> Result<Vec<ForbiddenZone>, String> {
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(format!("禁行区读取失败：{error}")),
    };
    let document: Document =
        serde_json::from_slice(&bytes).map_err(|e| format!("禁行区格式无效：{e}"))?;
    if document.zones.len() > MAX_ZONES {
        return Err("禁行区数量超过上限".into());
    }
    let mut ids = HashSet::new();
    for zone in &document.zones {
        validate_zone(zone)?;
        if !ids.insert(&zone.id) {
            return Err("禁行区编号重复".into());
        }
    }
    Ok(document.zones)
}
pub(crate) fn overlay_zones(
    zones: &[ForbiddenZone],
    grid: &nuwax_mapping::OccupancyGrid,
    data: &mut [u8],
) -> Result<(), String> {
    let raw = MapState {
        width: grid.width as u32,
        height: grid.height as u32,
        resolution: grid.resolution as f32,
        origin_x: grid.origin[0],
        origin_y: grid.origin[1],
        data: data.iter().map(|v| *v as i8).collect(),
    };
    validate_map_layout(&raw)?;
    let effective = rasterize_effective_map(
        raw,
        &LoadedForbiddenZones {
            zones: zones.to_vec(),
        },
    );
    for (out, value) in data.iter_mut().zip(effective.data) {
        *out = value as u8;
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

fn validate_map_layout(raw: &MapState) -> Result<(), String> {
    let expected = raw.width as usize * raw.height as usize;
    if raw.width == 0
        || raw.height == 0
        || !raw.resolution.is_finite()
        || raw.resolution <= 0.0
        || raw.data.len() != expected
    {
        return Err("raw occupancy layout is invalid".to_string());
    }
    Ok(())
}

fn rasterize_effective_map(
    mut effective: MapState,
    loaded: &LoadedForbiddenZones,
) -> MapState {
    let enabled: Vec<&ForbiddenZone> = loaded.zones.iter().filter(|zone| zone.enabled).collect();
    if enabled.is_empty() {
        return effective;
    }

    let cell_count = effective.data.len();
    let mut forbidden = vec![false; cell_count];
    let outside_zones: Vec<_> = enabled
        .iter()
        .copied()
        .filter(|zone| zone.mode == ZoneMode::Outside)
        .collect();
    if !outside_zones.is_empty() {
        for row in 0..effective.height {
            for col in 0..effective.width {
                let rect = cell_rect(&effective, col, row);
                let allowed = outside_zones.iter().all(|zone| {
                    cell_fully_inside_closed_shape(rect, zone, effective.resolution as f64)
                });
                if !allowed {
                    forbidden[cell_index(&effective, col, row)] = true;
                }
            }
        }
    }

    for zone in enabled {
        if zone.mode == ZoneMode::Outside {
            continue;
        }
        if zone.shape == ZoneShape::Rectangle {
            let shape = rectangle_from_corners(zone.vertices[0], zone.vertices[1]);
            let expanded = expand_rect(
                shape,
                boundary_half_width(zone, effective.resolution as f64),
            );
            let first_true = |len: u32, predicate: &dyn Fn(u32) -> bool| {
                let (mut lo, mut hi) = (0, len);
                while lo < hi {
                    let mid = lo + (hi - lo) / 2;
                    if predicate(mid) {
                        hi = mid;
                    } else {
                        lo = mid + 1;
                    }
                }
                lo
            };
            // Keep actual rounded bounds: (origin + i * r) + r can differ
            // from origin + (i + 1) * r at large origins.
            let col_start = first_true(effective.width, &|col| {
                cell_rect(&effective, col, 0).max_x >= expanded.min_x - GEOMETRY_EPSILON
            });
            // Negating <= also makes a NaN upper threshold yield no cells.
            let col_end = first_true(effective.width, &|col| {
                !(cell_rect(&effective, col, 0).min_x <= expanded.max_x + GEOMETRY_EPSILON)
            });
            let row_start = first_true(effective.height, &|row| {
                cell_rect(&effective, 0, row).max_y >= expanded.min_y - GEOMETRY_EPSILON
            });
            let row_end = first_true(effective.height, &|row| {
                !(cell_rect(&effective, 0, row).min_y <= expanded.max_y + GEOMETRY_EPSILON)
            });
            if col_start >= col_end || row_start >= row_end {
                continue;
            }
            for row in row_start..row_end {
                let offset = row as usize * effective.width as usize;
                forbidden[offset + col_start as usize..offset + col_end as usize].fill(true);
            }
            continue;
        }
        for row in 0..effective.height {
            for col in 0..effective.width {
                let rect = cell_rect(&effective, col, row);
                if cell_forbidden_by_inside_zone(rect, zone, effective.resolution as f64) {
                    forbidden[cell_index(&effective, col, row)] = true;
                }
            }
        }
    }

    for (index, is_forbidden) in forbidden.into_iter().enumerate() {
        if is_forbidden {
            effective.data[index] = 100;
        }
    }
    effective
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
fn compose_effective_map(
    raw: &MapState,
    loaded: &LoadedForbiddenZones,
) -> Result<MapState, String> {
    validate_map_layout(raw)?;
    Ok(rasterize_effective_map(raw.clone(), loaded))
}

#[cfg(test)]
#[path = "forbidden_zones_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "forbidden_zones_copy_trial_reference.rs"]
mod copy_trial_reference;
#[cfg(test)]
#[path = "forbidden_zones_rectangle_scan_tests.rs"]
mod rectangle_scan_tests;
