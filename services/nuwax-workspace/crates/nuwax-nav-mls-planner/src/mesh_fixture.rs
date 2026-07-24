// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Deterministic OBJ surface sampling and voxelization for evaluator fixtures.

use std::collections::BTreeSet;
use std::path::Path;

const MAX_OBJ_BYTES: u64 = 16 * 1024 * 1024;
const MAX_MESH_ITEMS: usize = 1_000_000;
const MAX_MESH_SAMPLES: usize = 1_000_000;

/// Mesh-fixture loading options.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshFixtureConfig {
    /// Output voxel edge length in meters.
    pub voxel_size: f32,
    /// Maximum spacing between deterministic samples on each triangle.
    pub sample_spacing: f32,
    /// Convert Y-up vertices `(x, y, z)` to Z-up `(x, -z, y)`.
    pub swap_y_up_to_z_up: bool,
    /// Center the XY bounds and move the minimum Z to zero.
    pub recenter: bool,
}

impl Default for MeshFixtureConfig {
    fn default() -> Self {
        Self {
            voxel_size: 0.1,
            sample_spacing: 0.1,
            swap_y_up_to_z_up: true,
            recenter: true,
        }
    }
}

/// Mesh fixture parse or validation error.
#[derive(Debug, thiserror::Error)]
pub enum MeshFixtureError {
    /// File could not be read.
    #[error("failed to read mesh fixture: {0}")]
    Io(#[from] std::io::Error),
    /// OBJ syntax or index was invalid.
    #[error("invalid OBJ fixture at line {line}: {reason}")]
    Obj {
        /// One-based source line.
        line: usize,
        /// Parse failure.
        reason: String,
    },
    /// Configuration or geometry was unusable.
    #[error("invalid mesh fixture: {0}")]
    Invalid(String),
}

/// Load, sample, transform and voxelize an ASCII OBJ fixture.
pub fn load_voxelized_obj(
    path: impl AsRef<Path>,
    config: MeshFixtureConfig,
) -> Result<Vec<(f32, f32, f32)>, MeshFixtureError> {
    let path = path.as_ref();
    if std::fs::metadata(path)?.len() > MAX_OBJ_BYTES {
        return Err(MeshFixtureError::Invalid(format!(
            "OBJ exceeds the {MAX_OBJ_BYTES}-byte limit"
        )));
    }
    let source = std::fs::read_to_string(path)?;
    voxelize_obj_str(&source, config)
}

/// Parse and voxelize an in-memory ASCII OBJ fixture.
pub fn voxelize_obj_str(
    source: &str,
    config: MeshFixtureConfig,
) -> Result<Vec<(f32, f32, f32)>, MeshFixtureError> {
    if source.len() as u64 > MAX_OBJ_BYTES {
        return Err(MeshFixtureError::Invalid(format!(
            "OBJ exceeds the {MAX_OBJ_BYTES}-byte limit"
        )));
    }
    if !(config.voxel_size.is_finite()
        && config.voxel_size > 0.0
        && (1.0 / config.voxel_size).is_finite())
    {
        return Err(MeshFixtureError::Invalid(
            "voxel_size must be finite and positive".into(),
        ));
    }
    if !(config.sample_spacing.is_finite() && config.sample_spacing > 0.0) {
        return Err(MeshFixtureError::Invalid(
            "sample_spacing must be finite and positive".into(),
        ));
    }

    let (mut vertices, triangles) = parse_obj(source)?;
    if vertices.is_empty() {
        return Err(MeshFixtureError::Invalid("OBJ has no vertices".into()));
    }
    if config.swap_y_up_to_z_up {
        for vertex in &mut vertices {
            *vertex = (vertex.0, -vertex.2, vertex.1);
        }
    }

    let mut samples = if triangles.is_empty() {
        vertices.clone()
    } else {
        sample_triangles(&vertices, &triangles, config.sample_spacing)?
    };
    if config.recenter {
        recenter_points(&mut samples);
    }
    if samples
        .iter()
        .any(|point| ![point.0, point.1, point.2].into_iter().all(f32::is_finite))
    {
        return Err(MeshFixtureError::Invalid(
            "mesh transformation produced non-finite coordinates".into(),
        ));
    }
    Ok(quantize_unique(&samples, config.voxel_size))
}

type Triangle = (usize, usize, usize);
type ParsedObj = (Vec<(f32, f32, f32)>, Vec<Triangle>);

fn parse_obj(source: &str) -> Result<ParsedObj, MeshFixtureError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for (line_idx, raw) in source.lines().enumerate() {
        let line_number = line_idx + 1;
        let line = raw.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_whitespace();
        match parts.next() {
            Some("v") => {
                let coordinates: Vec<_> = parts.take(3).collect();
                if coordinates.len() != 3 {
                    return Err(obj_error(line_number, "vertex requires three coordinates"));
                }
                let x = parse_f32(coordinates[0], line_number)?;
                let y = parse_f32(coordinates[1], line_number)?;
                let z = parse_f32(coordinates[2], line_number)?;
                vertices.push((x, y, z));
                if vertices.len() > MAX_MESH_ITEMS {
                    return Err(obj_error(line_number, "vertex limit exceeded"));
                }
            }
            Some("f") => {
                let indices: Result<Vec<_>, _> = parts
                    .map(|token| parse_face_index(token, vertices.len(), line_number))
                    .collect();
                let indices = indices?;
                if indices.len() < 3 {
                    return Err(obj_error(
                        line_number,
                        "face requires at least three vertices",
                    ));
                }
                for pair in indices[1..].windows(2) {
                    triangles.push((indices[0], pair[0], pair[1]));
                    if triangles.len() > MAX_MESH_ITEMS {
                        return Err(obj_error(line_number, "triangle limit exceeded"));
                    }
                }
            }
            _ => {}
        }
    }
    Ok((vertices, triangles))
}

fn parse_f32(value: &str, line: usize) -> Result<f32, MeshFixtureError> {
    let value = value
        .parse::<f32>()
        .map_err(|_| obj_error(line, format!("invalid float {value:?}")))?;
    value
        .is_finite()
        .then_some(value)
        .ok_or_else(|| obj_error(line, "non-finite vertex coordinate"))
}

fn parse_face_index(
    token: &str,
    vertex_count: usize,
    line: usize,
) -> Result<usize, MeshFixtureError> {
    let raw = token.split('/').next().unwrap_or_default();
    let index = raw
        .parse::<isize>()
        .map_err(|_| obj_error(line, format!("invalid face index {token:?}")))?;
    let resolved = if index > 0 {
        index - 1
    } else if index < 0 {
        vertex_count as isize + index
    } else {
        return Err(obj_error(line, "OBJ indices are one-based"));
    };
    if resolved < 0 || resolved as usize >= vertex_count {
        return Err(obj_error(
            line,
            format!("face index {index} is out of range"),
        ));
    }
    Ok(resolved as usize)
}

fn obj_error(line: usize, reason: impl Into<String>) -> MeshFixtureError {
    MeshFixtureError::Obj {
        line,
        reason: reason.into(),
    }
}

fn sample_triangles(
    vertices: &[(f32, f32, f32)],
    triangles: &[Triangle],
    spacing: f32,
) -> Result<Vec<(f32, f32, f32)>, MeshFixtureError> {
    let mut samples = Vec::new();
    for &(a_idx, b_idx, c_idx) in triangles {
        let a = vertices[a_idx];
        let b = vertices[b_idx];
        let c = vertices[c_idx];
        let longest = distance(a, b).max(distance(b, c)).max(distance(c, a));
        if !longest.is_finite() {
            return Err(MeshFixtureError::Invalid(
                "triangle dimensions must be finite".into(),
            ));
        }
        let divisions = (f64::from(longest) / f64::from(spacing)).ceil().max(1.0);
        if divisions > MAX_MESH_SAMPLES as f64 {
            return Err(MeshFixtureError::Invalid(format!(
                "mesh sampling exceeds the {MAX_MESH_SAMPLES}-point limit"
            )));
        }
        let divisions = divisions as usize;
        let sample_count = (divisions + 1)
            .checked_mul(divisions + 2)
            .map(|count| count / 2)
            .ok_or_else(|| MeshFixtureError::Invalid("triangle sample count overflowed".into()))?;
        if samples.len().saturating_add(sample_count) > MAX_MESH_SAMPLES {
            return Err(MeshFixtureError::Invalid(format!(
                "mesh sampling exceeds the {MAX_MESH_SAMPLES}-point limit"
            )));
        }
        for i in 0..=divisions {
            for j in 0..=(divisions - i) {
                let wb = i as f32 / divisions as f32;
                let wc = j as f32 / divisions as f32;
                let wa = 1.0 - wb - wc;
                samples.push((
                    wa * a.0 + wb * b.0 + wc * c.0,
                    wa * a.1 + wb * b.1 + wc * c.1,
                    wa * a.2 + wb * b.2 + wc * c.2,
                ));
            }
        }
    }
    Ok(samples)
}

fn distance(a: (f32, f32, f32), b: (f32, f32, f32)) -> f32 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let dz = a.2 - b.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn recenter_points(points: &mut [(f32, f32, f32)]) {
    let mut min = (f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let mut max = (f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    for &(x, y, z) in points.iter() {
        min.0 = min.0.min(x);
        min.1 = min.1.min(y);
        min.2 = min.2.min(z);
        max.0 = max.0.max(x);
        max.1 = max.1.max(y);
        max.2 = max.2.max(z);
    }
    let center_x = (min.0 + max.0) * 0.5;
    let center_y = (min.1 + max.1) * 0.5;
    for point in points {
        point.0 -= center_x;
        point.1 -= center_y;
        point.2 -= min.2;
    }
}

fn quantize_unique(points: &[(f32, f32, f32)], voxel_size: f32) -> Vec<(f32, f32, f32)> {
    let inv = 1.0 / voxel_size;
    let cells: BTreeSet<_> = points
        .iter()
        .map(|&(x, y, z)| {
            (
                (x * inv).floor() as i32,
                (y * inv).floor() as i32,
                (z * inv).floor() as i32,
            )
        })
        .collect();
    cells
        .into_iter()
        .map(|(x, y, z)| {
            (
                (x as f32 + 0.5) * voxel_size,
                (y as f32 + 0.5) * voxel_size,
                (z as f32 + 0.5) * voxel_size,
            )
        })
        .collect()
}
