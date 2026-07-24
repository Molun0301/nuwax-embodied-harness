//! Voxelized terrain height aggregation and obstacle classification.

use std::collections::BTreeMap;

/// XYZ scan sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
    /// Z coordinate.
    pub z: f64,
}

/// Classified terrain cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainClass {
    /// Locally level surface.
    Ground,
    /// Height spread exceeds threshold.
    Obstacle,
}

/// One output voxel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainCell {
    /// Voxel centre X.
    pub x: f64,
    /// Voxel centre Y.
    pub y: f64,
    /// Selected quantile height.
    pub z: f64,
    /// Classification.
    pub class: TerrainClass,
    /// Samples in the voxel.
    pub samples: usize,
}

/// Terrain analysis parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainCoreConfig {
    /// XY voxel size.
    pub voxel_size: f64,
    /// Quantile in `[0,1]` used as representative height.
    pub quantile: f64,
    /// Maximum ground height spread.
    pub obstacle_height_threshold: f64,
    /// Sensor range around robot origin.
    pub sensor_range: f64,
}

impl Default for TerrainCoreConfig {
    fn default() -> Self {
        Self {
            voxel_size: 0.2,
            quantile: 0.25,
            obstacle_height_threshold: 0.15,
            sensor_range: 20.0,
        }
    }
}

/// Aggregate finite in-range points into deterministic classified terrain cells.
pub fn analyze(
    points: &[Point3],
    origin: Point3,
    config: TerrainCoreConfig,
) -> Option<Vec<TerrainCell>> {
    if ![
        origin.x,
        origin.y,
        origin.z,
        config.voxel_size,
        config.quantile,
        config.obstacle_height_threshold,
        config.sensor_range,
    ]
    .into_iter()
    .all(f64::is_finite)
        || config.voxel_size <= 0.0
        || !(0.0..=1.0).contains(&config.quantile)
        || config.obstacle_height_threshold < 0.0
        || config.sensor_range <= 0.0
    {
        return None;
    }
    let mut voxels: BTreeMap<(i64, i64), Vec<f64>> = BTreeMap::new();
    for point in points {
        if ![point.x, point.y, point.z].into_iter().all(f64::is_finite)
            || (point.x - origin.x).hypot(point.y - origin.y) > config.sensor_range
        {
            continue;
        }
        let key = (
            (point.x / config.voxel_size).floor() as i64,
            (point.y / config.voxel_size).floor() as i64,
        );
        voxels.entry(key).or_default().push(point.z);
    }
    Some(
        voxels
            .into_iter()
            .map(|((x, y), mut heights)| {
                heights.sort_by(f64::total_cmp);
                let index = ((heights.len() - 1) as f64 * config.quantile).round() as usize;
                let spread = heights.last().unwrap() - heights.first().unwrap();
                TerrainCell {
                    x: (x as f64 + 0.5) * config.voxel_size,
                    y: (y as f64 + 0.5) * config.voxel_size,
                    z: heights[index],
                    class: if spread > config.obstacle_height_threshold {
                        TerrainClass::Obstacle
                    } else {
                        TerrainClass::Ground
                    },
                    samples: heights.len(),
                }
            })
            .collect(),
    )
}
