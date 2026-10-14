//! 单帧点云的水平扫描投影；持久地图由上层拥有方逐帧累积。

use crate::cost_mapper::PointCloud2Like;
use crate::models::{Costmap, MapMeta};
use crate::OccupancyGrid;
use nalgebra::{Isometry3, Point3};
use thiserror::Error;

#[path = "planar_scan.rs"]
mod planar_scan;

/// 投影输入或栅格范围无效。
#[derive(Debug, Error)]
pub enum MappingError {
    /// 无法用有效栅格表达输入。
    #[error("invalid config: {0}")]
    InvalidConfig(&'static str),
}

/// 与导航代价地图共用的容器。
pub type CostMap2D = Costmap;

/// 创建全部未知的固定尺寸代价图。
#[must_use]
pub fn make_costmap2d(width: u32, height: u32, resolution: f64) -> CostMap2D {
    Costmap::new(MapMeta::new(
        resolution,
        width,
        height,
        Isometry3::identity(),
    ))
}

/// 将单帧水平观测写入现有栅格，未知保持为255。
pub fn project_to_occupancy<P: PointCloud2Like + ?Sized>(
    cloud: &P,
    grid: &mut CostMap2D,
) -> Result<(), MappingError> {
    validate_resolution(grid.metadata.resolution)?;
    if grid.data.len() != grid.metadata.width as usize * grid.metadata.height as usize {
        return Err(MappingError::InvalidConfig(
            "grid dimensions do not match storage",
        ));
    }
    let builder = PointCloudOccupancyBuilder::new(
        cloud.sensor_origin(),
        grid.metadata.resolution,
        grid.metadata.width,
        grid.metadata.height,
        (
            grid.metadata.origin.translation.x,
            grid.metadata.origin.translation.y,
        ),
    );
    let projected = builder.build(cloud.points());
    for (target, source) in grid.data.iter_mut().zip(projected.data) {
        *target = if source < 0 { 255 } else { source as u8 };
    }
    Ok(())
}

/// 按高度带内的有效观测和同帧雷达原点确定范围，四周留一米边界。
/// 没有有效回波时返回一格未知，不将缺测方向推定为自由。
pub fn build_occupancy_auto<P: PointCloud2Like + ?Sized>(
    cloud: &P,
    resolution: f64,
) -> Result<OccupancyGrid, MappingError> {
    validate_resolution(resolution)?;
    let origin = cloud.sensor_origin();
    if !origin.coords.iter().all(|value| value.is_finite()) {
        return Err(MappingError::InvalidConfig("sensor origin must be finite"));
    }
    let mut min_x = origin.x;
    let mut max_x = origin.x;
    let mut min_y = origin.y;
    let mut max_y = origin.y;
    let mut observed = false;
    for point in cloud.points() {
        let Some((point, _)) = planar_scan::observed_ray_end(point, &origin) else {
            continue;
        };
        observed = true;
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_y = min_y.min(point.y);
        max_y = max_y.max(point.y);
    }
    if !observed {
        let mut grid = OccupancyGrid::with_frame(1, 1, resolution, cloud.frame_id());
        grid.ts = cloud.timestamp();
        return Ok(grid);
    }
    let (origin_x, width) = auto_grid_axis_bounds(min_x - 1.0, max_x + 1.0, resolution)?;
    let (origin_y, height) = auto_grid_axis_bounds(min_y - 1.0, max_y + 1.0, resolution)?;
    if u64::from(width) * u64::from(height) > u64::from(u32::MAX) {
        return Err(MappingError::InvalidConfig(
            "auto-bounds grid exceeds occupancy storage capacity",
        ));
    }
    let mut grid =
        PointCloudOccupancyBuilder::new(origin, resolution, width, height, (origin_x, origin_y))
            .build(cloud.points());
    grid.frame_id = cloud.frame_id().to_string();
    grid.ts = cloud.timestamp();
    Ok(grid)
}

fn validate_resolution(resolution: f64) -> Result<(), MappingError> {
    if !resolution.is_finite() || resolution <= 0.0 {
        return Err(MappingError::InvalidConfig(
            "resolution must be finite and positive",
        ));
    }
    Ok(())
}

// 仅消除坐标往返变换的舍入误差，不平滑或移动真实点位。
fn snap_gridline(value: f64) -> f64 {
    let nearest = value.round();
    if (value - nearest).abs() <= 1e-9 {
        nearest
    } else {
        value
    }
}

fn auto_grid_axis_bounds(
    minimum: f64,
    maximum: f64,
    resolution: f64,
) -> Result<(f64, u32), MappingError> {
    let min_cell = snap_gridline(minimum / resolution).floor();
    let max_cell = snap_gridline(maximum / resolution).ceil();
    let cells = max_cell - min_cell;
    if !min_cell.is_finite()
        || !max_cell.is_finite()
        || !cells.is_finite()
        || cells < 1.0
        || cells > f64::from(u32::MAX)
    {
        return Err(MappingError::InvalidConfig(
            "auto-bounds dimension is not representable",
        ));
    }
    Ok((min_cell * resolution, cells as u32))
}

/// 固定边界的单帧水平扫描投影器。
pub struct PointCloudOccupancyBuilder {
    pub sensor_origin: Point3<f64>,
    pub resolution: f64,
    pub width: u32,
    pub height: u32,
    pub origin_xy: (f64, f64),
}

impl PointCloudOccupancyBuilder {
    #[must_use]
    pub fn new(
        sensor_origin: Point3<f64>,
        resolution: f64,
        width: u32,
        height: u32,
        origin_xy: (f64, f64),
    ) -> Self {
        Self {
            sensor_origin,
            resolution,
            width,
            height,
            origin_xy,
        }
    }

    #[must_use]
    pub fn build(&self, points: &[Point3<f64>]) -> OccupancyGrid {
        self.build_planar_scan(points)
    }

    fn empty_grid(&self) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(self.width, self.height, self.resolution);
        grid.origin = [self.origin_xy.0, self.origin_xy.1, 0.0];
        grid
    }

    #[must_use]
    pub fn ray_direction(&self, point: Point3<f64>) -> nalgebra::Vector3<f64> {
        point - self.sensor_origin
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cost_mapper::SimplePointCloud2;

    #[test]
    fn invalid_points_do_not_change_observations_or_metadata() {
        let mut cloud = SimplePointCloud2::with_metadata(
            vec![
                Point3::new(2.025, 0.025, 0.0),
                Point3::new(2.025, 0.525, 0.0),
            ],
            Point3::origin(),
            "map",
            7.0,
        );
        let clean = build_occupancy_auto(&cloud, 0.05).unwrap();
        cloud.points.extend([
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(0.0, f64::INFINITY, 0.0),
        ]);
        let actual = build_occupancy_auto(&cloud, 0.05).unwrap();
        assert_eq!(clean.data, actual.data);
        assert_eq!(clean.origin, actual.origin);
        assert_eq!((clean.width, clean.height), (actual.width, actual.height));
        assert_eq!(actual.frame_id, "map");
        assert_eq!(actual.ts, 7.0);
    }

    #[test]
    fn invalid_resolution_and_origin_are_rejected() {
        let mut cloud = SimplePointCloud2::new(vec![Point3::new(2.0, 0.0, 0.0)], Point3::origin());
        for resolution in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            assert!(build_occupancy_auto(&cloud, resolution).is_err());
        }
        cloud.sensor_origin.x = f64::NAN;
        assert!(build_occupancy_auto(&cloud, 0.05).is_err());
    }

    #[test]
    fn fixed_grid_preserves_free_occupied_and_unknown_values() {
        let cloud = SimplePointCloud2::new(
            vec![Point3::new(0.825, 0.025, 0.0)],
            Point3::new(0.025, 0.025, 0.0),
        );
        let mut grid = make_costmap2d(20, 20, 0.05);
        project_to_occupancy(&cloud, &mut grid).unwrap();
        assert_eq!(grid.get(0, 8), Some(0));
        assert_eq!(grid.get(0, 16), Some(100));
        assert_eq!(grid.get(0, 17), Some(255));
        assert_eq!(grid.get(1, 8), Some(255));
    }
}
