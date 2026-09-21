//! 按雷达当前高度提取水平扫描；仅对真实回波的前方射线声明自由空间。

use super::PointCloudOccupancyBuilder;
use crate::OccupancyGrid;
use nalgebra::Point3;

const HALF_HEIGHT_M: f64 = 0.1;
const MIN_RANGE_M: f64 = 0.3;
const MAX_RANGE_M: f64 = 30.0;
const ANGULAR_BINS: usize = 1440;

pub(super) fn contains_return(point: &Point3<f64>, origin: &Point3<f64>) -> bool {
    point.coords.iter().all(|v| v.is_finite())
        && (point.z - origin.z).abs() <= HALF_HEIGHT_M
        && (MIN_RANGE_M..=MAX_RANGE_M).contains(&(point.x - origin.x).hypot(point.y - origin.y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cost_mapper::SimplePointCloud2;
    use crate::pointclouds::occupancy::build_occupancy_auto;
    use nalgebra::point;

    fn build(points: Vec<Point3<f64>>, origin: Point3<f64>) -> OccupancyGrid {
        build_occupancy_auto(
            &SimplePointCloud2::with_metadata(points, origin, "map", 10.0),
            0.05,
        )
        .unwrap()
    }

    fn at(grid: &OccupancyGrid, x: f64, y: f64) -> i8 {
        let col = ((x - grid.origin[0]) / grid.resolution).floor() as u32;
        let row = ((y - grid.origin[1]) / grid.resolution).floor() as u32;
        grid.data[grid.index(row, col)]
    }

    #[test]
    fn planar_scan_uses_nearest_return_and_keeps_unseen_space_unknown() {
        let grid = build(
            vec![
                point![2.025, 0.025, 0.0],
                point![4.025, 0.025, 0.0],
                point![1.025, 0.025, -0.4],
                point![1.025, 0.025, 2.0],
            ],
            point![0.025, 0.025, 0.0],
        );
        assert_eq!(at(&grid, 1.025, 0.025), 0);
        assert_eq!(at(&grid, 2.025, 0.025), 100);
        assert_eq!(at(&grid, 3.025, 0.025), -1);
        assert_eq!(at(&grid, 1.025, 0.525), -1);
    }

    #[test]
    fn planar_scan_preserves_thin_obstacles_and_is_height_translation_invariant() {
        let points = vec![
            point![1.025, 0.025, 0.01],
            point![2.025, 0.025, 0.05],
            point![2.025, 0.525, -0.02],
            point![0.5, 0.5, -0.4],
        ];
        let original = build(points.clone(), point![0.025, 0.025, 0.0]);
        let shifted = build(
            points
                .into_iter()
                .map(|p| point![p.x, p.y, p.z + 12.0])
                .collect(),
            point![0.025, 0.025, 12.0],
        );
        assert_eq!(at(&original, 1.025, 0.025), 100);
        assert_eq!(original.data, shifted.data);
        assert_eq!(original.origin, shifted.origin);
    }

    #[test]
    fn planar_scan_keeps_wall_cells_without_height_gradient_halos() {
        let mut points = Vec::new();
        for row in -40..=40 {
            let y = row as f64 * 0.05 + 0.025;
            points.push(point![2.025, y, 0.0]);
            points.push(point![2.025, y, -0.4]);
            points.push(point![2.025, y, 2.0]);
        }
        let grid = build(points, point![0.025, 0.025, 0.0]);
        for row in -40..=40 {
            let y = row as f64 * 0.05 + 0.025;
            assert_eq!(at(&grid, 2.025, y), 100);
            assert_eq!(at(&grid, 2.075, y), -1);
        }
        assert_eq!(grid.data.iter().filter(|&&v| v == 100).count(), 81);
        assert_eq!(at(&grid, 1.975, 0.025), 0);
    }

    #[test]
    fn planar_scan_gridline_origin_is_stable_after_coordinate_roundoff() {
        let points = vec![point![2.025, 0.725, 0.0], point![2.025, -0.725, 0.0]];
        let exact = build(points.clone(), point![0.2, -0.1, 0.0]);
        for epsilon in [-1e-14, 1e-14] {
            let perturbed = build(points.clone(), point![0.2 + epsilon, -0.1 + epsilon, 0.0]);
            assert_eq!(exact.origin, perturbed.origin);
            assert_eq!(exact.data, perturbed.data);
        }
    }

    #[test]
    fn planar_scan_bounds_ignore_rejected_points_and_include_sensor() {
        let grid = build(
            vec![
                point![10.025, 0.025, 0.0],
                point![1e8, 1e8, 8.0],
                point![40.0, 0.0, 0.0],
                point![f64::NAN, 0.0, 0.0],
            ],
            point![0.025, 0.025, 0.0],
        );
        assert!(grid.width < 250);
        assert!(grid.height < 50);
        assert_eq!(at(&grid, 0.025, 0.025), 0);
        assert_eq!(at(&grid, 10.025, 0.025), 100);
        assert_eq!(grid.frame_id, "map");
        assert_eq!(grid.ts, 10.0);
        let missing = build(vec![point![1.0, 0.0, -0.4]], Point3::origin());
        assert!(missing.data.iter().all(|&v| v < 0));
    }
}

impl PointCloudOccupancyBuilder {
    pub(super) fn build_planar_scan(&self, points: &[Point3<f64>]) -> OccupancyGrid {
        let mut grid = self.empty_grid();
        if !self.sensor_origin.coords.iter().all(|v| v.is_finite()) {
            return grid;
        }
        let cell = |point: &Point3<f64>| -> Option<(i32, i32)> {
            // 坐标往返变换的舍入误差不能把格线上的原点移到相邻格。
            let index = |value: f64| super::snap_gridline(value).floor();
            let x = index((point.x - self.origin_xy.0) / self.resolution);
            let y = index((point.y - self.origin_xy.1) / self.resolution);
            (x >= 0.0 && y >= 0.0 && x < self.width as f64 && y < self.height as f64)
                .then_some((x as i32, y as i32))
        };
        let Some((start_x, start_y)) = cell(&self.sensor_origin) else {
            return grid;
        };
        let mut nearest = [f64::INFINITY; ANGULAR_BINS];
        let mut endpoints = [None; ANGULAR_BINS];
        for point in points {
            if !contains_return(point, &self.sensor_origin) {
                continue;
            }
            let dx = point.x - self.sensor_origin.x;
            let dy = point.y - self.sensor_origin.y;
            let range = dx.hypot(dy);
            let angle = dy.atan2(dx).rem_euclid(std::f64::consts::TAU);
            let bin = ((angle / std::f64::consts::TAU * ANGULAR_BINS as f64) as usize)
                .min(ANGULAR_BINS - 1);
            if range < nearest[bin] {
                if let Some(endpoint) = cell(point) {
                    nearest[bin] = range;
                    endpoints[bin] = Some(endpoint);
                }
            }
        }
        // 先标记全部最近端点，射线到达任一障碍就停止，避免清穿其它方向的薄墙。
        for &(x, y) in endpoints.iter().flatten() {
            grid.data[y as usize * self.width as usize + x as usize] = 100;
        }
        for &(end_x, end_y) in endpoints.iter().flatten() {
            let (mut x, mut y) = (start_x, start_y);
            let dx = (end_x - x).abs();
            let dy = -(end_y - y).abs();
            let sx = if x < end_x { 1 } else { -1 };
            let sy = if y < end_y { 1 } else { -1 };
            let mut error = dx + dy;
            loop {
                let index = y as usize * self.width as usize + x as usize;
                if grid.data[index] == 100 || (x == end_x && y == end_y) {
                    break;
                }
                grid.data[index] = 0;
                let twice = 2 * error;
                if twice >= dy {
                    error += dy;
                    x += sx;
                }
                if twice <= dx {
                    error += dx;
                    y += sy;
                }
            }
        }
        grid
    }
}
