//! 点云 → 占据栅格
//! 算法：
//! 1. ray cast 从传感器到每点
//! 2. 把点云落到栅格
//! 3. 应用 Bresenham 标记路径为 free
//! 4. 点本身标记为 occupied

use crate::occupancy::types::{OccupancyGrid, OccupancyValue};
use nalgebra::{Point3, Vector3};
use rayon::prelude::*;

/// 点云占据栅格构建器
pub struct PointCloudOccupancyBuilder {
    /// 传感器 origin
    pub sensor_origin: Point3<f64>,
    /// 分辨率
    pub resolution: f64,
    /// 栅格宽
    pub width: u32,
    /// 栅格高
    pub height: u32,
    /// origin XY
    pub origin_xy: (f64, f64),
}

impl PointCloudOccupancyBuilder {
    /// 构造
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

    /// 并行构建占据栅格
    ///
    /// `points` 是已变换到世界坐标系下的点云。
    #[must_use]
    pub fn build(&self, points: &[Point3<f64>]) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(self.width, self.height, self.resolution);

        // 阶段 1：标记每个点为 Occupied
        // rayon 并行：每个 chunk 更新自己的局部栅格，最后归约
        let local_grids: Vec<OccupancyGrid> = points
            .par_chunks(1024)
            .map(|chunk| {
                let mut lg = OccupancyGrid::new(self.width, self.height, self.resolution);
                for &p in chunk {
                    if let Some((r, c)) = self.world_to_cell(p) {
                        lg.set(r, c, OccupancyValue::Occupied);
                    }
                }
                lg
            })
            .collect();

        for lg in &local_grids {
            for r in 0..self.height {
                for c in 0..self.width {
                    if matches!(lg.get(r, c), Some(OccupancyValue::Occupied)) {
                        grid.set(r, c, OccupancyValue::Occupied);
                    }
                }
            }
        }

        // 阶段 2：每个点 ray cast 路径标记为 Free（Bresenham 3D）
        // 简化版：从 sensor_origin 到每个点做 3D Bresenham，全部经过 cell 设为 Free
        for &p in points {
            if let Some(path) = self.bresenham_3d(self.sensor_origin, p) {
                for (r, c) in path {
                    let cur = grid.get(r, c).unwrap_or(OccupancyValue::Unknown);
                    if matches!(cur, OccupancyValue::Unknown) {
                        grid.set(r, c, OccupancyValue::Free);
                    }
                }
            }
        }

        grid
    }

    /// World → cell (row, col)
    fn world_to_cell(&self, p: Point3<f64>) -> Option<(u32, u32)> {
        let c = ((p.x - self.origin_xy.0) / self.resolution).floor() as i32;
        let r = ((p.y - self.origin_xy.1) / self.resolution).floor() as i32;
        if r < 0 || c < 0 || r >= self.height as i32 || c >= self.width as i32 {
            return None;
        }
        Some((r as u32, c as u32))
    }

    /// 3D Bresenham：从 `from` 到 `to` 沿直线经过的 2D cell 路径（忽略 Z）
    fn bresenham_3d(&self, from: Point3<f64>, to: Point3<f64>) -> Option<Vec<(u32, u32)>> {
        let p0 = self.world_to_cell(from)?;
        let p1 = self.world_to_cell(to)?;
        let mut cells = Vec::new();
        // 简化：用 2D Bresenham（XY 平面）
        let mut x0 = p0.1 as i32;
        let mut y0 = p0.0 as i32;
        let x1 = p1.1 as i32;
        let y1 = p1.0 as i32;
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            if x0 < 0 || y0 < 0 || x0 >= self.width as i32 || y0 >= self.height as i32 {
                return None;
            }
            cells.push((y0 as u32, x0 as u32));
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
        }
        Some(cells)
    }

    /// ray cast 方向辅助
    #[must_use]
    pub fn ray_direction(&self, p: Point3<f64>) -> Vector3<f64> {
        p - self.sensor_origin
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world_to_cell_basic() {
        let b = PointCloudOccupancyBuilder::new(
            nalgebra::point![0.0, 0.0, 0.0],
            0.1,
            100,
            100,
            (-5.0, -5.0),
        );
        // world (0.15, 0.25)：origin=(-5,-5)，world_x - origin_x = 5.15 → col = floor(5.15/0.1) = 51
        let cell = b.world_to_cell(nalgebra::point![0.15, 0.25, 0.0]).unwrap();
        assert_eq!(cell, (52, 51));
    }

    #[test]
    fn build_marks_occupied() {
        let b = PointCloudOccupancyBuilder::new(
            nalgebra::point![0.0, 0.0, 0.0],
            0.1,
            10,
            10,
            (-0.5, -0.5),
        );
        let pts = vec![nalgebra::point![0.0, 0.0, 0.0]];
        let g = b.build(&pts);
        // (0.0, 0.0) → (col=5, row=5)
        assert_eq!(g.get(5, 5), Some(OccupancyValue::Occupied));
    }

    #[test]
    fn build_parallel_runs() {
        let b = PointCloudOccupancyBuilder::new(
            nalgebra::point![0.0, 0.0, 0.0],
            0.1,
            10,
            10,
            (0.0, 0.0),
        );
        let pts: Vec<Point3<f64>> = (0..1000)
            .map(|i| nalgebra::point![(i as f64) * 0.01, 0.0, 0.0])
            .collect();
        let g = b.build(&pts);
        // 多点 → block_count > 0
        assert!(g.occupied_ratio() > 0.0);
    }
}
