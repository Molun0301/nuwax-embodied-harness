//! 点云累积器

use nalgebra::Point3;

/// 累积器抽象 trait
pub trait PointCloudAccumulator {
    /// 喂入一帧点云
    fn feed(&mut self, points: &[Point3<f64>]);
    /// 当前累积点数
    fn count(&self) -> usize;
}

/// 简单 XYZ 累积器（无限追加）
pub struct XyzAccumulator {
    points: Vec<Point3<f64>>,
}

impl XyzAccumulator {
    /// 构造空累积器
    #[must_use]
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// 取出累积的所有点
    #[must_use]
    pub fn into_points(self) -> Vec<Point3<f64>> {
        self.points
    }
}

impl Default for XyzAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl PointCloudAccumulator for XyzAccumulator {
    fn feed(&mut self, points: &[Point3<f64>]) {
        self.points.extend_from_slice(points);
    }
    fn count(&self) -> usize {
        self.points.len()
    }
}

/// 体素降采样累积器（保留最近点）
pub struct VoxelDownsampleAccumulator {
    /// 体素边长
    pub voxel_size: f64,
    voxels: std::collections::HashMap<(i32, i32, i32), Point3<f64>>,
}

impl VoxelDownsampleAccumulator {
    /// 构造
    #[must_use]
    pub fn new(voxel_size: f64) -> Self {
        Self {
            voxel_size,
            voxels: std::collections::HashMap::new(),
        }
    }

    /// 取出降采样后的点集
    #[must_use]
    pub fn into_points(self) -> Vec<Point3<f64>> {
        self.voxels.into_values().collect()
    }
}

impl PointCloudAccumulator for VoxelDownsampleAccumulator {
    fn feed(&mut self, points: &[Point3<f64>]) {
        for p in points {
            let key = (
                (p.x / self.voxel_size).floor() as i32,
                (p.y / self.voxel_size).floor() as i32,
                (p.z / self.voxel_size).floor() as i32,
            );
            self.voxels.insert(key, *p); // 后点覆盖
        }
    }
    fn count(&self) -> usize {
        self.voxels.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xyz_accumulator_basic() {
        let mut acc = XyzAccumulator::new();
        acc.feed(&[nalgebra::point![0.0, 0.0, 0.0]]);
        acc.feed(&[nalgebra::point![1.0, 1.0, 1.0]]);
        assert_eq!(acc.count(), 2);
        assert_eq!(acc.into_points().len(), 2);
    }

    #[test]
    fn voxel_downsample_merges_close_points() {
        let mut acc = VoxelDownsampleAccumulator::new(0.1);
        acc.feed(&[
            nalgebra::point![0.0, 0.0, 0.0],
            nalgebra::point![0.05, 0.05, 0.05], // 同一 voxel
            nalgebra::point![1.0, 1.0, 1.0],
        ]);
        // 3 个输入 → 2 个 voxel
        assert_eq!(acc.count(), 2);
    }
}
