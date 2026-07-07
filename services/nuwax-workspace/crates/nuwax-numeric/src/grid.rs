//! 占据栅格基础

use ndarray::Array2;
use serde::{Deserialize, Serialize};

/// 占据栅格值语义
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[repr(i8)]
pub enum OccupancyValue {
    /// 未知（原始 ROS 用 -1）
    #[default]
    Unknown = -1,
    /// 空闲
    Free = 0,
    /// 占据
    Occupied = 100,
}

/// 栅格接口（`nuwax-mapping::occupancy` 实现）
pub trait OccupancyGridLike {
    /// 宽度（列数）
    #[must_use]
    fn width(&self) -> usize;
    /// 高度（行数）
    #[must_use]
    fn height(&self) -> usize;
    /// 读取某格
    #[must_use]
    fn get(&self, x: usize, y: usize) -> OccupancyValue;
    /// 写入某格
    fn set(&mut self, x: usize, y: usize, value: OccupancyValue);
}

/// 占据栅格实现（基于 ndarray）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyGrid {
    data: Array2<i8>,
    resolution: f64, // meters / cell
}

impl OccupancyGrid {
    /// 构造新栅格
    #[must_use]
    pub fn new(width: usize, height: usize, resolution: f64) -> Self {
        Self {
            data: Array2::from_elem((height, width), OccupancyValue::Unknown as i8),
            resolution,
        }
    }

    /// 分辨率（m/cell）
    #[must_use]
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// 转换为 ndarray 视图（供 rayon 并行计算用）
    #[must_use]
    pub fn as_array(&self) -> &Array2<i8> {
        &self.data
    }
}

impl OccupancyGridLike for OccupancyGrid {
    fn width(&self) -> usize {
        self.data.ncols()
    }
    fn height(&self) -> usize {
        self.data.nrows()
    }
    fn get(&self, x: usize, y: usize) -> OccupancyValue {
        let v = self.data[(y, x)];
        match v {
            0 => OccupancyValue::Free,
            100 => OccupancyValue::Occupied,
            _ => OccupancyValue::Unknown,
        }
    }
    fn set(&mut self, x: usize, y: usize, value: OccupancyValue) {
        self.data[(y, x)] = value as i8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_grid_all_unknown() {
        let g = OccupancyGrid::new(10, 20, 0.05);
        assert_eq!(g.width(), 10);
        assert_eq!(g.height(), 20);
        for y in 0..20 {
            for x in 0..10 {
                assert_eq!(g.get(x, y), OccupancyValue::Unknown);
            }
        }
    }

    #[test]
    fn set_get_roundtrip() {
        let mut g = OccupancyGrid::new(4, 4, 0.1);
        g.set(1, 2, OccupancyValue::Occupied);
        assert_eq!(g.get(1, 2), OccupancyValue::Occupied);
        assert_eq!(g.get(0, 0), OccupancyValue::Unknown);
    }
}
