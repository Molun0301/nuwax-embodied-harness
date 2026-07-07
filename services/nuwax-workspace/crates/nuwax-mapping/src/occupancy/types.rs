//! 占据栅格基础类型
//! 提供：
//! - `CostValues`：位标志，cell 状态分类
//! - `OccupancyValue`：单 cell 值语义（Unknown / Free / Occupied）
//! - `OccupancyGrid`：栅格容器（包含 metadata）

use bitflags::bitflags;
use nuwax_types::Timestamp;
use serde::{Deserialize, Serialize};

/// 单元值语义
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
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

impl OccupancyValue {
    /// 从原始 i8 转换
    #[must_use]
    pub fn from_i8(v: i8) -> Self {
        match v {
            v if v >= 65 => Self::Occupied,
            v if v >= 1 => Self::Free,
            0 => Self::Free,
            _ => Self::Unknown,
        }
    }
}

bitflags! {
    /// 单元位标志
    ///
    /// 用于 ray tracing / 占据栅格的快速位运算分类。
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct CostValues: u8 {
        /// 未知
        const NO_INFORMATION = 0b0000_0001;
        /// 自由
        const FREE_SPACE     = 0b0000_0010;
        /// 占据
        const OCCUPIED       = 0b0000_0100;
        /// 内切（不可行走）
        const INFLATED       = 0b0000_1000;
        /// 障碍
        const OBSTACLE       = 0b0001_0000;
    }
}

/// 占据栅格（含 metadata）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyGrid {
    /// 单元值（按行优先）
    pub data: Vec<i8>,
    /// 宽度（列数）
    pub width: u32,
    /// 高度（行数）
    pub height: u32,
    /// 分辨率（m/cell）
    pub resolution: f64,
    /// 起源位姿（可选）
    pub origin: [f64; 3],
    /// 加载时间
    pub load_time: Timestamp,
}

impl OccupancyGrid {
    /// 构造新栅格
    #[must_use]
    pub fn new(width: u32, height: u32, resolution: f64) -> Self {
        Self {
            data: vec![OccupancyValue::Unknown as i8; (width * height) as usize],
            width,
            height,
            resolution,
            origin: [0.0, 0.0, 0.0],
            load_time: Timestamp::now(),
        }
    }

    /// 索引：(row, col) → data index
    #[must_use]
    pub fn index(&self, row: u32, col: u32) -> usize {
        (row * self.width + col) as usize
    }

    /// 读取某格
    /// Returns the cell value, or `None` if out of bounds.
    #[must_use]
    pub fn get(&self, row: u32, col: u32) -> Option<OccupancyValue> {
        if row >= self.height || col >= self.width {
            return None;
        }
        Some(OccupancyValue::from_i8(self.data[self.index(row, col)]))
    }

    /// 写入某格
    /// Returns `true` on success.
    pub fn set(&mut self, row: u32, col: u32, value: OccupancyValue) -> bool {
        if row >= self.height || col >= self.width {
            return false;
        }
        let idx = self.index(row, col);
        self.data[idx] = value as i8;
        true
    }

    /// 是否所有 cell 都为 Free（unknown 不计）
    #[must_use]
    pub fn is_all_free(&self) -> bool {
        self.data.iter().all(|&v| v <= 0)
    }

    /// 占据率（0.0 ~ 1.0）
    #[must_use]
    pub fn occupied_ratio(&self) -> f64 {
        if self.data.is_empty() {
            return 0.0;
        }
        let occ = self.data.iter().filter(|&&v| v >= 65).count();
        occ as f64 / self.data.len() as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn occupancy_value_from_i8() {
        assert_eq!(OccupancyValue::from_i8(-1), OccupancyValue::Unknown);
        assert_eq!(OccupancyValue::from_i8(0), OccupancyValue::Free);
        assert_eq!(OccupancyValue::from_i8(50), OccupancyValue::Free);
        assert_eq!(OccupancyValue::from_i8(65), OccupancyValue::Occupied);
        assert_eq!(OccupancyValue::from_i8(100), OccupancyValue::Occupied);
    }

    #[test]
    fn cost_values_bitflags() {
        let v = CostValues::OCCUPIED | CostValues::INFLATED;
        assert!(v.contains(CostValues::OCCUPIED));
        assert!(v.contains(CostValues::INFLATED));
        assert!(!v.contains(CostValues::FREE_SPACE));
    }

    #[test]
    fn grid_new_all_unknown() {
        let g = OccupancyGrid::new(10, 20, 0.05);
        assert_eq!(g.width, 10);
        assert_eq!(g.height, 20);
        assert!(g.is_all_free()); // 全部 -1 是 Unknown，但 -1 != 0 ，但我们 init 是 Unknown
        for r in 0..20 {
            for c in 0..10 {
                assert_eq!(g.get(r, c), Some(OccupancyValue::Unknown));
            }
        }
    }

    #[test]
    fn grid_set_get_roundtrip() {
        let mut g = OccupancyGrid::new(4, 4, 0.1);
        assert!(g.set(1, 2, OccupancyValue::Occupied));
        assert_eq!(g.get(1, 2), Some(OccupancyValue::Occupied));
        assert_eq!(g.get(0, 0), Some(OccupancyValue::Unknown));
    }

    #[test]
    fn grid_out_of_bounds() {
        let g = OccupancyGrid::new(4, 4, 0.1);
        assert_eq!(g.get(10, 0), None);
        assert_eq!(g.get(0, 10), None);
    }

    #[test]
    fn grid_occupied_ratio() {
        let mut g = OccupancyGrid::new(2, 2, 0.1);
        g.set(0, 0, OccupancyValue::Occupied);
        g.set(0, 1, OccupancyValue::Occupied);
        // 2/4 = 0.5
        assert!((g.occupied_ratio() - 0.5).abs() < 1e-9);
    }
}
