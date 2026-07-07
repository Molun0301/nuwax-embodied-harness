//! Costmap 与 MapMeta 数据类
//! 提供建图板块最核心的两个数据结构。

use nalgebra::Isometry3;
use nuwax_types::Timestamp;
use serde::{Deserialize, Serialize};

/// 地图元数据
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapMeta {
    /// 加载时间
    pub map_load_time: Timestamp,
    /// 分辨率（m/cell）
    pub resolution: f64,
    /// 宽度（cells）
    pub width: u32,
    /// 高度（cells）
    pub height: u32,
    /// 原点（map → world 变换）
    pub origin: Isometry3<f64>,
}

impl MapMeta {
    /// 构造
    #[must_use]
    pub fn new(resolution: f64, width: u32, height: u32, origin: Isometry3<f64>) -> Self {
        Self {
            map_load_time: Timestamp::now(),
            resolution,
            width,
            height,
            origin,
        }
    }
}

/// Costmap
///
/// 存储格式：`Vec<u8>`，值遵循 Nav2 约定：
/// - 0 = Free
/// - 100 = Occupied
/// - 252 / 253 / 254 = 距离障碍 N 格
/// - 255 = No information / unknown
///
/// ⚠️ 注意：原 Nav2 ROS message `nav_msgs/OccupancyGrid` 使用 `int8`（-1 = unknown），
/// 但 Rust 端存储用 u8 与大多数 ROS2 桥接 crate 一致；序列化时按 ROS 规范转换。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Costmap {
    /// 元数据
    pub metadata: MapMeta,
    /// 数据（按行优先：data[row * width + col]）
    pub data: Vec<u8>,
}

impl Costmap {
    /// 构造新 Costmap（默认所有 cell 为 Free=0）
    #[must_use]
    pub fn new(metadata: MapMeta) -> Self {
        let n = (metadata.width * metadata.height) as usize;
        Self {
            metadata,
            data: vec![0; n],
        }
    }

    /// 索引：(row, col) → data index
    #[must_use]
    pub fn index(&self, row: u32, col: u32) -> usize {
        (row * self.metadata.width + col) as usize
    }

    /// 读取某格值
    /// Returns the value at (row, col), or `None` if out of bounds.
    #[must_use]
    pub fn get(&self, row: u32, col: u32) -> Option<u8> {
        if row >= self.metadata.height || col >= self.metadata.width {
            return None;
        }
        Some(self.data[self.index(row, col)])
    }

    /// 写入某格值
    /// Returns `true` on success, `false` if out of bounds.
    pub fn set(&mut self, row: u32, col: u32, value: u8) -> bool {
        if row >= self.metadata.height || col >= self.metadata.width {
            return false;
        }
        let idx = self.index(row, col);
        self.data[idx] = value;
        true
    }

    /// 宽度
    #[must_use]
    pub fn width(&self) -> u32 {
        self.metadata.width
    }

    /// 高度
    #[must_use]
    pub fn height(&self) -> u32 {
        self.metadata.height
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn costmap_new_all_zero() {
        let meta = MapMeta::new(0.05, 10, 20, Isometry3::identity());
        let cm = Costmap::new(meta);
        assert_eq!(cm.width(), 10);
        assert_eq!(cm.height(), 20);
        for r in 0..20 {
            for c in 0..10 {
                assert_eq!(cm.get(r, c), Some(0));
            }
        }
    }

    #[test]
    fn costmap_set_get_roundtrip() {
        let meta = MapMeta::new(0.05, 4, 4, Isometry3::identity());
        let mut cm = Costmap::new(meta);
        assert!(cm.set(1, 2, 100));
        assert_eq!(cm.get(1, 2), Some(100));
        assert_eq!(cm.get(0, 0), Some(0));
    }

    #[test]
    fn costmap_out_of_bounds() {
        let meta = MapMeta::new(0.05, 4, 4, Isometry3::identity());
        let mut cm = Costmap::new(meta);
        assert_eq!(cm.get(10, 0), None);
        assert!(!cm.set(10, 0, 100));
    }

    #[test]
    fn map_meta_serialize() {
        let meta = MapMeta::new(0.1, 100, 100, Isometry3::identity());
        let json = serde_json::to_string(&meta).unwrap();
        let back: MapMeta = serde_json::from_str(&json).unwrap();
        assert_eq!(back.resolution, 0.1);
        assert_eq!(back.width, 100);
        assert_eq!(back.height, 100);
    }
}
