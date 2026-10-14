//! 占据栅格基础操作
//! 提供：
//! - 翻转（横向/纵向）
//! - 缩放
//! - 子区域裁剪
//! - 合并两个栅格（取 max / min / 加权）

use crate::occupancy::types::{OccupancyGrid, OccupancyValue};

/// 栅格操作 trait
pub struct OccupancyOps;

impl OccupancyOps {
    /// 横向翻转（左右镜像）
    #[must_use]
    pub fn flip_horizontal(grid: &OccupancyGrid) -> OccupancyGrid {
        let mut out = grid.clone();
        for r in 0..grid.height {
            for c in 0..grid.width {
                let new_c = grid.width - 1 - c;
                let val = grid.get(r, c).unwrap_or(OccupancyValue::Unknown);
                out.set(r, new_c, val);
            }
        }
        out
    }

    /// 纵向翻转（上下镜像）
    #[must_use]
    pub fn flip_vertical(grid: &OccupancyGrid) -> OccupancyGrid {
        let mut out = grid.clone();
        for r in 0..grid.height {
            for c in 0..grid.width {
                let new_r = grid.height - 1 - r;
                let val = grid.get(r, c).unwrap_or(OccupancyValue::Unknown);
                out.set(new_r, c, val);
            }
        }
        out
    }

    /// 转置（行变列）
    #[must_use]
    pub fn transpose(grid: &OccupancyGrid) -> OccupancyGrid {
        let mut out = OccupancyGrid::new(grid.height, grid.width, grid.resolution);
        for r in 0..grid.height {
            for c in 0..grid.width {
                let val = grid.get(r, c).unwrap_or(OccupancyValue::Unknown);
                out.set(c, r, val);
            }
        }
        out
    }

    /// 合并：取每格最大值（-1 / 0 / 100 中的最大）
    #[must_use]
    pub fn merge_max(a: &OccupancyGrid, b: &OccupancyGrid) -> Option<OccupancyGrid> {
        if a.width != b.width || a.height != b.height {
            return None;
        }
        let mut out = a.clone();
        for r in 0..a.height {
            for c in 0..a.width {
                let va = a.get(r, c).unwrap_or(OccupancyValue::Unknown) as i8;
                let vb = b.get(r, c).unwrap_or(OccupancyValue::Unknown) as i8;
                let max = if va > vb { va } else { vb };
                out.set(r, c, OccupancyValue::from_i8(max));
            }
        }
        Some(out)
    }

    /// 合并：取每格最小值
    #[must_use]
    pub fn merge_min(a: &OccupancyGrid, b: &OccupancyGrid) -> Option<OccupancyGrid> {
        if a.width != b.width || a.height != b.height {
            return None;
        }
        let mut out = a.clone();
        for r in 0..a.height {
            for c in 0..a.width {
                let va = a.get(r, c).unwrap_or(OccupancyValue::Unknown) as i8;
                let vb = b.get(r, c).unwrap_or(OccupancyValue::Unknown) as i8;
                let min = if va < vb { va } else { vb };
                out.set(r, c, OccupancyValue::from_i8(min));
            }
        }
        Some(out)
    }

    /// 子区域裁剪
    #[must_use]
    pub fn crop(
        grid: &OccupancyGrid,
        row_start: u32,
        col_start: u32,
        height: u32,
        width: u32,
    ) -> Option<OccupancyGrid> {
        if row_start + height > grid.height || col_start + width > grid.width {
            return None;
        }
        let mut out = OccupancyGrid::new(width, height, grid.resolution);
        for r in 0..height {
            for c in 0..width {
                let val = grid
                    .get(row_start + r, col_start + c)
                    .unwrap_or(OccupancyValue::Unknown);
                out.set(r, c, val);
            }
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flip_horizontal_mirror() {
        let mut g = OccupancyGrid::new(3, 3, 0.1);
        g.set(1, 0, OccupancyValue::Occupied);
        let f = OccupancyOps::flip_horizontal(&g);
        // (1, 0) → (1, 2)
        assert_eq!(f.get(1, 2), Some(OccupancyValue::Occupied));
        assert_eq!(f.get(1, 0), Some(OccupancyValue::Unknown));
    }

    #[test]
    fn flip_vertical_mirror() {
        let mut g = OccupancyGrid::new(3, 3, 0.1);
        g.set(0, 1, OccupancyValue::Occupied);
        let f = OccupancyOps::flip_vertical(&g);
        // (0, 1) → (2, 1)
        assert_eq!(f.get(2, 1), Some(OccupancyValue::Occupied));
        assert_eq!(f.get(0, 1), Some(OccupancyValue::Unknown));
    }

    #[test]
    fn transpose_3x2_to_2x3() {
        let mut g = OccupancyGrid::new(3, 2, 0.1);
        g.set(0, 0, OccupancyValue::Occupied);
        let t = OccupancyOps::transpose(&g);
        assert_eq!(t.width, 2);
        assert_eq!(t.height, 3);
        assert_eq!(t.get(0, 0), Some(OccupancyValue::Occupied));
    }

    #[test]
    fn merge_max_unknown_free_occupied() {
        let mut a = OccupancyGrid::new(2, 1, 0.1);
        let mut b = OccupancyGrid::new(2, 1, 0.1);
        a.set(0, 0, OccupancyValue::Unknown); // -1
        b.set(0, 0, OccupancyValue::Free); // 0
        let merged = OccupancyOps::merge_max(&a, &b).unwrap();
        // max(-1, 0) = 0 = Free
        assert_eq!(merged.get(0, 0), Some(OccupancyValue::Free));

        a.set(0, 1, OccupancyValue::Free);
        b.set(0, 1, OccupancyValue::Occupied);
        let merged = OccupancyOps::merge_max(&a, &b).unwrap();
        assert_eq!(merged.get(0, 1), Some(OccupancyValue::Occupied));
    }

    #[test]
    fn crop_subregion() {
        let mut g = OccupancyGrid::new(10, 10, 0.1);
        g.set(2, 2, OccupancyValue::Occupied);
        let c = OccupancyOps::crop(&g, 1, 1, 3, 3).unwrap();
        assert_eq!(c.width, 3);
        assert_eq!(c.height, 3);
        // 原 (2,2) → crop 内 (1,1)
        assert_eq!(c.get(1, 1), Some(OccupancyValue::Occupied));
    }

    #[test]
    fn crop_out_of_bounds() {
        let g = OccupancyGrid::new(10, 10, 0.1);
        assert!(OccupancyOps::crop(&g, 8, 8, 5, 5).is_none());
    }
}
