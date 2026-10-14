//! 路径掩码
//! 把一条 cell 级路径转换为栅格掩码（bool grid，路径经过的 cell 为 true）。

use crate::occupancy::types::OccupancyGrid;

/// 路径掩码
pub struct PathMask;

impl PathMask {
    /// 从路径生成掩码
    ///
    /// Returns 与输入栅格同尺寸的 OccupancyGrid，路径经过的 cell 标记为 Free（0），
    /// 其他位置标记为 Unknown（-1）。
    #[must_use]
    pub fn from_path(grid: &OccupancyGrid, path: &[(u32, u32)]) -> OccupancyGrid {
        let mut out = OccupancyGrid::new(grid.width, grid.height, grid.resolution);
        // 默认全部 Unknown（init 时已经设置）
        use crate::occupancy::types::OccupancyValue;
        for &(r, c) in path {
            if r < grid.height && c < grid.width {
                out.set(r, c, OccupancyValue::Free);
            }
        }
        out
    }

    /// 提取掩码中的路径（按行优先遍历）
    #[must_use]
    pub fn to_path(mask: &OccupancyGrid) -> Vec<(u32, u32)> {
        use crate::occupancy::types::OccupancyValue;
        let mut out = Vec::new();
        for r in 0..mask.height {
            for c in 0..mask.width {
                if matches!(mask.get(r, c), Some(OccupancyValue::Free)) {
                    out.push((r, c));
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::types::OccupancyValue;

    #[test]
    fn from_path_marks_cells_free() {
        let g = OccupancyGrid::new(5, 5, 0.1);
        let path = vec![(0, 0), (1, 0), (2, 0), (3, 0)];
        let mask = PathMask::from_path(&g, &path);
        assert_eq!(mask.get(0, 0), Some(OccupancyValue::Free));
        assert_eq!(mask.get(1, 0), Some(OccupancyValue::Free));
        assert_eq!(mask.get(4, 4), Some(OccupancyValue::Unknown));
    }

    #[test]
    fn roundtrip() {
        let g = OccupancyGrid::new(5, 5, 0.1);
        let path = vec![(0, 0), (1, 1), (2, 2)];
        let mask = PathMask::from_path(&g, &path);
        let back = PathMask::to_path(&mask);
        assert_eq!(back, path);
    }

    #[test]
    fn out_of_bounds_path_ignored() {
        let g = OccupancyGrid::new(3, 3, 0.1);
        let path = vec![(0, 0), (10, 10)]; // 越界
        let mask = PathMask::from_path(&g, &path);
        // 只有 (0, 0) 标记为 Free
        assert_eq!(mask.get(0, 0), Some(OccupancyValue::Free));
        assert_eq!(mask.get(1, 0), Some(OccupancyValue::Unknown));
    }
}
