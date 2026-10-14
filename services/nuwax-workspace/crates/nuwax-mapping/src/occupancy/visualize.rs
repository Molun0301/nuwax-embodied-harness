//! 占据栅格可视化
//! 提供简单的 ASCII 与 Rerun 桥接 stub。

use crate::occupancy::types::{OccupancyGrid, OccupancyValue};

/// 占据栅格可视化器
pub struct OccupancyVisualize;

impl OccupancyVisualize {
    /// 渲染为 ASCII 字符画（用于终端调试）
    ///
    /// 每个 cell 显示：`#` = Occupied, `.` = Free, `?` = Unknown
    #[must_use]
    pub fn to_ascii(grid: &OccupancyGrid) -> String {
        let mut out = String::with_capacity(((grid.width + 1) * grid.height) as usize);
        for r in 0..grid.height {
            for c in 0..grid.width {
                let ch = match grid.get(r, c) {
                    Some(OccupancyValue::Occupied) => '#',
                    Some(OccupancyValue::Free) => '.',
                    _ => '?',
                };
                out.push(ch);
            }
            if r + 1 < grid.height {
                out.push('\n');
            }
        }
        out
    }

    /// Rerun 桥接 stub（实际实装需要 nuwax-vision-rerun 集成；本阶段返回 SVG 文本占位）
    #[must_use]
    pub fn to_rerun_placeholder(grid: &OccupancyGrid) -> String {
        format!(
            "<svg width='{}' height='{}' xmlns='http://www.w3.org/2000/svg'>{}</svg>",
            grid.width,
            grid.height,
            // 占位：未来用 nuwax-viewer / rerun 替换
            String::new()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_ascii_basic() {
        let mut g = OccupancyGrid::new(3, 1, 0.1);
        g.set(0, 1, OccupancyValue::Occupied);
        let ascii = OccupancyVisualize::to_ascii(&g);
        assert_eq!(ascii, "?#?");
    }

    #[test]
    fn to_ascii_multiline() {
        let mut g = OccupancyGrid::new(2, 2, 0.1);
        g.set(0, 0, OccupancyValue::Occupied);
        let ascii = OccupancyVisualize::to_ascii(&g);
        assert_eq!(ascii, "#?\n??");
    }

    #[test]
    fn to_rerun_placeholder_non_empty() {
        let g = OccupancyGrid::new(10, 10, 0.1);
        let s = OccupancyVisualize::to_rerun_placeholder(&g);
        assert!(s.contains("<svg"));
    }
}
