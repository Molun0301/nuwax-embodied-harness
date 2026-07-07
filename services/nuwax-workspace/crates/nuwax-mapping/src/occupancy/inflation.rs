//! 占据栅格膨胀
//! 简单 BFS 膨胀：把已占据 cell 周围 radius 范围内的 cell 标记为 INFLATED。

use crate::occupancy::types::{OccupancyGrid, OccupancyValue};
use std::collections::VecDeque;

/// 膨胀配置
#[derive(Debug, Clone, Copy)]
pub struct InflationConfig {
    /// 膨胀半径（cell）
    pub radius: u32,
    /// 是否使用对角膨胀（8 邻域）
    pub diagonal: bool,
}

impl Default for InflationConfig {
    fn default() -> Self {
        Self {
            radius: 3,
            diagonal: true,
        }
    }
}

/// 简单 inflate
///
/// 对每个 occupied cell，将其 radius 范围内所有 cell 标记为 Occupied。
/// 返回新的栅格。
#[must_use]
pub fn simple_inflate(grid: &OccupancyGrid, config: InflationConfig) -> OccupancyGrid {
    let mut out = grid.clone();
    let radius = config.radius as i32;

    // 先收集所有 occupied cell
    let occupied: Vec<(u32, u32)> = (0..grid.height)
        .flat_map(|r| (0..grid.width).map(move |c| (r, c)))
        .filter(|&(r, c)| matches!(grid.get(r, c), Some(OccupancyValue::Occupied)))
        .collect();

    // BFS 膨胀
    for &(or, oc) in &occupied {
        let mut visited = vec![false; (grid.width * grid.height) as usize];
        let mut queue: VecDeque<(i32, i32, u32)> = VecDeque::new();
        queue.push_back((or as i32, oc as i32, 0));

        while let Some((r, c, depth)) = queue.pop_front() {
            if depth > radius as u32 {
                continue;
            }
            if r < 0 || c < 0 || r >= grid.height as i32 || c >= grid.width as i32 {
                continue;
            }
            let (ru, cu) = (r as u32, c as u32);
            let idx = (ru * grid.width + cu) as usize;
            if visited[idx] {
                continue;
            }
            visited[idx] = true;

            // 当前 cell 标记
            let cur = out.get(ru, cu).unwrap_or(OccupancyValue::Unknown);
            if matches!(cur, OccupancyValue::Free | OccupancyValue::Unknown) {
                out.set(ru, cu, OccupancyValue::Occupied);
            }

            // 8 邻域扩散
            let neighbors: [(i32, i32); 8] = if config.diagonal {
                [
                    (-1, -1),
                    (-1, 0),
                    (-1, 1),
                    (0, -1),
                    (0, 1),
                    (1, -1),
                    (1, 0),
                    (1, 1),
                ]
            } else {
                [
                    (-1, 0),
                    (0, -1),
                    (0, 1),
                    (1, 0),
                    (0, 0),
                    (0, 0),
                    (0, 0),
                    (0, 0), // 填充
                ]
            };

            for (dr, dc) in neighbors.iter().take(if config.diagonal { 8 } else { 4 }) {
                queue.push_back((r + dr, c + dc, depth + 1));
            }
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inflate_radius_one() {
        let mut g = OccupancyGrid::new(5, 5, 0.1);
        g.set(2, 2, OccupancyValue::Occupied);

        let cfg = InflationConfig {
            radius: 1,
            diagonal: false,
        };
        let inflated = simple_inflate(&g, cfg);

        // 中心仍是 Occupied
        assert_eq!(inflated.get(2, 2), Some(OccupancyValue::Occupied));
        // 4 邻域变为 Occupied
        assert_eq!(inflated.get(1, 2), Some(OccupancyValue::Occupied));
        assert_eq!(inflated.get(3, 2), Some(OccupancyValue::Occupied));
        assert_eq!(inflated.get(2, 1), Some(OccupancyValue::Occupied));
        assert_eq!(inflated.get(2, 3), Some(OccupancyValue::Occupied));
        // 对角线未变（diagonal=false）
        assert_eq!(inflated.get(1, 1), Some(OccupancyValue::Unknown));
    }

    #[test]
    fn inflate_diagonal_radius_two() {
        let mut g = OccupancyGrid::new(7, 7, 0.1);
        g.set(3, 3, OccupancyValue::Occupied);

        let cfg = InflationConfig {
            radius: 2,
            diagonal: true,
        };
        let inflated = simple_inflate(&g, cfg);

        // 距中心 2 的对角线 cell 应该是 Occupied
        assert_eq!(inflated.get(1, 1), Some(OccupancyValue::Occupied));
        // 距中心 2 的正交 cell 应该是 Occupied
        assert_eq!(inflated.get(3, 1), Some(OccupancyValue::Occupied));
        // 距中心 > 2 的 cell 不变
        assert_eq!(inflated.get(0, 0), Some(OccupancyValue::Unknown));
    }
}
