//! 路径距离场
//! BFS 多源距离场：从每个 occupied cell 开始向外 BFS，记录每个 free cell
//! 到最近 occupied 的曼哈顿距离。

use crate::occupancy::types::{OccupancyGrid, OccupancyValue};
use ndarray::Array2;
use std::collections::VecDeque;

/// 路径距离场
pub struct PathMap;

impl PathMap {
    /// 计算每格到最近 occupied cell 的距离（cell 数）
    ///
    /// 返回与输入栅格同尺寸的 `Array2<u32>`，occupied 与 unknown 位置为 0。
    #[must_use]
    pub fn distance_field(grid: &OccupancyGrid) -> Array2<u32> {
        let mut dist = Array2::from_elem((grid.height as usize, grid.width as usize), 0u32);
        let mut visited = Array2::from_elem((grid.height as usize, grid.width as usize), false);
        let mut queue: VecDeque<(u32, u32)> = VecDeque::new();

        // 多源 BFS：从所有 occupied cell 开始
        for r in 0..grid.height {
            for c in 0..grid.width {
                if matches!(grid.get(r, c), Some(OccupancyValue::Occupied)) {
                    queue.push_back((r, c));
                    visited[(r as usize, c as usize)] = true;
                }
            }
        }

        // 4 邻域 BFS
        while let Some((r, c)) = queue.pop_front() {
            let cur_dist = dist[(r as usize, c as usize)];
            for (dr, dc) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
                let nr = r as i32 + dr;
                let nc = c as i32 + dc;
                if nr < 0 || nc < 0 || nr >= grid.height as i32 || nc >= grid.width as i32 {
                    continue;
                }
                let (nru, ncu) = (nr as usize, nc as usize);
                if visited[(nru, ncu)] {
                    continue;
                }
                if matches!(
                    grid.get(nr as u32, nc as u32),
                    Some(OccupancyValue::Occupied)
                ) {
                    continue;
                }
                visited[(nru, ncu)] = true;
                dist[(nru, ncu)] = cur_dist + 1;
                queue.push_back((nr as u32, nc as u32));
            }
        }

        dist
    }

    /// 计算从 `start` 到 `goal` 的最短路径长度（cell 数，BFS）
    ///
    /// Returns `None` 当路径不可达。
    #[must_use]
    pub fn shortest_path_length(
        grid: &OccupancyGrid,
        start: (u32, u32),
        goal: (u32, u32),
    ) -> Option<u32> {
        if start == goal {
            return Some(0);
        }
        if matches!(grid.get(start.0, start.1), Some(OccupancyValue::Occupied))
            || matches!(grid.get(goal.0, goal.1), Some(OccupancyValue::Occupied))
        {
            return None;
        }
        let mut visited = Array2::from_elem((grid.height as usize, grid.width as usize), false);
        let mut queue: VecDeque<((u32, u32), u32)> = VecDeque::new();
        queue.push_back((start, 0));
        visited[(start.0 as usize, start.1 as usize)] = true;

        while let Some(((r, c), d)) = queue.pop_front() {
            for (dr, dc) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
                let nr = r as i32 + dr;
                let nc = c as i32 + dc;
                if nr < 0 || nc < 0 || nr >= grid.height as i32 || nc >= grid.width as i32 {
                    continue;
                }
                let (nru, ncu) = (nr as usize, nc as usize);
                if visited[(nru, ncu)] {
                    continue;
                }
                if matches!(
                    grid.get(nr as u32, nc as u32),
                    Some(OccupancyValue::Occupied)
                ) {
                    continue;
                }
                let new_pos = (nr as u32, nc as u32);
                if new_pos == goal {
                    return Some(d + 1);
                }
                visited[(nru, ncu)] = true;
                queue.push_back((new_pos, d + 1));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_field_basic() {
        let mut g = OccupancyGrid::new(5, 5, 0.1);
        g.set(2, 2, OccupancyValue::Occupied);
        let df = PathMap::distance_field(&g);
        // 中心是 0
        assert_eq!(df[(2, 2)], 0);
        // 4 邻域是 1
        assert_eq!(df[(1, 2)], 1);
        assert_eq!(df[(3, 2)], 1);
        assert_eq!(df[(2, 1)], 1);
        assert_eq!(df[(2, 3)], 1);
        // 角落是 4
        assert_eq!(df[(0, 0)], 4);
    }

    #[test]
    fn shortest_path_horizontal() {
        let mut g = OccupancyGrid::new(5, 1, 0.1);
        // 障碍在中间
        g.set(0, 2, OccupancyValue::Occupied);
        // 起点 (0,0) → 终点 (0,4)：必须绕过 → 不可达（1D）
        assert_eq!(PathMap::shortest_path_length(&g, (0, 0), (0, 4)), None);
    }

    #[test]
    fn shortest_path_clear() {
        let g = OccupancyGrid::new(5, 1, 0.1);
        // (0,0) → (0,4)：4 步
        assert_eq!(PathMap::shortest_path_length(&g, (0, 0), (0, 4)), Some(4));
    }

    #[test]
    fn shortest_path_same() {
        let g = OccupancyGrid::new(3, 3, 0.1);
        assert_eq!(PathMap::shortest_path_length(&g, (1, 1), (1, 1)), Some(0));
    }
}
