//! 路径前向障碍检测
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/path_clearance.py`。
//!
//! Phase 1 实现：
//! - `PathClearance` 缓存最近一次 costmap 与 pose_index。
//! - `mask`：根据 `robot_width`（米）计算圆盘膨胀（cell 单位），并结合 `path_lookup_distance`
//!   限制只覆盖路径前向 `path_lookup_distance` 米。
//! - `is_obstacle_ahead()`：检查 mask 覆盖的 cell 是否有 OCCUPIED。
//!
//! 与 Python `make_path_mask` 的真实实现相比，本阶段使用等效几何：圆盘 mask + 路径段 clip。
//! 复杂 `make_path_mask`（涉及 rasterize / rotation）放在后续 PR 与 mapping 协作完成。

use crate::helpers::Path2D;
use crate::navigation_map::{CostMap2D, OCCUPIED};
// 注：本文件不直接调用 make_* helpers; Path2D 由调用方提供(make_path2d 构造)

/// 路径前向障碍检测
#[derive(Debug, Clone)]
pub struct PathClearance {
    path: Path2D,
    robot_width: f64,
    path_lookup_distance: f64,
    max_distance_cache: f64,
    costmap: Option<CostMap2D>,
    pose_index: usize,
    last_costmap_shape: Option<(usize, usize)>,
    last_pose_index: usize,
    last_mask: Option<Vec<bool>>,
}

impl PathClearance {
    /// 构造
    ///
    /// # 参数
    /// - `path`：路径
    /// - `robot_width`：机器人宽度（米），用于圆盘膨胀半径
    /// - `path_lookup_distance`：从当前 pose_index 向前的查找距离（米，默认 3.0）
    #[must_use]
    pub fn new(path: Path2D, robot_width: f64, path_lookup_distance: f64) -> Self {
        Self {
            path,
            robot_width,
            path_lookup_distance,
            max_distance_cache: 1.0,
            costmap: None,
            pose_index: 0,
            last_costmap_shape: None,
            last_pose_index: 0,
            last_mask: None,
        }
    }

    /// 更新 costmap
    pub fn update_costmap(&mut self, costmap: CostMap2D) {
        self.costmap = Some(costmap);
    }

    /// 更新 pose_index（当前路径上的进度）
    pub fn update_pose_index(&mut self, index: usize) {
        self.pose_index = index;
    }

    /// 是否前方有 obstacle
    #[must_use]
    pub fn is_obstacle_ahead(&mut self) -> bool {
        if self.costmap.is_none() {
            return true;
        }
        let mask = self.compute_mask();
        let Some(costmap) = self.costmap.as_ref() else {
            return true;
        };
        let width = costmap.width();
        let height = costmap.height();
        for y in 0..height {
            for x in 0..width {
                if mask[y * width + x] && costmap.get(x, y) >= OCCUPIED {
                    return true;
                }
            }
        }
        false
    }

    /// 计算 mask（bool grid）
    ///
    /// 算法：
    /// - 取从 `pose_index` 开始、累积距离 `≤ path_lookup_distance` 的路径段。
    /// - 在路径段每个 cell 周围画半径 `robot_width / 2 / resolution` 的圆盘（cell 单位）。
    /// - 合并成 mask。
    #[must_use]
    pub fn compute_mask(&mut self) -> Vec<bool> {
        let Some(costmap) = self.costmap.as_ref() else {
            return Vec::new();
        };
        let shape = (costmap.width(), costmap.height());
        if let Some(prev_shape) = self.last_costmap_shape {
            if prev_shape == shape {
                let pose_dist = self.pose_distance(self.last_pose_index, self.pose_index);
                if pose_dist < self.max_distance_cache && self.last_mask.is_some() {
                    return self.last_mask.clone().unwrap();
                }
            }
        }

        let width = costmap.width();
        let height = costmap.height();
        let mut mask = vec![false; width * height];
        let radius_cells = ((self.robot_width * 0.5) / costmap.resolution()).ceil() as i32;
        let r2 = radius_cells * radius_cells;

        let points = &self.path.points;
        if points.is_empty() {
            return mask;
        }
        let n = points.len();
        let start_idx = self.pose_index.min(n.saturating_sub(1));
        let mut remaining = self.path_lookup_distance;

        // 累加距离推进；超过前瞻距离的路径点不能进入 mask。
        let mut idx = start_idx;
        loop {
            let p = &points[idx];
            let (cx, cy) = costmap.world_to_cell(*p);
            if costmap.in_bounds(cx, cy) {
                self.stamp_disk(&mut mask, width, height, cx, cy, radius_cells, r2);
            }

            if idx + 1 >= n {
                break;
            }
            let next = &points[idx + 1];
            let seg_len = ((next.x - p.x).powi(2) + (next.y - p.y).powi(2)).sqrt();
            if seg_len > remaining {
                break;
            }
            remaining -= seg_len;
            idx += 1;
        }

        self.last_costmap_shape = Some(shape);
        self.last_pose_index = self.pose_index;
        self.last_mask = Some(mask.clone());
        mask
    }

    #[allow(clippy::too_many_arguments)] // 函数为内联热路径,分组会增加栈分配
    fn stamp_disk(
        &self,
        mask: &mut [bool],
        width: usize,
        height: usize,
        cx: i32,
        cy: i32,
        radius: i32,
        r2: i32,
    ) {
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy > r2 {
                    continue;
                }
                let nx = cx + dx;
                let ny = cy + dy;
                if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
                    mask[ny as usize * width + nx as usize] = true;
                }
            }
        }
    }

    fn pose_distance(&self, idx1: usize, idx2: usize) -> f64 {
        let points = &self.path.points;
        if idx1 >= points.len() || idx2 >= points.len() {
            return f64::INFINITY;
        }
        let a = &points[idx1];
        let b = &points[idx2];
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// 当前 path 引用（只读）
    #[must_use]
    pub fn path(&self) -> &Path2D {
        &self.path
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::{make_path2d, Point2};
    use crate::navigation_map::CostMap2D;

    fn path_through_straight_line() -> Path2D {
        make_path2d((0..=10).map(|i| Point2::new(i as f64 * 0.5, 0.0)).collect())
    }

    #[test]
    fn is_obstacle_ahead_returns_false_when_clear() {
        let cm = CostMap2D::new_all_free("map", 30, 10, 0.1);
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, 0.3, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(!pc.is_obstacle_ahead());
    }

    #[test]
    fn is_obstacle_ahead_returns_true_when_occupied() {
        let mut cm = CostMap2D::new_all_free("map", 30, 10, 0.1);
        // 在路径前向 1.5m 处放 obstacle（cell 15,5 = world (1.55, 0.55)）
        cm.set(15, 5, OCCUPIED);
        let path = path_through_straight_line();
        // robot_width=1.2 让 disk 半径覆盖 y=5
        let mut pc = PathClearance::new(path, 1.2, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(pc.is_obstacle_ahead());
    }

    #[test]
    fn is_obstacle_ahead_returns_true_without_costmap() {
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, 0.3, 2.0);
        assert!(pc.is_obstacle_ahead());
    }

    #[test]
    fn mask_respects_path_lookup_distance() {
        // obstacle 远离 path_lookup_distance：不应被检测
        let mut cm = CostMap2D::new_all_free("map", 60, 10, 0.1);
        cm.set(50, 5, OCCUPIED);
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, 0.3, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(!pc.is_obstacle_ahead());
    }

    #[test]
    fn mask_does_not_include_a_far_endpoint_on_a_two_point_path() {
        let mut cm = CostMap2D::new_all_free("map", 60, 10, 0.1);
        cm.set(50, 0, OCCUPIED);
        let path = make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(5.0, 0.0)]);
        let mut pc = PathClearance::new(path, 0.1, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);

        assert!(!pc.is_obstacle_ahead());
    }
}
