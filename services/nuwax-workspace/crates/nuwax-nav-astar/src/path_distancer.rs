//! 路径距离辅助
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/path_distancer.py`。
//!
//! 提供：
//! - 累积距离数组（按 segment 累加的 world-frame 距离）
//! - `find_lookahead_point`：从 `start_idx` 沿路径前进 `lookahead_dist` 米
//! - `distance_to_goal`：当前点到路径终点距离
//! - `get_distance_to_path`：当前点到路径最近线段投影的距离
//! - `find_lookahead_from_position`：从当前投影位置向前推进，支持稀疏路径
//! - `find_closest_point_index`：路径最近点 index

use crate::helpers::{Path2D, Point2};
// 注：本文件不直接调用 make_* helpers; Path2D 由调用方提供

/// 路径距离辅助
#[derive(Debug, Clone)]
pub struct PathDistancer {
    path_points: Vec<Point2>,
    /// 每个 segment 的距离：`cum[i] = distance from path[0] to path[i+1]`。
    cumulative_dists: Vec<f64>,
    lookahead_dist: f64,
}

impl PathDistancer {
    /// 构造
    ///
    /// # 参数
    /// - `path`：2D 路径（点列）
    /// - `lookahead_dist`：lookahead 距离（米，默认 0.5）
    #[must_use]
    pub fn new(path: &Path2D, lookahead_dist: f64) -> Self {
        let path_points = path.points.clone();
        let cumulative_dists = make_cumulative_distance_array(&path_points);
        Self {
            path_points,
            cumulative_dists,
            lookahead_dist,
        }
    }

    /// lookahead 距离（米）
    #[must_use]
    pub fn lookahead_distance(&self) -> f64 {
        self.lookahead_dist
    }

    /// 路径长度（米）
    #[must_use]
    pub fn length(&self) -> f64 {
        self.cumulative_dists.last().copied().unwrap_or(0.0)
    }

    /// 找 lookahead point
    ///
    /// 与 Python 行为一致：
    /// - `start_idx >= len(path) - 1` → 返回 path 终点
    /// - 在 cumulative_dists 上做 binary search
    /// - 在 segment 上做线性插值
    #[must_use]
    pub fn find_lookahead_point(&self, start_idx: usize) -> Point2 {
        let n = self.path_points.len();
        if n == 0 {
            return Point2::new(0.0, 0.0);
        }
        if start_idx >= n.saturating_sub(1) {
            return self.path_points[n - 1];
        }
        let base_dist = if start_idx == 0 {
            0.0
        } else {
            self.cumulative_dists[start_idx - 1]
        };
        self.point_at_distance(base_dist + self.lookahead_dist)
    }

    /// 从当前位置的最近线段投影沿路径前进，而非固定在最近顶点的前瞻点。
    #[must_use]
    pub fn find_lookahead_from_position(&self, pos: Point2) -> Point2 {
        let (segment, projected) = self.closest_projection(pos);
        let base = if segment == 0 {
            0.0
        } else {
            self.cumulative_dists[segment - 1]
        };
        let along_segment = self
            .path_points
            .get(segment)
            .map_or(0.0, |start| distance(start, &projected));
        self.point_at_distance(base + along_segment + self.lookahead_dist)
    }

    fn point_at_distance(&self, target_dist: f64) -> Point2 {
        let n = self.path_points.len();
        if n == 0 {
            return Point2::new(0.0, 0.0);
        }
        if n == 1 {
            return self.path_points[0];
        }
        let idx = searchsorted(&self.cumulative_dists, target_dist);
        if idx >= self.cumulative_dists.len() {
            return self.path_points[n - 1];
        }
        let prev_cum = if idx == 0 {
            0.0
        } else {
            self.cumulative_dists[idx - 1]
        };
        let segment_dist = self.cumulative_dists[idx] - prev_cum;
        let remaining = target_dist - prev_cum;
        if segment_dist > 0.0 {
            let t = remaining / segment_dist;
            lerp_point(self.path_points[idx], self.path_points[idx + 1], t)
        } else {
            self.path_points[idx]
        }
    }

    /// 当前点到路径终点的距离
    #[must_use]
    pub fn distance_to_goal(&self, current_pos: Point2) -> f64 {
        if self.path_points.is_empty() {
            return 0.0;
        }
        distance(&self.path_points[self.path_points.len() - 1], &current_pos)
    }

    /// 当前点到路径最近线段的距离；路径采样稀疏不应被误判为偏离路径。
    #[must_use]
    pub fn get_distance_to_path(&self, pos: Point2) -> f64 {
        if self.path_points.is_empty() {
            return 0.0;
        }
        let (_, projected) = self.closest_projection(pos);
        distance(&projected, &pos)
    }

    /// 返回最近线段的起点索引与该线段上的投影，重复点作为零长度线段处理。
    #[must_use]
    pub fn closest_projection(&self, pos: Point2) -> (usize, Point2) {
        let Some(first) = self.path_points.first().copied() else {
            return (0, Point2::new(0.0, 0.0));
        };
        let mut best = (0, first);
        let mut best_distance = distance(&first, &pos);
        for (index, segment) in self.path_points.windows(2).enumerate() {
            let dx = segment[1].x - segment[0].x;
            let dy = segment[1].y - segment[0].y;
            let length_squared = dx * dx + dy * dy;
            let fraction = if length_squared > 0.0 {
                (((pos.x - segment[0].x) * dx + (pos.y - segment[0].y) * dy) / length_squared)
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            let projected = lerp_point(segment[0], segment[1], fraction);
            let candidate_distance = distance(&projected, &pos);
            if candidate_distance < best_distance {
                best = (index, projected);
                best_distance = candidate_distance;
            }
        }
        best
    }

    /// 当前点到路径最近点的 index
    #[must_use]
    pub fn find_closest_point_index(&self, pos: Point2) -> usize {
        let mut best = 0usize;
        let mut best_d = f64::INFINITY;
        for (i, p) in self.path_points.iter().enumerate() {
            let d = distance(p, &pos);
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        best
    }

    /// 路径点
    #[must_use]
    pub fn points(&self) -> &[Point2] {
        &self.path_points
    }
}

fn make_cumulative_distance_array(points: &[Point2]) -> Vec<f64> {
    if points.len() < 2 {
        return vec![0.0];
    }
    let mut cum = Vec::with_capacity(points.len() - 1);
    let mut total = 0.0;
    for w in points.windows(2) {
        total += distance(&w[0], &w[1]);
        cum.push(total);
    }
    cum
}

#[inline]
fn distance(a: &Point2, b: &Point2) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}

#[inline]
fn lerp_point(a: Point2, b: Point2, t: f64) -> Point2 {
    Point2::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
}

/// binary search：返回第一个 `>= target` 的 index；若都 < target 则返回 `len()`。
fn searchsorted(arr: &[f64], target: f64) -> usize {
    let mut lo = 0usize;
    let mut hi = arr.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if arr[mid] < target {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::make_path2d;

    fn straight_path() -> Path2D {
        make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(3.0, 0.0),
        ])
    }

    #[test]
    fn cumulative_distances_for_straight_path() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        assert!((pd.cumulative_dists[0] - 1.0).abs() < 1e-9);
        assert!((pd.cumulative_dists[2] - 3.0).abs() < 1e-9);
        assert!((pd.length() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn lookahead_at_start() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        let p = pd.find_lookahead_point(0);
        // base_dist = 0, target_dist = 0.5, idx=0 (cum[0]=1.0 >= 0.5), segment 0->1, t=0.5
        assert!((p.x - 0.5).abs() < 1e-9);
    }

    #[test]
    fn lookahead_past_end_returns_end() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        let p = pd.find_lookahead_point(3);
        // start_idx >= len-1 → 返回 path[-1]
        assert!((p.x - 3.0).abs() < 1e-9);
    }

    #[test]
    fn closest_index() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        let i = pd.find_closest_point_index(Point2::new(2.2, 0.05));
        assert_eq!(i, 2);
    }

    #[test]
    fn distance_to_goal() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        let d = pd.distance_to_goal(Point2::new(0.0, 0.0));
        assert!((d - 3.0).abs() < 1e-9);
    }

    #[test]
    fn distance_to_path_is_zero_on_path() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        let d = pd.get_distance_to_path(Point2::new(2.0, 0.0));
        assert!(d.abs() < 1e-9);
    }

    #[test]
    fn sparse_path_distance_and_lookahead_use_segment_projection() {
        let path = make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)]);
        let pd = PathDistancer::new(&path, 0.5);
        for x in [0.5, 1.0, 2.0, 3.0] {
            assert!((pd.get_distance_to_path(Point2::new(x, 0.3)) - 0.3).abs() < 1e-9);
            let ahead = pd.find_lookahead_from_position(Point2::new(x, 0.0));
            assert!((ahead.x - x - 0.5).abs() < 1e-9);
        }
    }

    #[test]
    fn projected_lookahead_follows_bends_and_ignores_repeated_points() {
        let path = make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
        ]);
        let pd = PathDistancer::new(&path, 0.5);
        assert!((pd.get_distance_to_path(Point2::new(1.1, 0.1)) - 0.1).abs() < 1e-9);
        let ahead = pd.find_lookahead_from_position(Point2::new(0.8, 0.0));
        assert!((ahead.x - 1.0).abs() < 1e-9);
        assert!((ahead.y - 0.3).abs() < 1e-9);
        assert_eq!(
            pd.find_lookahead_from_position(Point2::new(1.0, 0.9)),
            Point2::new(1.0, 1.0),
        );
    }

    #[test]
    fn empty_path_returns_zero() {
        let path = crate::helpers::make_path2d_empty();
        let pd = PathDistancer::new(&path, 0.5);
        assert_eq!(pd.length(), 0.0);
        assert_eq!(pd.distance_to_goal(Point2::new(1.0, 1.0)), 0.0);
        assert_eq!(pd.get_distance_to_path(Point2::new(1.0, 1.0)), 0.0);
        assert_eq!(pd.find_closest_point_index(Point2::new(0.0, 0.0)), 0);
    }
}
