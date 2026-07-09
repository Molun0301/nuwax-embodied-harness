//! 路径距离辅助
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/path_distancer.py`。
//!
//! 提供：
//! - 累积距离数组（按 segment 累加的 world-frame 距离）
//! - `find_lookahead_point`：从 `start_idx` 沿路径前进 `lookahead_dist` 米
//! - `distance_to_goal`：当前点到路径终点距离
//! - `get_distance_to_path`：当前点到路径最近点的距离(最近顶点距离,非线段投影;
//!   与 Python `dimos/navigation/replanning_a_star/path_distancer.py:68-75` 一致,
//!   避免线段投影 vs 点投影的语义差异;fixes 审计 H-17)
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
        let target_dist = base_dist + self.lookahead_dist;
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

    /// 当前点到路径最近点的距离
    ///
    /// 与 Python `path_distancer.py:68-70` 一致:返回 `pos` 到 `path_points[i]` 欧氏距离的
    /// 最小值,其中 `i = find_closest_point_index(pos)`(即最近顶点距离,非线段投影)。
    ///
    /// 为什么不用段投影:段投影会给出"投影到线段最近处"的距离;在路径拐弯处会高估
    /// 局部最小值,导致障碍物避让半径计算偏小。Python 原始实现采用顶点距离,语义
    /// 简洁且与 `find_closest_point_index` 调用约定一致。
    #[must_use]
    pub fn get_distance_to_path(&self, pos: Point2) -> f64 {
        if self.path_points.is_empty() {
            return 0.0;
        }
        let idx = self.find_closest_point_index(pos);
        distance(&self.path_points[idx], &pos)
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

    /// H-17 修复验证:Python `path_distancer.get_distance_to_path` 用最近顶点距离,
    /// 而非段投影。对 L 形路径,顶点距离与段投影在拐角处给出不同结果。
    #[test]
    fn distance_to_path_uses_vertex_not_segment_projection() {
        // L 形路径:(0,0) → (1,0) → (1,1);靠近拐角 (1,0) 但不在路径上的查询点 (1.1, 0.1)
        let path = make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
        ]);
        let pd = PathDistancer::new(&path, 0.5);
        let query = Point2::new(1.1, 0.1);
        // 最近顶点是 (1,0),distance = sqrt(0.1^2 + 0.1^2) ≈ 0.1414
        let expected = (0.1_f64 * 0.1 + 0.1_f64 * 0.1).sqrt();
        let d = pd.get_distance_to_path(query);
        assert!(
            (d - expected).abs() < 1e-9,
            "expected vertex distance {expected}, got {d}"
        );
        // 段投影会给出 (1.0, 0.1) 处的距离 0.1(更小),与 Python 不一致。
        // 因此该测试同时为修复行为的契约断言。
        let segment_projection_distance =
            // segment (1,0)→(1,1) 投影:clamp 到 t=0...1,本例 t=-0.01→clamp 0,得 (1,0)
            0.1_f64;
        assert!(
            d > segment_projection_distance + 1e-6,
            "vertex distance should exceed segment-projection distance for L-shape near corner"
        );
    }

    /// H-17 单测:S 形路径,查询点偏离路径中段,验证顶点距离语义稳定。
    #[test]
    fn distance_to_path_s_curve_vertex_distance() {
        // S 形:(0,0) → (1,1) → (2,0) → (3,1)
        let path = make_path2d(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 0.0),
            Point2::new(3.0, 1.0),
        ]);
        let pd = PathDistancer::new(&path, 0.5);
        // 中点 (1.5, 0.5) 距 (1,1)=sqrt(0.5)≈0.7071,距 (2,0)=sqrt(0.5)≈0.7071,距 (1.5 midy?)
        // 取绝对最近:argmin(|(1,1)-(1.5,0.5)|, |(2,0)-(1.5,0.5)|) — 两者相等 (≈0.7071),argmin 取 index 1
        let d = pd.get_distance_to_path(Point2::new(1.5, 0.5));
        let expected = ((1.0_f64 - 1.5).powi(2) + (1.0_f64 - 0.5).powi(2)).sqrt();
        assert!((d - expected).abs() < 1e-9, "expected {expected}, got {d}");
        // 另一个测试点:明显靠近 (2,0)
        let d2 = pd.get_distance_to_path(Point2::new(2.1, 0.1));
        let expected2 = ((2.0_f64 - 2.1).powi(2) + (0.0_f64 - 0.1).powi(2)).sqrt();
        assert!((d2 - expected2).abs() < 1e-9);
    }

    /// H-17 单测:直线路径,偏离路径中点。验证段投影与顶点距离相等时(中点附近)结果一致。
    #[test]
    fn distance_to_path_straight_aligned_with_vertex() {
        let pd = PathDistancer::new(&straight_path(), 0.5);
        // 偏离 (1,0) 0.3 米向右 (1,0.3):最近顶点仍为 (1,0),距离 0.3
        let d = pd.get_distance_to_path(Point2::new(1.0, 0.3));
        assert!((d - 0.3).abs() < 1e-9);
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
