//! 路径重采样
//! 提供：
//! - Douglas-Peucker 简化
//! - 等距重采样（按固定步长插值）
//! - 平滑（Catmull-Rom 样条）

use nalgebra::Point2;

/// 路径重采样器
pub struct PathResampling;

impl PathResampling {
    /// Douglas-Peucker 路径简化
    ///
    /// `epsilon` 为点到直线的最大允许垂直距离（与 path 单位一致）。
    /// Returns 简化后的路径点序列（保持首尾）。
    #[must_use]
    pub fn douglas_peucker(path: &[Point2<f64>], epsilon: f64) -> Vec<Point2<f64>> {
        if path.len() < 3 {
            return path.to_vec();
        }
        let mut keep = vec![false; path.len()];
        keep[0] = true;
        keep[path.len() - 1] = true;
        Self::dp_recurse(path, 0, path.len() - 1, epsilon, &mut keep);
        path.iter()
            .zip(keep.iter())
            .filter(|(_, k)| **k)
            .map(|(p, _)| *p)
            .collect()
    }

    fn dp_recurse(path: &[Point2<f64>], start: usize, end: usize, epsilon: f64, keep: &mut [bool]) {
        if end <= start + 1 {
            return;
        }
        let a = path[start];
        let b = path[end];
        let mut max_dist = 0.0;
        let mut max_idx = start;

        for (i, &p) in path.iter().enumerate().take(end).skip(start + 1) {
            let d = Self::perp_distance(p, a, b);
            if d > max_dist {
                max_dist = d;
                max_idx = i;
            }
        }

        if max_dist > epsilon {
            keep[max_idx] = true;
            Self::dp_recurse(path, start, max_idx, epsilon, keep);
            Self::dp_recurse(path, max_idx, end, epsilon, keep);
        }
    }

    /// 点到线段 (a, b) 的垂直距离
    fn perp_distance(p: Point2<f64>, a: Point2<f64>, b: Point2<f64>) -> f64 {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let len_sq = dx * dx + dy * dy;
        if len_sq < 1e-12 {
            // 退化：点到点距离
            return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
        }
        let num = (dy * p.x - dx * p.y + b.x * a.y - b.y * a.x).abs();
        num / len_sq.sqrt()
    }

    /// 等距重采样（沿折线按固定 step 重新分布点）
    #[must_use]
    pub fn resample_by_step(path: &[Point2<f64>], step: f64) -> Vec<Point2<f64>> {
        if path.len() < 2 {
            return path.to_vec();
        }
        let mut cum_dist = vec![0.0_f64];
        for i in 1..path.len() {
            let seg =
                ((path[i].x - path[i - 1].x).powi(2) + (path[i].y - path[i - 1].y).powi(2)).sqrt();
            cum_dist.push(cum_dist[i - 1] + seg);
        }
        let total = cum_dist[cum_dist.len() - 1];
        let mut out = Vec::new();
        let mut target = 0.0;
        let mut seg_idx = 0;
        while target < total {
            // 找到 target 所在段
            while seg_idx < cum_dist.len() - 1 && cum_dist[seg_idx + 1] < target {
                seg_idx += 1;
            }
            let d0 = cum_dist[seg_idx];
            let d1 = cum_dist[seg_idx + 1];
            let t = if (d1 - d0).abs() < 1e-12 {
                0.0
            } else {
                (target - d0) / (d1 - d0)
            };
            let p0 = path[seg_idx];
            let p1 = path[seg_idx + 1];
            out.push(Point2::new(
                p0.x + t * (p1.x - p0.x),
                p0.y + t * (p1.y - p0.y),
            ));
            target += step;
        }
        // 确保终点在
        out.push(*path.last().unwrap());
        out
    }

    /// 路径总长度
    #[must_use]
    pub fn total_length(path: &[Point2<f64>]) -> f64 {
        if path.len() < 2 {
            return 0.0;
        }
        (0..path.len() - 1)
            .map(|i| {
                ((path[i + 1].x - path[i].x).powi(2) + (path[i + 1].y - path[i].y).powi(2)).sqrt()
            })
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::point;

    #[test]
    fn douglas_peucker_short_path() {
        let p = vec![point![0.0, 0.0], point![1.0, 1.0]];
        let r = PathResampling::douglas_peucker(&p, 0.1);
        assert_eq!(r.len(), 2);
    }

    #[test]
    fn douglas_peucker_collinear() {
        let p = vec![
            point![0.0, 0.0],
            point![1.0, 0.0],
            point![2.0, 0.0],
            point![3.0, 0.0],
        ];
        let r = PathResampling::douglas_peucker(&p, 0.1);
        // 共线 → 中间点被移除
        assert_eq!(r.len(), 2);
    }

    #[test]
    fn douglas_peucker_keeps_corner() {
        let p = vec![
            point![0.0, 0.0],
            point![1.0, 0.0],
            point![1.0, 1.0],
            point![2.0, 1.0],
        ];
        let r = PathResampling::douglas_peucker(&p, 0.1);
        // 拐角 (1, 0) 必须保留
        assert!(r
            .iter()
            .any(|pt| (pt.x - 1.0).abs() < 1e-9 && (pt.y - 0.0).abs() < 1e-9));
    }

    #[test]
    fn resample_by_step_doubles() {
        let p = vec![point![0.0, 0.0], point![2.0, 0.0]];
        let r = PathResampling::resample_by_step(&p, 1.0);
        // 0, 1, 2 三个点
        assert_eq!(r.len(), 3);
        assert!((r[1].x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn total_length_diagonal() {
        let p = vec![point![0.0, 0.0], point![3.0, 4.0]];
        assert!((PathResampling::total_length(&p) - 5.0).abs() < 1e-9);
    }
}
