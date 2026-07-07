//! 距离计算
//! 基于 kiddo KD-tree 实现：
//! - 欧几里得距离（默认）
//! - 曼哈顿距离
//! - 切比雪夫距离
//! - 最近邻查询
//! - 成对距离矩阵

use kiddo::KdTree;
use nalgebra::Point3;

/// 距离度量类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DistanceMetric {
    /// L2 欧几里得
    Euclidean,
    /// L1 曼哈顿
    Manhattan,
    /// L∞ 切比雪夫
    Chebyshev,
}

impl DistanceMetric {
    /// 计算两点距离
    #[must_use]
    pub fn distance(&self, a: &Point3<f64>, b: &Point3<f64>) -> f64 {
        match self {
            Self::Euclidean => {
                ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
            }
            Self::Manhattan => (a.x - b.x).abs() + (a.y - b.y).abs() + (a.z - b.z).abs(),
            Self::Chebyshev => {
                let dx = (a.x - b.x).abs();
                let dy = (a.y - b.y).abs();
                let dz = (a.z - b.z).abs();
                dx.max(dy).max(dz)
            }
        }
    }
}

/// 最近邻查询（kiddo KD-tree）
///
/// 从 query 中找出 points 中最近的点索引与距离。
#[must_use]
pub fn nearest_neighbor(points: &[Point3<f64>], query: &Point3<f64>) -> Option<(usize, f64)> {
    if points.is_empty() {
        return None;
    }
    let mut tree: KdTree<f64, 3> = KdTree::new();
    for (i, p) in points.iter().enumerate() {
        tree.add(&[p.x, p.y, p.z], i as u64);
    }
    let nn = tree.nearest_one::<kiddo::SquaredEuclidean>(&[query.x, query.y, query.z]);
    Some((nn.item as usize, nn.distance.sqrt()))
}

/// 成对距离矩阵（n × n）
#[must_use]
pub fn pairwise_distance(points: &[Point3<f64>], metric: DistanceMetric) -> Vec<Vec<f64>> {
    let n = points.len();
    let mut out = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in (i + 1)..n {
            let d = metric.distance(&points[i], &points[j]);
            out[i][j] = d;
            out[j][i] = d;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::point;

    #[test]
    fn metric_euclidean() {
        let a = point![0.0, 0.0, 0.0];
        let b = point![3.0, 4.0, 0.0];
        let d = DistanceMetric::Euclidean.distance(&a, &b);
        assert!((d - 5.0).abs() < 1e-9);
    }

    #[test]
    fn metric_manhattan() {
        let a = point![0.0, 0.0, 0.0];
        let b = point![3.0, 4.0, 0.0];
        let d = DistanceMetric::Manhattan.distance(&a, &b);
        assert!((d - 7.0).abs() < 1e-9);
    }

    #[test]
    fn metric_chebyshev() {
        let a = point![0.0, 0.0, 0.0];
        let b = point![3.0, 4.0, 1.0];
        let d = DistanceMetric::Chebyshev.distance(&a, &b);
        assert!((d - 4.0).abs() < 1e-9);
    }

    #[test]
    fn nearest_neighbor_basic() {
        let pts = vec![
            point![0.0, 0.0, 0.0],
            point![1.0, 0.0, 0.0],
            point![10.0, 10.0, 10.0],
        ];
        let query = point![1.1, 0.0, 0.0];
        let (idx, dist) = nearest_neighbor(&pts, &query).unwrap();
        assert_eq!(idx, 1);
        assert!((dist - 0.1).abs() < 1e-6);
    }

    #[test]
    fn nearest_neighbor_empty() {
        let pts: Vec<Point3<f64>> = vec![];
        let query = point![0.0, 0.0, 0.0];
        assert!(nearest_neighbor(&pts, &query).is_none());
    }

    #[test]
    fn pairwise_distance_symmetric() {
        let pts = vec![
            point![0.0, 0.0, 0.0],
            point![1.0, 0.0, 0.0],
            point![0.0, 1.0, 0.0],
        ];
        let m = pairwise_distance(&pts, DistanceMetric::Euclidean);
        assert_eq!(m.len(), 3);
        assert_eq!(m[0].len(), 3);
        assert!((m[0][1] - m[1][0]).abs() < 1e-9);
        assert!((m[0][0] - 0.0).abs() < 1e-9);
        assert!((m[0][1] - 1.0).abs() < 1e-9);
        assert!((m[0][2] - 1.0).abs() < 1e-9);
        assert!((m[1][2] - 2.0_f64.sqrt()).abs() < 1e-9);
    }
}
