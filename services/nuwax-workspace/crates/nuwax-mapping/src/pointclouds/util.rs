//! 点云工具

use nalgebra::Point3;

/// 计算点云包围盒
#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    /// 最小点
    pub min: Point3<f64>,
    /// 最大点
    pub max: Point3<f64>,
}

impl BoundingBox {
    /// 从点集计算 AABB
    #[must_use]
    pub fn from_points(points: &[Point3<f64>]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }
        let mut min = points[0];
        let mut max = points[0];
        for p in &points[1..] {
            if p.x < min.x {
                min.x = p.x;
            }
            if p.y < min.y {
                min.y = p.y;
            }
            if p.z < min.z {
                min.z = p.z;
            }
            if p.x > max.x {
                max.x = p.x;
            }
            if p.y > max.y {
                max.y = p.y;
            }
            if p.z > max.z {
                max.z = p.z;
            }
        }
        Some(Self { min, max })
    }

    /// 尺寸（max - min）
    #[must_use]
    pub fn size(&self) -> nalgebra::Vector3<f64> {
        self.max - self.min
    }

    /// 中心点
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::from((self.min.coords + self.max.coords) * 0.5)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_three_points() {
        let pts = vec![
            nalgebra::point![0.0, 0.0, 0.0],
            nalgebra::point![3.0, 4.0, 1.0],
            nalgebra::point![-1.0, 2.0, 5.0],
        ];
        let bb = BoundingBox::from_points(&pts).unwrap();
        assert_eq!(bb.min.x, -1.0);
        assert_eq!(bb.min.y, 0.0);
        assert_eq!(bb.min.z, 0.0);
        assert_eq!(bb.max.x, 3.0);
        assert_eq!(bb.max.y, 4.0);
        assert_eq!(bb.max.z, 5.0);
    }

    #[test]
    fn aabb_empty() {
        let pts: Vec<Point3<f64>> = vec![];
        assert!(BoundingBox::from_points(&pts).is_none());
    }

    #[test]
    fn aabb_size_and_center() {
        let pts = vec![
            nalgebra::point![0.0, 0.0, 0.0],
            nalgebra::point![2.0, 2.0, 2.0],
        ];
        let bb = BoundingBox::from_points(&pts).unwrap();
        assert_eq!(bb.size(), nalgebra::Vector3::new(2.0, 2.0, 2.0));
        assert_eq!(bb.center(), nalgebra::point![1.0, 1.0, 1.0]);
    }
}
