//! 点与点云基础类型

use nalgebra::{Point2 as Np2, Point3 as Np3};
use serde::{Deserialize, Serialize};

/// 2D 点
pub type Point2 = Np2<f64>;

/// 3D 点
pub type Point3 = Np3<f64>;

/// PointCloud2 简化接口（下游 nuwax-mapping / nuwax-slam 等共用）
pub trait PointCloud2Like {
    /// 所有点的引用
    fn points(&self) -> &[Point3];

    /// 点的数量
    #[must_use]
    fn len(&self) -> usize {
        self.points().len()
    }

    /// 是否为空
    #[must_use]
    fn is_empty(&self) -> bool {
        self.points().is_empty()
    }
}

/// PointField
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PointFieldKind {
    /// 8 位无符号
    Uint8,
    /// 8 位有符号
    Int8,
    /// 16 位无符号
    Uint16,
    /// 16 位有符号
    Int16,
    /// 32 位无符号
    Uint32,
    /// 32 位有符号
    Int32,
    /// 32 位浮点
    Float32,
    /// 64 位浮点
    Float64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point2_construction() {
        let p: Point2 = nalgebra::point![1.0, 2.0];
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
    }

    #[test]
    fn point3_construction() {
        let p: Point3 = nalgebra::point![1.0, 2.0, 3.0];
        assert_eq!(p.z, 3.0);
    }
}
