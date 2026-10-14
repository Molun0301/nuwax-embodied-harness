//! 点与点云基础类型
//!
//! SSOT：`/home/ml/Deeprob/dimos/dimos/msgs/sensor_msgs/PointCloud2.py`
//! + `services/nuwax-workspace/lcmtypes/sensor_msgs/PointCloud2.lcm`
//!
//! 修复要点（见 docs/.../20_rework20_全仓审计_对dimos原始.md §2.1 H-3）：
//! `PointCloud2Like` 之前缺 intensities/colors 通道；本次补齐默认实现
//! 返回 `None` / `0` / `false`，由实现方按需 override。

use nalgebra::{Point2 as Np2, Point3 as Np3};
use serde::{Deserialize, Serialize};

/// 2D 点
pub type Point2 = Np2<f64>;

/// 3D 点
pub type Point3 = Np3<f64>;

/// PointCloud2 简化接口(下游 nuwax-mapping / nuwax-slam 等共用)
///
/// 字段对齐 `sensor_msgs/PointCloud2.lcm` wire format:
/// stamp_sec, stamp_nsec, frame_id, height, width, fields, is_bigendian,
/// point_step, row_step, data, is_dense。
///
/// 暴露 xyz 简化视图 + 标准 wire 字段;`intensities`/`colors` 等通道由实现方
/// 按 `point_step` + `fields` 在 `data` 缓冲上投影(默认 `None`,表示无该通道)。
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

    /// 强度通道(per-point f32),可选;默认 `None`
    #[must_use]
    fn intensities(&self) -> Option<&[f32]> {
        None
    }

    /// 颜色通道(RGB 或 RGBA 字节),可选;默认 `None`
    #[must_use]
    fn colors(&self) -> Option<&[u8]> {
        None
    }

    /// 坐标系名(LCM `string frame_id`)
    fn frame_id(&self) -> &str;

    /// 点云宽度(LCM `int32_t width`,unorganized 时等于点数)
    #[must_use]
    fn width(&self) -> u32;

    /// 点云高度(LCM `int32_t height`,unorganized 时为 1)
    #[must_use]
    fn height(&self) -> u32;

    /// 单点字节步长(LCM `int32_t point_step`)
    #[must_use]
    fn point_step(&self) -> u32;

    /// 一行字节步长(LCM `int32_t row_step`)
    #[must_use]
    fn row_step(&self) -> u32;

    /// 是否大端字节序(LCM `boolean is_bigendian`)
    #[must_use]
    fn is_bigendian(&self) -> bool;

    /// 是否稠密(LCM `boolean is_dense`,`false` 表示含 NaN/Inf)
    #[must_use]
    fn is_dense(&self) -> bool;
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
