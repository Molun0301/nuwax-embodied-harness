//! 真实固定刚体与点、闭体素的有限相交查询。
//!
//! 本模块不读取地面身份、能力、支撑误差或 whole-body 包络，不决定接触豁免。
//! 圆柱沿实体局部 Z 轴，输入旋转与 URDF 导出的 row-major 顺序一致。
use super::{SurfaceError, cross, dot, rotation, world};

#[path = "body_contact_motion.rs"]
mod motion;
pub(in crate::surface) use motion::RigidMotionQuery;

const ROUNDING_FACTOR: f64 = 128.0 * f64::EPSILON;
const ABSOLUTE_TOLERANCE_M: f64 = 1e-10;
const WORLD_AXES: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// 全尺寸与真实半径；圆柱长度包含两端闭平面。
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::surface) enum RigidShape {
    Box { size_m: [f64; 3] },
    Sphere { radius_m: f64 },
    Cylinder { radius_m: f64, length_m: f64 },
}

#[derive(Clone, Debug)]
pub(in crate::surface) struct RigidPrimitive {
    shape: RigidShape,
    translation: [f64; 3],
    // 本地实体到机身的旋转列。
    axes: [[f64; 3]; 3],
    extent_scale: f64,
}

impl RigidPrimitive {
    pub(in crate::surface) fn new(
        shape: RigidShape,
        translation_m: [f64; 3],
        rotation_row_major: [f64; 9],
    ) -> Result<Self, SurfaceError> {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        // 仅作为舍入误差尺度，不计算可能溢出的外接球半径，也不限制合法尺寸。
        let extent_scale = match shape {
            RigidShape::Box { size_m } if size_m.into_iter().all(positive) => max_abs(size_m) * 0.5,
            RigidShape::Sphere { radius_m } if positive(radius_m) => radius_m,
            RigidShape::Cylinder { radius_m, length_m }
                if positive(radius_m) && positive(length_m) =>
            {
                radius_m.max(length_m * 0.5)
            }
            _ => return Err(invalid_geometry()),
        };
        if !translation_m.into_iter().all(f64::is_finite)
            || !rotation_row_major.into_iter().all(f64::is_finite)
        {
            return Err(invalid_geometry());
        }
        let axes = std::array::from_fn(|column| {
            std::array::from_fn(|row| rotation_row_major[row * 3 + column])
        });
        // 只接受右手正交旋转；不把缩放、反射或剪切当作位姿。
        for (a, axis_a) in axes.iter().enumerate() {
            for (b, axis_b) in axes.iter().enumerate() {
                let expected = if a == b { 1.0 } else { 0.0 };
                let residual = dot(*axis_a, *axis_b) - expected;
                if !residual.is_finite() || residual.abs() > ROUNDING_FACTOR {
                    return Err(invalid_geometry());
                }
            }
        }
        if (dot(cross(axes[0], axes[1]), axes[2]) - 1.0).abs() > ROUNDING_FACTOR {
            return Err(invalid_geometry());
        }
        Ok(Self {
            shape,
            translation: translation_m,
            axes,
            extent_scale,
        })
    }
}

// 候选边界点尺度可达 center+extent；解 tol <= ABS+ROUND*(2*scale+tol)。
// 先乘微小 ROUND 再乘 scale，避免 2*scale 对合法有限输入溢出。
fn candidate_allowance(scale: f64) -> f64 {
    ((ABSOLUTE_TOLERANCE_M + (2.0 * ROUNDING_FACTOR) * scale) / (1.0 - ROUNDING_FACTOR)).next_up()
}

fn invalid_geometry() -> SurfaceError {
    SurfaceError::InvalidConfig("invalid fixed rigid primitive geometry".into())
}

#[derive(Clone, Debug)]
struct PreparedPrimitive {
    shape: RigidShape,
    center: [f64; 3],
    axes: [[f64; 3]; 3],
    scale: f64,
}

/// 同一真实机身位姿复用的实体查询；无全局状态、地面成员副本或身份缓存。
pub(in crate::surface) struct RigidBodyQuery {
    primitives: Vec<PreparedPrimitive>,
}

impl RigidBodyQuery {
    /// 候选范围只取真实实体的世界 AABB，不假定调用方外包络已包含实体。
    pub(in crate::surface) fn bounds(&self) -> Result<Option<([f64; 3], [f64; 3])>, SurfaceError> {
        let mut result: Option<([f64; 3], [f64; 3])> = None;
        for primitive in &self.primitives {
            let extent: [f64; 3] = std::array::from_fn(|axis| match primitive.shape {
                RigidShape::Box { size_m } => (0..3)
                    .map(|i| primitive.axes[i][axis].abs() * size_m[i] * 0.5)
                    .sum(),
                RigidShape::Sphere { radius_m } => radius_m,
                RigidShape::Cylinder { radius_m, length_m } => {
                    primitive.axes[0][axis].hypot(primitive.axes[1][axis]) * radius_m
                        + primitive.axes[2][axis].abs() * length_m * 0.5
                }
            });
            let allowance = candidate_allowance(primitive.scale.max(max_abs(primitive.center)).max(max_abs(extent)));
            let low = std::array::from_fn(|axis| {
                (primitive.center[axis] - extent[axis] - allowance).next_down()
            });
            let high = std::array::from_fn(|axis| {
                (primitive.center[axis] + extent[axis] + allowance).next_up()
            });
            finite(low)?;
            finite(high)?;
            result = Some(match result {
                None => (low, high),
                Some((a, b)) => (
                    std::array::from_fn(|i| a[i].min(low[i])),
                    std::array::from_fn(|i| b[i].max(high[i])),
                ),
            });
        }
        Ok(result)
    }
    pub(in crate::surface) fn new(
        primitives: &[RigidPrimitive],
        body_pose: ([f64; 3], [f64; 4]),
    ) -> Result<Self, SurfaceError> {
        finite(body_pose.0)?;
        let body_axes = rotation(body_pose.1)?;
        let primitives = primitives
            .iter()
            .map(|primitive| {
                let center = world(body_axes, body_pose.0, primitive.translation);
                finite(center)?;
                let axes = primitive.axes.map(|axis| world(body_axes, [0.0; 3], axis));
                // 合成过程与后续世界坐标相减只加浮点误差，不引入能力/体素尺寸余量。
                let scale = max_abs(center)
                    .max(max_abs(body_pose.0))
                    .max(max_abs(primitive.translation))
                    .max(primitive.extent_scale);
                Ok(PreparedPrimitive {
                    shape: primitive.shape,
                    center,
                    axes,
                    scale,
                })
            })
            .collect::<Result<Vec<_>, SurfaceError>>()?;
        Ok(Self { primitives })
    }

    pub(in crate::surface) fn intersects_point(
        &self,
        point: [f64; 3],
    ) -> Result<bool, SurfaceError> {
        finite(point)?;
        for primitive in &self.primitives {
            let local = primitive.to_local(point)?;
            let tolerance = primitive.tolerance(max_abs(point));
            let intersects = match primitive.shape {
                RigidShape::Box { size_m } => {
                    (0..3).all(|axis| local[axis].abs() <= size_m[axis] * 0.5 + tolerance)
                }
                RigidShape::Sphere { radius_m } => within_radius(local, radius_m, tolerance),
                RigidShape::Cylinder { radius_m, length_m } => {
                    local[2].abs() <= length_m * 0.5 + tolerance
                        && within_radius([local[0], local[1], 0.0], radius_m, tolerance)
                }
            };
            if intersects {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// min/max 为闭世界 AABB；允许零宽体素，相切即相交。
    pub(in crate::surface) fn intersects_voxel(
        &self,
        min: [f64; 3],
        max: [f64; 3],
    ) -> Result<bool, SurfaceError> {
        finite(min)?;
        finite(max)?;
        if (0..3).any(|axis| min[axis] > max[axis]) {
            return Err(SurfaceError::InvalidPose);
        }
        for index in 0..self.primitives.len() {
            if self.primitive_intersects_voxel(index, min, max)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn primitive_intersects_voxel(
        &self,
        index: usize,
        min: [f64; 3],
        max: [f64; 3],
    ) -> Result<bool, SurfaceError> {
        finite(min)?;
        finite(max)?;
        let half: [f64; 3] = std::array::from_fn(|axis| max[axis] * 0.5 - min[axis] * 0.5);
        let center = std::array::from_fn(|axis| max[axis] * 0.5 + min[axis] * 0.5);
        let scale = max_abs(min).max(max_abs(max));
        let primitive = &self.primitives[index];
        let tolerance = primitive.tolerance(scale);
        let intersects = match primitive.shape {
            RigidShape::Box { size_m } => {
                primitive.box_voxel(size_m.map(|v| v * 0.5), center, half, tolerance)?
            }
            RigidShape::Sphere { radius_m } => {
                let nearest =
                    std::array::from_fn(|axis| primitive.center[axis].clamp(min[axis], max[axis]));
                let delta = difference(nearest, primitive.center)?;
                within_radius(delta, radius_m, tolerance)
            }
            RigidShape::Cylinder { radius_m, length_m } => {
                primitive.cylinder_voxel(radius_m, length_m * 0.5, min, max, tolerance)?
            }
        };
        Ok(intersects)
    }
}

impl PreparedPrimitive {
    fn tolerance(&self, query_scale: f64) -> f64 {
        ABSOLUTE_TOLERANCE_M + ROUNDING_FACTOR * self.scale.max(query_scale)
    }

    fn to_local(&self, point: [f64; 3]) -> Result<[f64; 3], SurfaceError> {
        let delta = difference(point, self.center)?;
        let local = self.axes.map(|axis| dot(axis, delta));
        finite(local)?;
        Ok(local)
    }

    fn box_voxel(
        &self,
        body_half: [f64; 3],
        voxel_center: [f64; 3],
        voxel_half: [f64; 3],
        tolerance: f64,
    ) -> Result<bool, SurfaceError> {
        let delta = difference(voxel_center, self.center)?;
        let axes = self.axes.into_iter().chain(WORLD_AXES).chain(
            self.axes
                .into_iter()
                .flat_map(|body| WORLD_AXES.map(|voxel| cross(body, voxel))),
        );
        for axis in axes {
            let length = norm(axis);
            // 平行轴叉积为零，没有分离信息；近零轴仍正规化计算。
            if length == 0.0 {
                continue;
            }
            let axis = axis.map(|value| value / length);
            let body_radius: f64 = (0..3)
                .map(|i| dot(axis, self.axes[i]).abs() * body_half[i])
                .sum();
            let voxel_radius: f64 = (0..3).map(|i| axis[i].abs() * voxel_half[i]).sum();
            let separation = dot(axis, delta).abs();
            if !body_radius.is_finite() || !voxel_radius.is_finite() || !separation.is_finite() {
                return Err(SurfaceError::InvalidPose);
            }
            if separation > body_radius + voxel_radius + tolerance {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn cylinder_voxel(
        &self,
        radius: f64,
        half_length: f64,
        min: [f64; 3],
        max: [f64; 3],
        tolerance: f64,
    ) -> Result<bool, SurfaceError> {
        let mut corners = [[0.0; 3]; 8];
        for (index, corner) in corners.iter_mut().enumerate() {
            *corner = self.to_local(std::array::from_fn(|axis| {
                if index & (1 << axis) == 0 {
                    min[axis]
                } else {
                    max[axis]
                }
            }))?;
        }
        // 先把凸体裁至圆柱两个端面之间，再投影 XY。
        // 截取后的所有顶点来自 8 原顶点或 12 原边与 2 端面的交点，最多 32 个。
        // 若投影凸包与圆盘相交，对应凸组合同时满足轴向范围，故是真实圆柱相交。
        let cap = half_length + tolerance;
        let mut projected = Vec::with_capacity(32);
        for corner in corners {
            if corner[2].abs() <= cap {
                projected.push([corner[0], corner[1]]);
            }
        }
        for index in 0..8 {
            for axis in 0..3 {
                if index & (1 << axis) != 0 {
                    continue;
                }
                let a = corners[index];
                let b = corners[index | (1 << axis)];
                for z in [-cap, cap] {
                    if (a[2] < z && b[2] > z) || (b[2] < z && a[2] > z) {
                        // 同时缩放分子分母，避免大世界坐标差在相减时溢出。
                        let scale = a[2].abs().max(b[2].abs()).max(z.abs());
                        let t = (z / scale - a[2] / scale) / (b[2] / scale - a[2] / scale);
                        projected.push([a[0] * (1.0 - t) + b[0] * t, a[1] * (1.0 - t) + b[1] * t]);
                    }
                }
            }
        }
        Ok(projected_disk_intersection(projected, radius, tolerance))
    }
}

fn finite(value: [f64; 3]) -> Result<(), SurfaceError> {
    if value.into_iter().all(f64::is_finite) {
        Ok(())
    } else {
        Err(SurfaceError::InvalidPose)
    }
}

fn difference(a: [f64; 3], b: [f64; 3]) -> Result<[f64; 3], SurfaceError> {
    let value = std::array::from_fn(|axis| a[axis] - b[axis]);
    finite(value)?;
    Ok(value)
}

fn max_abs(value: [f64; 3]) -> f64 {
    value.into_iter().map(f64::abs).fold(0.0, f64::max)
}
fn norm(value: [f64; 3]) -> f64 {
    value[0].hypot(value[1]).hypot(value[2])
}

fn within_radius(delta: [f64; 3], radius: f64, tolerance: f64) -> bool {
    let scale = max_abs(delta).max(radius).max(tolerance);
    // 分别归一化再相加；合法有限 radius 接近 MAX 时 radius + tol 会溢出。
    norm(delta.map(|value| value / scale)) <= radius / scale + tolerance / scale
}

fn projected_disk_intersection(mut points: Vec<[f64; 2]>, radius: f64, tolerance: f64) -> bool {
    if points.is_empty() {
        return false;
    }
    // 统一缩放防止凸包叉积和线段距离的中间乘积溢出；几何判定不变。
    let scale = points
        .iter()
        .flat_map(|p| p.iter())
        .map(|v| v.abs())
        .fold(radius.max(tolerance), f64::max);
    for point in &mut points {
        *point = point.map(|v| v / scale);
    }
    let radius = radius / scale + tolerance / scale;
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then_with(|| a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() == 1 {
        return points[0][0].hypot(points[0][1]) <= radius;
    }
    let polygon = if points.len() == 2 {
        points
    } else {
        super::hull(points)
    };
    let mut inside = polygon.len() >= 3;
    for (a, b) in polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
    {
        let edge = [b[0] - a[0], b[1] - a[1]];
        let length = edge[0].hypot(edge[1]);
        if length == 0.0 {
            continue;
        }
        let unit = edge.map(|v| v / length);
        let along = (-a[0] * unit[0] - a[1] * unit[1]).clamp(0.0, length);
        let nearest = [a[0] + along * unit[0], a[1] + along * unit[1]];
        if nearest[0].hypot(nearest[1]) <= radius {
            return true;
        }
        if unit[0] * -a[1] - unit[1] * -a[0] < 0.0 {
            inside = false;
        }
    }
    inside
}

#[cfg(test)]
#[path = "body_contact_tests.rs"]
mod tests;
