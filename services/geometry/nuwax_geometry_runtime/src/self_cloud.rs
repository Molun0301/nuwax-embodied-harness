//! 规范注册之前只删除与机体刚性相连的碰撞实体，不把导航凸包当实心机器人。

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion};
use serde::Deserialize;

pub(crate) const SELF_MASK_FIELD: &str = "nuwax_self_mask";

#[cfg(test)]
#[path = "self_cloud_python_tests.rs"]
mod python_geometry_tests;

#[derive(Deserialize)]
#[serde(tag = "shape", rename_all = "lowercase")]
enum Shape {
    Box { size_m: [f64; 3] },
    Sphere { radius_m: f64 },
    Cylinder { radius_m: f64, length_m: f64 },
}

#[derive(Deserialize)]
struct PrimitiveConfig {
    translation_m: [f64; 3],
    rotation_row_major: [f64; 9],
    #[serde(flatten)]
    shape: Shape,
}

struct Primitive {
    shape_from_body: Isometry3<f64>,
    shape: Shape,
}

pub(crate) struct RobotSelfFilter {
    primitives: Vec<Primitive>,
    body_bounds: ([f64; 3], [f64; 3]),
    model_base_link: String,
    excluded_moving: usize,
    unsupported: usize,
}

impl RobotSelfFilter {
    pub(crate) fn from_env(runtime_body_frame: &str) -> Result<Option<Self>, String> {
        let geometry = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").ok();
        let result = Self::from_config(geometry.as_deref())?;
        if let Some(filter) = &result {
            // 配置已声明选定模型连杆是机体基准；名称绑定不是物理标定验证。
            eprintln!("固定自体过滤：模型基准 {} 按几何配置对应运行时 body {}，实体 {}，排除活动几何 {}，未支持 {}；动态腿足未覆盖",
                filter.model_base_link, runtime_body_frame, filter.primitives.len(), filter.excluded_moving, filter.unsupported);
        } else {
            eprintln!("固定自体过滤未启用：配置没有可用固定碰撞实体；不使用导航外包替代");
        }
        Ok(result)
    }

    pub(crate) fn from_config(geometry: Option<&str>) -> Result<Option<Self>, String> {
        #[derive(Deserialize)]
        struct GeometryConfig {
            self_collision: Option<SelfCollision>,
        }
        #[derive(Deserialize)]
        struct SelfCollision {
            primitives: Vec<PrimitiveConfig>,
            #[serde(default)]
            model_base_link: String,
            #[serde(default)]
            excluded_moving_collision_count: usize,
            #[serde(default)]
            unsupported_collision_count: usize,
        }
        let Some(geometry) = geometry.filter(|value| !value.trim().is_empty()) else {
            return Ok(None);
        };
        let config: GeometryConfig = serde_json::from_str(geometry).map_err(|e| e.to_string())?;
        let Some(collision) = config.self_collision else {
            return Ok(None);
        };
        let mut primitives = Vec::with_capacity(collision.primitives.len());
        let mut lower = [f64::INFINITY; 3];
        let mut upper = [f64::NEG_INFINITY; 3];
        for item in collision.primitives {
            let dimensions_valid = match &item.shape {
                Shape::Box { size_m } => size_m.iter().all(|v| v.is_finite() && *v > 0.0),
                Shape::Sphere { radius_m } => radius_m.is_finite() && *radius_m > 0.0,
                Shape::Cylinder { radius_m, length_m } => {
                    radius_m.is_finite()
                        && *radius_m > 0.0
                        && length_m.is_finite()
                        && *length_m > 0.0
                }
            };
            let rotation = Matrix3::from_row_slice(&item.rotation_row_major);
            if !dimensions_valid
                || item.translation_m.iter().any(|v| !v.is_finite())
                || item.rotation_row_major.iter().any(|v| !v.is_finite())
                || (rotation.transpose() * rotation - Matrix3::identity()).norm() > 1e-8
                || (rotation.determinant() - 1.0).abs() > 1e-8
            {
                return Err("固定自体碰撞模型需要有效尺寸及刚体变换".into());
            }
            let pose = Isometry3::from_parts(
                Translation3::new(
                    item.translation_m[0],
                    item.translation_m[1],
                    item.translation_m[2],
                ),
                UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
            );
            // 启动时计算保守联合边界；多数外部点不必逐实体做逆变换。
            let matrix = pose.rotation.to_rotation_matrix();
            for axis in 0..3 {
                let extent = match &item.shape {
                    Shape::Box { size_m } => (0..3)
                        .map(|j| matrix[(axis, j)].abs() * size_m[j] / 2.0)
                        .sum::<f64>(),
                    Shape::Sphere { radius_m } => *radius_m,
                    Shape::Cylinder { radius_m, length_m } => {
                        radius_m * matrix[(axis, 0)].hypot(matrix[(axis, 1)])
                            + length_m / 2.0 * matrix[(axis, 2)].abs()
                    }
                };
                let center = pose.translation.vector[axis];
                // 仅吸收边界表达的浮点舍入，不膨胀实际实体分类。
                let rounding = 1e-12 * (1.0 + center.abs() + extent);
                lower[axis] = lower[axis].min(center - extent - rounding);
                upper[axis] = upper[axis].max(center + extent + rounding);
            }
            primitives.push(Primitive {
                shape_from_body: pose.inverse(),
                shape: item.shape,
            });
        }
        Ok(if primitives.is_empty() {
            None
        } else {
            Some(Self {
                primitives,
                body_bounds: (lower, upper),
                model_base_link: collision.model_base_link,
                excluded_moving: collision.excluded_moving_collision_count,
                unsupported: collision.unsupported_collision_count,
            })
        })
    }

    pub(crate) fn retain_external(
        &self,
        points: &mut Vec<Point3<f64>>,
        odom_from_body: &Isometry3<f64>,
    ) {
        let body_from_odom = odom_from_body.inverse();
        points.retain(|point| !self.contains_body(&(body_from_odom * point)));
    }

    pub(crate) fn contains_body(&self, body: &Point3<f64>) -> bool {
        // 非有限输入继续交给规范准入，不在过滤中静默吞掉。
        if body.coords.iter().any(|v| !v.is_finite()) {
            return false;
        }
        if (0..3).any(|axis| {
            body[axis] < self.body_bounds.0[axis] || body[axis] > self.body_bounds.1[axis]
        }) {
            return false;
        }
        self.primitives.iter().any(|primitive| {
            let p = primitive.shape_from_body * body;
            match &primitive.shape {
                Shape::Box { size_m } => (0..3).all(|axis| p[axis].abs() <= size_m[axis] / 2.0),
                Shape::Sphere { radius_m } => p.coords.norm_squared() <= radius_m * radius_m,
                Shape::Cylinder { radius_m, length_m } => {
                    p.z.abs() <= length_m / 2.0 && p.x * p.x + p.y * p.y <= radius_m * radius_m
                }
            }
        })
    }
}

/// 原始输入仅附加内部无效标记，保留所有采样和惯性时间窗。
pub(crate) struct NativeSelfFilter {
    filter: RobotSelfFilter,
    body_from_sensor: crate::native_lio_owner::BodyFromSensorTransform,
}

impl NativeSelfFilter {
    pub(crate) fn from_env(runtime_body_frame: &str) -> Result<Option<Self>, String> {
        let Some(filter) = RobotSelfFilter::from_env(runtime_body_frame)? else {
            return Ok(None);
        };
        Ok(Some(Self {
            filter,
            body_from_sensor: crate::native_lio_owner::BodyFromSensorTransform::from_env()?,
        }))
    }

    pub(crate) fn mask_request(
        &self,
        source: &crate::proto::PublishPointCloudRequest,
    ) -> Result<crate::proto::PublishPointCloudRequest, String> {
        let step = source.point_step as usize;
        if step < 12
            || source.points.len() % step != 0
            || source
                .fields
                .iter()
                .any(|field| field.name == SELF_MASK_FIELD)
        {
            return Err("原始自体筛选要求完整点布局且无重复内部标记".into());
        }
        let next_step = source
            .point_step
            .checked_add(1)
            .ok_or("自体标记点步长溢出")?;
        let capacity = (source.points.len() / step)
            .checked_mul(next_step as usize)
            .ok_or("自体标记点云大小溢出")?;
        // 只复制仍需拥有的小字段；原始点缓冲直接读入新布局，不能先整帧clone再丢弃。
        let mut request = crate::proto::PublishPointCloudRequest {
            points: Vec::with_capacity(capacity),
            point_step: source.point_step,
            frame_id: source.frame_id.clone(),
            timestamp_ns: source.timestamp_ns,
            fields: source.fields.clone(),
            point_count: source.point_count,
            is_dense: source.is_dense,
            imu_samples: source.imu_samples.clone(),
            mapping_only: source.mapping_only,
            sensor_origin_xyz: source.sensor_origin_xyz.clone(),
            sensor_id: source.sensor_id.clone(),
            source_sequence: source.source_sequence,
            clock_id: source.clock_id.clone(),
            calibration_ref: source.calibration_ref.clone(),
        };
        for point in source.points.chunks_exact(step) {
            let xyz = [0, 4, 8].map(|offset| {
                f32::from_le_bytes(point[offset..offset + 4].try_into().unwrap()) as f64
            });
            let body = self.body_from_sensor.transform_point(&Point3::from(xyz));
            let is_self = self.filter.contains_body(&body);
            request.points.extend_from_slice(point);
            request.points.push(u8::from(is_self));
        }
        request.fields.push(crate::proto::PointCloudField {
            name: SELF_MASK_FIELD.into(),
            offset: source.point_step,
            datatype: 2,
            count: 1,
        });
        request.point_step = next_step;
        Ok(request)
    }
}

#[cfg(test)]
#[path = "self_cloud_tests.rs"]
pub(crate) mod tests;
