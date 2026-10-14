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
        struct GeometryConfig { self_collision: Option<SelfCollision> }
        #[derive(Deserialize)]
        struct SelfCollision {
            primitives: Vec<PrimitiveConfig>,
            #[serde(default)] model_base_link: String,
            #[serde(default)] excluded_moving_collision_count: usize,
            #[serde(default)] unsupported_collision_count: usize,
        }
        let Some(geometry) = geometry.filter(|value| !value.trim().is_empty()) else {
            return Ok(None);
        };
        let config: GeometryConfig = serde_json::from_str(geometry).map_err(|e| e.to_string())?;
        let Some(collision) = config.self_collision else { return Ok(None); };
        let mut primitives = Vec::with_capacity(collision.primitives.len());
        let mut lower = [f64::INFINITY; 3];
        let mut upper = [f64::NEG_INFINITY; 3];
        for item in collision.primitives {
            let dimensions_valid = match &item.shape {
                Shape::Box { size_m } => size_m.iter().all(|v| v.is_finite() && *v > 0.0),
                Shape::Sphere { radius_m } => radius_m.is_finite() && *radius_m > 0.0,
                Shape::Cylinder { radius_m, length_m } => radius_m.is_finite() && *radius_m > 0.0
                    && length_m.is_finite() && *length_m > 0.0,
            };
            let rotation = Matrix3::from_row_slice(&item.rotation_row_major);
            if !dimensions_valid || item.translation_m.iter().any(|v| !v.is_finite())
                || item.rotation_row_major.iter().any(|v| !v.is_finite())
                || (rotation.transpose() * rotation - Matrix3::identity()).norm() > 1e-8
                || (rotation.determinant() - 1.0).abs() > 1e-8 {
                return Err("固定自体碰撞模型需要有效尺寸及刚体变换".into());
            }
            let pose = Isometry3::from_parts(Translation3::new(item.translation_m[0], item.translation_m[1], item.translation_m[2]),
                UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)));
            // 启动时计算保守联合边界；多数外部点不必逐实体做逆变换。
            let matrix = pose.rotation.to_rotation_matrix();
            for axis in 0..3 {
                let extent = match &item.shape {
                    Shape::Box { size_m } => (0..3).map(|j| matrix[(axis,j)].abs() * size_m[j] / 2.0).sum::<f64>(),
                    Shape::Sphere { radius_m } => *radius_m,
                    Shape::Cylinder { radius_m, length_m } => radius_m * matrix[(axis,0)].hypot(matrix[(axis,1)])
                        + length_m / 2.0 * matrix[(axis,2)].abs(),
                };
                let center = pose.translation.vector[axis];
                // 仅吸收边界表达的浮点舍入，不膨胀实际实体分类。
                let rounding = 1e-12 * (1.0 + center.abs() + extent);
                lower[axis] = lower[axis].min(center - extent - rounding);
                upper[axis] = upper[axis].max(center + extent + rounding);
            }
            primitives.push(Primitive { shape_from_body: pose.inverse(), shape: item.shape });
        }
        Ok(if primitives.is_empty() { None } else { Some(Self { primitives, body_bounds: (lower, upper),
            model_base_link: collision.model_base_link,
            excluded_moving: collision.excluded_moving_collision_count,
            unsupported: collision.unsupported_collision_count,
        }) })
    }

    pub(crate) fn retain_external(&self, points: &mut Vec<Point3<f64>>, odom_from_body: &Isometry3<f64>) {
        let body_from_odom = odom_from_body.inverse();
        points.retain(|point| !self.contains_body(&(body_from_odom * point)));
    }

    fn contains_body(&self, body: &Point3<f64>) -> bool {
        // 非有限输入继续交给规范准入，不在过滤中静默吞掉。
        if body.coords.iter().any(|v| !v.is_finite()) { return false; }
        if (0..3).any(|axis| body[axis] < self.body_bounds.0[axis] || body[axis] > self.body_bounds.1[axis]) {
            return false;
        }
        self.primitives.iter().any(|primitive| {
            let p = primitive.shape_from_body * body;
            match &primitive.shape {
                Shape::Box { size_m } => (0..3).all(|axis| p[axis].abs() <= size_m[axis] / 2.0),
                Shape::Sphere { radius_m } => p.coords.norm_squared() <= radius_m * radius_m,
                Shape::Cylinder { radius_m, length_m } => p.z.abs() <= length_m / 2.0
                    && p.x * p.x + p.y * p.y <= radius_m * radius_m,
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
pub(crate) mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    pub(crate) fn filter() -> RobotSelfFilter {
        RobotSelfFilter::from_config(Some(r#"{"self_collision":{"primitives":[{"shape":"box","size_m":[0.58,0.328,0.357],"translation_m":[0.05,0,-0.1215],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}]}}"#)).unwrap().unwrap()
    }

    pub(crate) fn native_filter() -> NativeSelfFilter {
        NativeSelfFilter {
            filter: filter(),
            body_from_sensor: crate::native_lio_owner::BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.1,0.0,0.0],"rotation_row_major":[0,-1,0,1,0,0,0,0,1]}"#,
            )
            .unwrap(),
        }
    }

    #[test]
    fn self_cloud_fixed_shapes_preserve_gaps_and_low_external_points() {
        let filter = RobotSelfFilter::from_config(Some(r#"{"self_collision":{"primitives":[
          {"shape":"box","size_m":[0.4,0.2,0.2],"translation_m":[0,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]},
          {"shape":"sphere","radius_m":0.1,"translation_m":[1,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]},
          {"shape":"cylinder","radius_m":0.1,"length_m":0.4,"translation_m":[2,0,0],"rotation_row_major":[0,0,1,0,1,0,-1,0,0]}
        ]}}"#)).unwrap().unwrap();
        for point in [[0.0,0.0,0.0], [1.0,0.0,0.0], [2.15,0.0,0.0]] {
            assert!(filter.contains_body(&Point3::from(point)), "{point:?}");
        }
        for point in [[0.5,0.0,0.0], [0.0,0.0,-0.2], [1.08,0.08,0.0], [2.0,0.0,0.15], [2.21,0.0,0.0]] {
            assert!(!filter.contains_body(&Point3::from(point)), "{point:?}");
        }
        for (roll,pitch,yaw) in [(0.0,0.0,0.0),(0.2,-0.3,1.5)] {
            let pose = Isometry3::from_parts(Translation3::new(3.0,-2.0,0.7),
                UnitQuaternion::from_euler_angles(roll,pitch,yaw));
            let external = pose * Point3::new(0.5,0.0,-0.2);
            let mut points = vec![external, pose * Point3::new(0.0,0.0,0.0)];
            filter.retain_external(&mut points, &pose);
            assert_eq!(points, vec![external]);
        }
    }

    #[test]
    fn self_cloud_broad_phase_matches_exact_shapes_at_boundaries() {
        for angle in [0.0_f64, 0.3, 1.7] {
            let matrix = UnitQuaternion::from_euler_angles(angle, -angle / 2.0, angle / 3.0).to_rotation_matrix();
            let rotation: Vec<_> = (0..3).flat_map(|i| (0..3).map(move |j| matrix[(i,j)])).collect();
            for center in [[0.0,0.0,0.0], [10000.0,-20000.0,30000.0]] {
                for shape in [
                    serde_json::json!({"shape":"box", "size_m":[0.4,0.2,0.6]}),
                    serde_json::json!({"shape":"sphere", "radius_m":0.2}),
                    serde_json::json!({"shape":"cylinder", "radius_m":0.2, "length_m":0.6}),
                ] {
                    let mut primitive = shape;
                    primitive["translation_m"] = serde_json::json!(center);
                    primitive["rotation_row_major"] = serde_json::json!(rotation);
                    let json = serde_json::json!({"self_collision":{"primitives":[primitive]}}).to_string();
                    let mut filter = RobotSelfFilter::from_config(Some(&json)).unwrap().unwrap();
                    let bounds = filter.body_bounds;
                    let pose = filter.primitives[0].shape_from_body.inverse();
                    for x in [-0.400000000001,-0.4,-0.3,-0.200000000001,-0.2,-0.1,0.0,0.1,0.2,0.200000000001,0.3,0.4,0.400000000001] {
                        for y in [-0.3,-0.2,-0.1,0.0,0.1,0.2,0.3] {
                            for z in [-0.300000000001,-0.3,-0.2,0.0,0.2,0.3,0.300000000001] {
                                let p = pose * Point3::new(x,y,z);
                                filter.body_bounds = ([f64::NEG_INFINITY;3],[f64::INFINITY;3]);
                                let exact = filter.contains_body(&p);
                                filter.body_bounds = bounds;
                                assert_eq!(filter.contains_body(&p), exact, "{angle} {center:?} {p:?}");
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn self_cloud_empty_shapes_do_not_fall_back_to_navigation_envelope() {
        assert!(RobotSelfFilter::from_config(Some(r#"{"self_collision":{"primitives":[]},"footprint_xy_m":[[-1,-1],[1,-1],[1,1],[-1,1]],"z_min_m":-1,"z_max_m":1}"#)).unwrap().is_none());
    }

    #[test]
    fn self_cloud_raw_mask_preserves_layout_samples_and_last_point_time() {
        use crate::proto::{LioImuSample, PointCloudField, PublishPointCloudRequest};
        for points in [
            vec![[0.0_f32, 0.1, -0.425], [0.3, 0.1, -0.2], [0.0, 0.1, -0.2]],
            vec![[0.0, 0.1, -0.2]; 3],
        ] {
            let mut source = PublishPointCloudRequest {
                point_step: 20,
                point_count: 3,
                timestamp_ns: 123,
                frame_id: "sensor".into(),
                is_dense: points[0][2] < -0.3,
                imu_samples: vec![LioImuSample {
                    timestamp_ns: 119,
                    frame_id: "imu_frame".into(),
                    accel_mps2_x: 1.0,
                    accel_mps2_y: 2.0,
                    accel_mps2_z: 9.81,
                    gyro_rps_x: 0.1,
                    gyro_rps_y: 0.2,
                    gyro_rps_z: 0.3,
                    orientation_valid: true,
                    orientation_x: 0.0,
                    orientation_y: 0.0,
                    orientation_z: 0.0,
                    orientation_w: 1.0,
                }],
                fields: [
                    ("x", 0, 7),
                    ("y", 4, 7),
                    ("z", 8, 7),
                    ("offset_ns", 12, 6),
                    ("intensity", 16, 7),
                ]
                .into_iter()
                .map(|(name, offset, datatype)| PointCloudField {
                    name: name.into(),
                    offset,
                    datatype,
                    count: 1,
                })
                .collect(),
                ..Default::default()
            };
            for (index, xyz) in points.iter().enumerate() {
                for v in xyz {
                    source.points.extend_from_slice(&v.to_le_bytes());
                }
                source
                    .points
                    .extend_from_slice(&((index as u32 + 1) * 10).to_le_bytes());
                source.points.extend_from_slice(&99_f32.to_le_bytes());
            }
            let masked = native_filter().mask_request(&source).unwrap();
            let payload =
                crate::server::encode_pointcloud_request_payload_with_sequence(&masked, 7).unwrap();
            let mut wire = nuwax_codec::lcm::schema::POINT_CLOUD2_FINGERPRINT.to_vec();
            wire.extend(payload);
            let decoded = nuwax_codec::lcm::schema::decode_point_cloud2(&wire).unwrap();
            assert_eq!(
                (
                    decoded.width,
                    decoded.height,
                    decoded.point_step,
                    decoded.row_step
                ),
                (3, 1, 21, 63)
            );
            assert!(!decoded.is_bigendian);
            assert_eq!(masked.point_count, source.point_count);
            assert_eq!(masked.timestamp_ns, source.timestamp_ns);
            assert_eq!(masked.imu_samples, source.imu_samples);
            assert_eq!(masked.frame_id, source.frame_id);
            assert_eq!(masked.is_dense, source.is_dense);
            assert_eq!(masked.point_step, source.point_step + 1);
            assert_eq!(masked.fields.len(), source.fields.len() + 1);
            assert_eq!(&masked.fields[..source.fields.len()], source.fields.as_slice());
            assert_eq!(masked.fields.last().unwrap(), &PointCloudField {
                name: SELF_MASK_FIELD.into(), offset: source.point_step, datatype: 2, count: 1,
            });
            for (index, (before, after)) in source
                .points
                .chunks_exact(20)
                .zip(decoded.data.chunks_exact(21))
                .enumerate()
            {
                assert_eq!(before, &after[..20]);
                assert_eq!(after[20], u8::from(points[index] == [0.0, 0.1, -0.2]));
            }
            assert!(native_filter().mask_request(&masked).is_err());
        }
    }

    #[test]
    fn self_cloud_preserves_near_ground_and_external_obstacles_under_full_pose() {
        for (roll, pitch, yaw) in [(0.0, 0.0, 0.0), (0.12, -0.15, 1.7), (-0.25, 0.18, -2.4)] {
            let pose = Isometry3::from_parts(
                Translation3::new(2.0, -3.0, 0.7),
                UnitQuaternion::from_euler_angles(roll, pitch, yaw),
            );
            let support = pose * Point3::new(0.0, 0.0, -0.4);
            let ground: Vec<_> = [-0.075, -0.025, 0.025]
                .into_iter()
                .map(|dz| Point3::new(pose.translation.x, pose.translation.y, support.z + dz))
                .collect();
            let external: Vec<_> = [
                [0.0, -0.165, -0.2],
                [0.341, 0.0, -0.1],
                [0.0, 0.0, 0.058],
                [-0.241, 0.0, -0.2],
            ]
            .into_iter()
            .map(|p| pose * Point3::from(p))
            .collect();
            let mut points = ground.clone();
            points.extend_from_slice(&external);
            points.extend(
                [[0.0, 0.0, -0.175], [0.2, -0.1, -0.25], [-0.2, 0.1, 0.0]]
                    .into_iter()
                    .map(|p| pose * Point3::from(p)),
            );
            filter().retain_external(&mut points, &pose);
            assert_eq!(points, [ground, external].concat());
        }
    }

    #[test]
    fn self_cloud_keeps_invalid_points_and_does_not_invent_missing_shapes() {
        assert!(RobotSelfFilter::from_config(None)
            .unwrap()
            .is_none());
        assert!(RobotSelfFilter::from_config(Some("{}" )).unwrap().is_none());
        let mut points = vec![Point3::new(f64::NAN, 0.0, 0.0)];
        filter().retain_external(&mut points, &Isometry3::identity());
        assert_eq!(points.len(), 1);
    }
}
