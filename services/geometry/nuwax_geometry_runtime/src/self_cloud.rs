//! 在规范注册之前，按同帧机体外包剔除自体回波并保留支撑带。

use nalgebra::{Isometry3, Point3};
use nuwax_nav_types_messages::RobotGeometry;

pub(crate) const SELF_MASK_FIELD: &str = "nuwax_self_mask";

pub(crate) struct RobotSelfFilter {
    geometry: RobotGeometry,
    body_to_support_z_m: f64,
}

impl RobotSelfFilter {
    pub(crate) fn from_env() -> Result<Option<Self>, String> {
        let geometry = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").ok();
        let support = std::env::var("NUWAX_NAV_BODY_TO_SUPPORT_Z_M").ok();
        Self::from_config(geometry.as_deref(), support.as_deref())
    }

    pub(crate) fn from_config(
        geometry: Option<&str>,
        support: Option<&str>,
    ) -> Result<Option<Self>, String> {
        let Some(geometry) = geometry.filter(|value| !value.trim().is_empty()) else {
            return Ok(None);
        };
        let geometry = RobotGeometry::from_runtime_json(geometry)?;
        let Some(support) = support.filter(|value| !value.trim().is_empty()) else {
            // 无支撑标定时不将模型腿部零位包络当成可信地面高度。
            return Ok(None);
        };
        let body_to_support_z_m = support
            .parse::<f64>()
            .map_err(|_| "自体过滤需要有效的机体至支撑面标定".to_string())?;
        if !body_to_support_z_m.is_finite() || body_to_support_z_m <= 0.0 {
            return Err("自体过滤需要正的机体至支撑面标定".into());
        }
        Ok(Some(Self {
            geometry,
            body_to_support_z_m,
        }))
    }

    pub(crate) fn retain_external(
        &self,
        points: &mut Vec<Point3<f64>>,
        odom_from_body: &Isometry3<f64>,
    ) {
        let body_from_odom = odom_from_body.inverse();
        let support = odom_from_body * Point3::new(0.0, 0.0, -self.body_to_support_z_m);
        // 规范精细图的一层量化误差内保留全部地面观测；不改变支撑高度标定。
        let support_ceiling = support.z + f64::from(crate::pointcloud::VOXEL_SIZE);
        points.retain(|point| {
            // 非有限值留给原规范注册校验拒绝，不能被自体过滤静默吞掉。
            if point.coords.iter().any(|value| !value.is_finite()) || point.z <= support_ceiling {
                return true;
            }
            let body = body_from_odom * point;
            !self.contains_body(&body)
        });
    }

    fn contains_body(&self, body: &Point3<f64>) -> bool {
        if body.coords.iter().any(|v| !v.is_finite())
            || body.z < self.geometry.z_min_m
            || body.z > self.geometry.z_max_m
        {
            return false;
        }
        let vertices = &self.geometry.footprint_xy_m;
        // 只删除声明的凸包内部；不膨胀轮廓，避免吞掉邻近桌腿等真实障碍。
        vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .all(|(a, b)| {
                (b[0] - a[0]) * (body.y - a[1]) - (b[1] - a[1]) * (body.x - a[0]) >= -1e-10
            })
    }
}

/// 原始输入仅附加内部无效标记，保留所有采样和惯性时间窗。
pub(crate) struct NativeSelfFilter {
    filter: RobotSelfFilter,
    body_from_sensor: crate::native_lio_owner::BodyFromSensorTransform,
}

impl NativeSelfFilter {
    pub(crate) fn from_env() -> Result<Option<Self>, String> {
        let Some(filter) = RobotSelfFilter::from_env()? else {
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
        let mut request = source.clone();
        request.points = Vec::with_capacity(capacity);
        for point in source.points.chunks_exact(step) {
            let xyz = [0, 4, 8].map(|offset| {
                f32::from_le_bytes(point[offset..offset + 4].try_into().unwrap()) as f64
            });
            let body = self.body_from_sensor.transform_point(&Point3::from(xyz));
            // 尚无LIO位姿时使用既有机体支撑平面，地面带仍留一层精细体素。
            let is_self = body.z
                > -self.filter.body_to_support_z_m + f64::from(crate::pointcloud::VOXEL_SIZE)
                && self.filter.contains_body(&body);
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
        RobotSelfFilter::from_config(Some(r#"{"footprint_xy_m":[[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]],"z_min_m":-0.448,"z_max_m":0.057}"#), Some("0.4")).unwrap().unwrap()
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
    fn self_cloud_raw_mask_preserves_layout_samples_and_last_point_time() {
        use crate::proto::{PointCloudField, PublishPointCloudRequest};
        for points in [
            vec![[0.0_f32, 0.1, -0.425], [0.3, 0.1, -0.2], [0.0, 0.1, -0.2]],
            vec![[0.0, 0.1, -0.2]; 3],
        ] {
            let mut source = PublishPointCloudRequest {
                point_step: 20,
                point_count: 3,
                timestamp_ns: 123,
                frame_id: "sensor".into(),
                is_dense: true,
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
    fn self_cloud_keeps_invalid_points_for_canonical_validation_and_requires_calibration() {
        assert!(RobotSelfFilter::from_config(None, Some("0.4"))
            .unwrap()
            .is_none());
        assert!(RobotSelfFilter::from_config(Some("{}"), Some("0.4")).is_err());
        let mut points = vec![Point3::new(f64::NAN, 0.0, 0.0)];
        filter().retain_external(&mut points, &Isometry3::identity());
        assert_eq!(points.len(), 1);
    }
}
