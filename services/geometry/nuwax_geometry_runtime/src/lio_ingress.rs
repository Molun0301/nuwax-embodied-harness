//! 输入边界的物理载荷编解码；不持有会话或帧准入状态。
use crate::proto::{LioImuSample, PointCloudField, PublishPointCloudRequest};
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use nuwax_codec::lcm::schema;
use nuwax_codec::{CodecError, CodecResult};

pub(crate) fn decode_odometry(payload: &[u8]) -> Result<schema::Odometry, String> {
    let mut wire = schema::ODOMETRY_FINGERPRINT.to_vec();
    wire.extend_from_slice(payload);
    schema::decode_odometry(&wire).map_err(|error| error.to_string())
}
pub(crate) fn decode_cloud(payload: &[u8]) -> Result<schema::PointCloud2, String> {
    let mut wire = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
    wire.extend_from_slice(payload);
    schema::decode_point_cloud2(&wire).map_err(|error| error.to_string())
}

pub(crate) fn pose_from_odometry(value: &schema::Odometry) -> Result<crate::proto::Pose, String> {
    let pose = crate::proto::Pose {
        position: Some(crate::proto::Vec3 {
            x: value.pose.position[0],
            y: value.pose.position[1],
            z: value.pose.position[2],
        }),
        orientation: Some(crate::proto::Quat {
            x: value.pose.orientation_xyzw[0],
            y: value.pose.orientation_xyzw[1],
            z: value.pose.orientation_xyzw[2],
            w: value.pose.orientation_xyzw[3],
        }),
        frame_id: value.header.frame_id.clone(),
        timestamp_ns: header_stamp(&value.header)?,
    };
    pose_isometry(&pose)?;
    Ok(pose)
}

pub(crate) fn header_stamp(header: &schema::Header) -> Result<u64, String> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err("传感器源时间非法".into());
    }
    Ok(header.stamp_sec as u64 * 1_000_000_000 + header.stamp_nsec as u64)
}

pub(crate) fn pose_isometry(pose: &crate::proto::Pose) -> Result<Isometry3<f64>, String> {
    let position = pose.position.as_ref().ok_or("位姿缺少位置")?;
    let orientation = pose.orientation.as_ref().ok_or("位姿缺少四元数")?;
    if [
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]
    .iter()
    .any(|value| !value.is_finite())
    {
        return Err("位姿含非有限数值".into());
    }
    let q = Quaternion::new(orientation.w, orientation.x, orientation.y, orientation.z);
    if (q.norm() - 1.0).abs() > 1.0e-3 {
        return Err("位姿四元数不是单位长度".into());
    }
    Ok(Isometry3::from_parts(
        Translation3::new(position.x, position.y, position.z),
        UnitQuaternion::new_normalize(q),
    ))
}

pub(crate) fn cloud_points(cloud: &schema::PointCloud2) -> Result<Vec<Point3<f64>>, String> {
    if cloud.is_bigendian
        || cloud.point_step < 12
        || cloud.data.len() % cloud.point_step as usize != 0
    {
        return Err("点云必须是完整的小端规范布局".into());
    }
    for (name, offset) in [("x", 0), ("y", 4), ("z", 8)] {
        if !cloud
            .fields
            .iter()
            .any(|f| f.name == name && f.offset == offset && f.datatype == 7 && f.count == 1)
        {
            return Err(format!("点云缺少规范{name}字段"));
        }
    }
    cloud
        .data
        .chunks_exact(cloud.point_step as usize)
        .map(|raw| {
            let xyz = [0, 4, 8].map(|offset| {
                f32::from_le_bytes(raw[offset..offset + 4].try_into().unwrap()) as f64
            });
            if xyz.iter().any(|v| !v.is_finite()) {
                return Err("点云含非有限坐标".into());
            }
            Ok(Point3::from(xyz))
        })
        .collect()
}

fn canonical_xyz_fields() -> Vec<schema::PointField> {
    vec![
        schema::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: 7,
            count: 1,
        },
        schema::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: 7,
            count: 1,
        },
        schema::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: 7,
            count: 1,
        },
    ]
}

fn point_field_size(datatype: u32) -> Option<u32> {
    match datatype {
        1 | 2 => Some(1),
        3 | 4 => Some(2),
        5..=7 => Some(4),
        8 => Some(8),
        _ => None,
    }
}

fn validate_runtime_point_fields(fields: &[PointCloudField], point_step: u32) -> CodecResult<()> {
    for field in fields {
        let size = point_field_size(field.datatype).ok_or_else(|| {
            CodecError::LcmEncode(format!(
                "unsupported point field datatype {} for {:?}",
                field.datatype, field.name
            ))
        })?;
        let field_end = field
            .offset
            .checked_add(size.checked_mul(field.count).ok_or_else(|| {
                CodecError::LcmEncode(format!("point field {:?} size overflow", field.name))
            })?)
            .ok_or_else(|| {
                CodecError::LcmEncode(format!("point field {:?} offset overflow", field.name))
            })?;
        if field.name.is_empty() || field.count == 0 || field_end > point_step {
            return Err(CodecError::LcmEncode(format!(
                "invalid point field name={:?} offset={} datatype={} count={} point_step={}",
                field.name, field.offset, field.datatype, field.count, point_step
            )));
        }
    }

    for (name, offset) in [("x", 0), ("y", 4), ("z", 8)] {
        let field = fields.iter().find(|field| field.name == name);
        if !matches!(
            field,
            Some(field) if field.offset == offset && field.datatype == 7 && field.count == 1
        ) {
            return Err(CodecError::LcmEncode(format!(
                "runtime requires canonical {name} float32 field at offset {offset}"
            )));
        }
    }
    Ok(())
}

pub(crate) fn validate_native_raw_pointcloud_fields(
    request: &PublishPointCloudRequest,
) -> CodecResult<()> {
    if request.fields.is_empty() {
        return Err(CodecError::LcmEncode(
            "native raw PointCloud2 requires explicit fields; fields cannot be inferred".into(),
        ));
    }
    validate_runtime_point_fields(&request.fields, request.point_step)?;
    let offset = request
        .fields
        .iter()
        .find(|field| field.name == "offset_ns");
    if !matches!(offset, Some(field) if field.datatype == 6 && field.count == 1) {
        return Err(CodecError::LcmEncode(
            "native raw PointCloud2 requires offset_ns UINT32 field".into(),
        ));
    }
    Ok(())
}

pub(crate) fn encode_native_imu_payload(sample: &LioImuSample) -> CodecResult<Vec<u8>> {
    let values = [
        sample.accel_mps2_x,
        sample.accel_mps2_y,
        sample.accel_mps2_z,
        sample.gyro_rps_x,
        sample.gyro_rps_y,
        sample.gyro_rps_z,
    ];
    if sample.frame_id.is_empty() || sample.frame_id.trim() != sample.frame_id {
        return Err(CodecError::LcmEncode(
            "native raw IMU frame_id must be non-empty without surrounding whitespace".into(),
        ));
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(CodecError::LcmEncode(
            "native raw IMU acceleration and gyro values must be finite".into(),
        ));
    }
    let timestamp_ns = sample.timestamp_ns;
    let stamp_sec = i32::try_from(timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("IMU timestamp seconds exceed int32".into()))?;
    let orientation = if sample.orientation_valid {
        let q = [
            sample.orientation_x,
            sample.orientation_y,
            sample.orientation_z,
            sample.orientation_w,
        ];
        if q.iter().any(|value| !value.is_finite()) {
            return Err(CodecError::LcmEncode(
                "native raw IMU orientation must be finite when orientation_valid".into(),
            ));
        }
        (q, [0.0; 9])
    } else {
        // sensor_msgs/Imu convention: orientation_covariance[0] == -1 means unknown.
        ([0.0; 4], [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    };
    Ok(schema::encode_imu_body(&schema::Imu {
        header: schema::Header {
            seq: 0,
            stamp_sec,
            stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
            frame_id: sample.frame_id.clone(),
        },
        orientation_xyzw: orientation.0,
        orientation_covariance: orientation.1,
        angular_velocity: [sample.gyro_rps_x, sample.gyro_rps_y, sample.gyro_rps_z],
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: [
            sample.accel_mps2_x,
            sample.accel_mps2_y,
            sample.accel_mps2_z,
        ],
        linear_acceleration_covariance: [0.0; 9],
    }))
}

pub(crate) fn encode_pointcloud_request_payload(
    request: &PublishPointCloudRequest,
) -> CodecResult<Vec<u8>> {
    encode_pointcloud_request_payload_with_sequence(request, 0)
}

pub(crate) fn encode_pointcloud_request_payload_with_sequence(
    request: &PublishPointCloudRequest,
    sequence: i32,
) -> CodecResult<Vec<u8>> {
    if request.frame_id.is_empty() || request.frame_id.trim() != request.frame_id {
        return Err(CodecError::LcmEncode(
            "point cloud frame_id must be non-empty without surrounding whitespace".into(),
        ));
    }
    let point_step = usize::try_from(request.point_step)
        .map_err(|_| CodecError::LcmEncode("point_step exceeds usize".into()))?;
    if point_step < 12 || request.points.len() % point_step != 0 {
        return Err(CodecError::LcmEncode(format!(
            "point cloud data length {} is not aligned to point_step {}",
            request.points.len(),
            request.point_step
        )));
    }

    let derived_point_count = request.points.len() / point_step;
    if request.point_count != 0 && request.point_count as usize != derived_point_count {
        return Err(CodecError::LcmEncode(format!(
            "point_count {} does not match data-derived count {}",
            request.point_count, derived_point_count
        )));
    }

    let fields = if request.fields.is_empty() {
        canonical_xyz_fields()
    } else {
        validate_runtime_point_fields(&request.fields, request.point_step)?;
        request
            .fields
            .iter()
            .map(|field| {
                Ok(schema::PointField {
                    name: field.name.clone(),
                    offset: i32::try_from(field.offset).map_err(|_| {
                        CodecError::LcmEncode("point field offset exceeds int32".into())
                    })?,
                    datatype: u8::try_from(field.datatype).map_err(|_| {
                        CodecError::LcmEncode("point field datatype exceeds uint8".into())
                    })?,
                    count: i32::try_from(field.count).map_err(|_| {
                        CodecError::LcmEncode("point field count exceeds int32".into())
                    })?,
                })
            })
            .collect::<CodecResult<Vec<_>>>()?
    };

    let stamp_sec = i32::try_from(request.timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("point cloud timestamp seconds exceed int32".into()))?;
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema::Header {
            seq: sequence,
            stamp_sec,
            stamp_nsec: (request.timestamp_ns % 1_000_000_000) as i32,
            frame_id: request.frame_id.clone(),
        },
        height: 1,
        width: i32::try_from(derived_point_count)
            .map_err(|_| CodecError::LcmEncode("point cloud width exceeds int32".into()))?,
        fields,
        is_bigendian: false,
        point_step: i32::try_from(request.point_step)
            .map_err(|_| CodecError::LcmEncode("point_step exceeds int32".into()))?,
        row_step: i32::try_from(request.points.len())
            .map_err(|_| CodecError::LcmEncode("point cloud row_step exceeds int32".into()))?,
        data: request.points.clone(),
        is_dense: request.is_dense,
    })
}
