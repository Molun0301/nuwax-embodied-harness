use super::*;
use nalgebra::{Translation3, UnitQuaternion};

fn activity_filter() -> RobotSelfFilter {
    RobotSelfFilter::from_runtime_geometry("base_link", r#"{
        "footprint_xy_m":[[-0.2,-0.4],[0.6,-0.4],[0.6,0.2],[-0.2,0.2]],
        "z_min_m":-0.4,"z_max_m":0.15,
        "self_collision":{"primitives":[{"shape":"box","size_m":[0.04,0.04,0.04],
            "translation_m":[0,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}]}
    }"#).unwrap().unwrap()
}

#[test]
fn activity_box_masks_swing_space_and_preserves_ground_and_external_returns() {
    let filter = activity_filter();
    // 偏心外形的活动范围为 X[-0.3,0.7]、Y[-0.5,0.3]、Z[-0.35,0.15]。
    for point in [[-0.29,-0.49,-0.34], [0.69,0.29,0.14], [0.4,0.1,-0.2]] {
        assert!(filter.contains_body(&Point3::from(point)), "活动盒内 {point:?}");
    }
    let external = [
        [-0.31,0.0,-0.2], [0.71,0.0,-0.2], [0.0,-0.51,-0.2], [0.0,0.31,-0.2],
        [0.0,0.0,0.16], [0.0,0.0,-0.351], [0.0,0.0,-0.39], [0.0,0.0,-0.4],
    ];
    for point in external {
        assert!(!filter.contains_body(&Point3::from(point)), "外部或支撑面附近 {point:?}");
    }
    let pose = Isometry3::from_parts(Translation3::new(2.0,-3.0,0.7),
        UnitQuaternion::from_euler_angles(0.2,-0.15,1.7));
    let retained: Vec<_> = external.iter().map(|p|pose*Point3::from(*p)).collect();
    let mut points = retained.clone();
    points.push(pose*Point3::new(0.4,0.1,-0.2));
    filter.retain_external(&mut points,&pose);
    assert_eq!(points,retained, "建图和近期点云只收到保留的外部点");
}

#[test]
fn activity_box_uses_existing_native_self_marker_without_changing_payload() {
    let mut filter = native_filter();
    filter.filter = activity_filter();
    let body_points = [[0.4_f32,0.1,-0.2],[-0.25,-0.45,-0.2],
        [0.71,0.0,-0.2],[0.0,0.31,-0.2],[0.0,0.0,-0.39],[0.0,0.0,-0.351],[0.0,0.0,0.16]];
    let mut request = crate::proto::PublishPointCloudRequest {
        point_step:12,point_count:body_points.len() as u32,..Default::default()
    };
    for [x,y,z] in body_points {
        // 复用既有非平凡外参：传感器点 [y,0.1-x,z] 对应机体点 [x,y,z]。
        for value in [y,0.1-x,z] {request.points.extend_from_slice(&value.to_le_bytes());}
    }
    let masked = filter.mask_request(&request).unwrap();
    assert_eq!(masked.points.chunks_exact(13).map(|p|p[12]).collect::<Vec<_>>(),
        vec![1,1,0,0,0,0,0]);
    assert_eq!(masked.point_count,request.point_count);
    assert_eq!(masked.fields.last().unwrap().name,SELF_MASK_FIELD);
    for (before,after) in request.points.chunks_exact(12).zip(masked.points.chunks_exact(13)) {
        assert_eq!(before,&after[..12], "自体标记保留每个原始采样");
    }
}

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
    for point in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.15, 0.0, 0.0]] {
        assert!(filter.contains_body(&Point3::from(point)), "{point:?}");
    }
    for point in [
        [0.5, 0.0, 0.0],
        [0.0, 0.0, -0.2],
        [1.08, 0.08, 0.0],
        [2.0, 0.0, 0.15],
        [2.21, 0.0, 0.0],
    ] {
        assert!(!filter.contains_body(&Point3::from(point)), "{point:?}");
    }
    for (roll, pitch, yaw) in [(0.0, 0.0, 0.0), (0.2, -0.3, 1.5)] {
        let pose = Isometry3::from_parts(
            Translation3::new(3.0, -2.0, 0.7),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        );
        let external = pose * Point3::new(0.5, 0.0, -0.2);
        let mut points = vec![external, pose * Point3::new(0.0, 0.0, 0.0)];
        filter.retain_external(&mut points, &pose);
        assert_eq!(points, vec![external]);
    }
}

#[test]
fn self_cloud_broad_phase_matches_exact_shapes_at_boundaries() {
    for angle in [0.0_f64, 0.3, 1.7] {
        let matrix = UnitQuaternion::from_euler_angles(angle, -angle / 2.0, angle / 3.0)
            .to_rotation_matrix();
        let rotation: Vec<_> = (0..3)
            .flat_map(|i| (0..3).map(move |j| matrix[(i, j)]))
            .collect();
        for center in [[0.0, 0.0, 0.0], [10000.0, -20000.0, 30000.0]] {
            for shape in [
                serde_json::json!({"shape":"box", "size_m":[0.4,0.2,0.6]}),
                serde_json::json!({"shape":"sphere", "radius_m":0.2}),
                serde_json::json!({"shape":"cylinder", "radius_m":0.2, "length_m":0.6}),
            ] {
                let mut primitive = shape;
                primitive["translation_m"] = serde_json::json!(center);
                primitive["rotation_row_major"] = serde_json::json!(rotation);
                let json =
                    serde_json::json!({"self_collision":{"primitives":[primitive]}}).to_string();
                let mut filter = RobotSelfFilter::from_config(Some(&json)).unwrap().unwrap();
                let bounds = filter.body_bounds;
                let pose = filter.primitives[0].shape_from_body.inverse();
                for x in [
                    -0.400000000001,
                    -0.4,
                    -0.3,
                    -0.200000000001,
                    -0.2,
                    -0.1,
                    0.0,
                    0.1,
                    0.2,
                    0.200000000001,
                    0.3,
                    0.4,
                    0.400000000001,
                ] {
                    for y in [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3] {
                        for z in [-0.300000000001, -0.3, -0.2, 0.0, 0.2, 0.3, 0.300000000001] {
                            let p = pose * Point3::new(x, y, z);
                            filter.body_bounds = ([f64::NEG_INFINITY; 3], [f64::INFINITY; 3]);
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
            crate::lio_ingress::encode_pointcloud_request_payload_with_sequence(&masked, 7)
                .unwrap();
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
        assert_eq!(
            &masked.fields[..source.fields.len()],
            source.fields.as_slice()
        );
        assert_eq!(
            masked.fields.last().unwrap(),
            &PointCloudField {
                name: SELF_MASK_FIELD.into(),
                offset: source.point_step,
                datatype: 2,
                count: 1,
            }
        );
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
    assert!(RobotSelfFilter::from_config(None).unwrap().is_none());
    assert!(RobotSelfFilter::from_config(Some("{}")).unwrap().is_none());
    let mut points = vec![Point3::new(f64::NAN, 0.0, 0.0)];
    filter().retain_external(&mut points, &Isometry3::identity());
    assert_eq!(points.len(), 1);
}
