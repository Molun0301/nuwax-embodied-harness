use super::*;

fn pose(x: f64, yaw: f64, stamp: u64) -> proto::Pose {
    proto::Pose {
        position: Some(proto::Vec3 { x, y: 0.0, z: 0.0 }),
        orientation: Some(proto::Quat {
            x: 0.0,
            y: 0.0,
            z: (yaw * 0.5).sin(),
            w: (yaw * 0.5).cos(),
        }),
        frame_id: "odom".into(),
        timestamp_ns: stamp,
    }
}

#[test]
fn auxiliary_capture_uses_interpolated_source_pose_and_requires_bracketing() {
    let poses = VecDeque::from([
        pose(2.0, 0.0, 100),
        pose(6.0, std::f64::consts::PI / 2.0, 200),
    ]);
    let captured = capture_pose(&poses, 150).unwrap().unwrap();
    let transform = pose_isometry(&captured).unwrap();
    let point = transform * Point3::new(1.0, 0.0, 0.0);
    let expected = std::f64::consts::FRAC_1_SQRT_2;
    assert!((point.x - (4.0 + expected)).abs() < 1.0e-12);
    assert!((point.y - expected).abs() < 1.0e-12);
    assert_eq!(captured.timestamp_ns, 150);
    assert!(capture_pose(&poses, 99).unwrap().is_none());
    assert!(capture_pose(&poses, 201).unwrap().is_none());
}

fn cloud(frame: &str, stamp: u64) -> schema::PointCloud2 {
    let request = proto::PublishPointCloudRequest {
        points: [1_f32, 0., 0.]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect(),
        point_step: 12,
        point_count: 1,
        frame_id: frame.into(),
        timestamp_ns: stamp,
        ..Default::default()
    };
    decode_cloud(&encode_pointcloud_request_payload(&request).unwrap()).unwrap()
}
#[test]
fn native_pairing_is_order_independent() {
    for cloud_first in [false, true] {
        let mut poses = VecDeque::new();
        let mut clouds = VecDeque::new();
        if cloud_first {
            clouds.push_back((1, 100, cloud("laser", 100)));
            assert!(take_pair(
                &poses,
                &mut clouds,
                Some(std::time::Duration::from_nanos(10))
            )
            .is_none());
            poses.push_back(pose(3., 0., 100));
        } else {
            poses.push_back(pose(3., 0., 100));
            assert!(take_pair(
                &poses,
                &mut clouds,
                Some(std::time::Duration::from_nanos(10))
            )
            .is_none());
            clouds.push_back((1, 100, cloud("laser", 100)));
        }
        let (_, paired, paired_cloud) = take_pair(
            &poses,
            &mut clouds,
            Some(std::time::Duration::from_nanos(10)),
        )
        .unwrap();
        assert_eq!(paired.timestamp_ns, 100);
        assert_eq!(header_stamp(&paired_cloud.header).unwrap(), 100);
        assert_eq!(paired.position.unwrap().x, 3.);
    }
}
#[test]
fn native_pairing_rejects_excess_skew_and_drops_cloud_without_candidate_after_watermark() {
    let poses = VecDeque::from([pose(0., 0., 100), pose(1., 0., 200)]);
    for stamp in [50, 150] {
        let mut clouds = VecDeque::from([(1, stamp, cloud("laser", stamp))]);
        assert!(take_pair(
            &poses,
            &mut clouds,
            Some(std::time::Duration::from_nanos(10))
        )
        .is_none());
        assert!(clouds.is_empty());
    }
    let mut waiting = VecDeque::from([(1, 210, cloud("laser", 210))]);
    assert!(take_pair(
        &poses,
        &mut waiting,
        Some(std::time::Duration::from_nanos(10))
    )
    .is_none());
    assert_eq!(waiting.len(), 1);
}
#[test]
fn cloud_parser_rejects_non_finite_points_and_input_layout_limits() {
    for step in [1, 4096] {
        let request = proto::PublishPointCloudRequest {
            points: vec![0; step],
            point_step: step as u32,
            point_count: 1,
            frame_id: "odom".into(),
            ..Default::default()
        };
        assert!(encode_pointcloud_request_payload(&request).is_err());
    }
    let request = proto::PublishPointCloudRequest {
        points: vec![0; (crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS + 1) * 12],
        point_step: 12,
        point_count: crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS as u32 + 1,
        frame_id: "odom".into(),
        ..Default::default()
    };
    assert!(encode_pointcloud_request_payload(&request).is_err());
    let mut invalid = cloud("laser", 1);
    invalid.data[..4].copy_from_slice(&f32::NAN.to_le_bytes());
    assert!(cloud_points(&invalid).is_err());
    invalid = cloud("laser", 1);
    invalid.fields[0].offset = 4;
    assert!(cloud_points(&invalid).is_err());
    assert_eq!(cloud_points(&cloud("laser", 1)).unwrap().len(), 1);
}

#[tokio::test]
async fn native_odometry_reaches_handoff_without_public_lcm_loop() {
    let mapper = crate::mapper::Mapper::start(41, crate::mapper::MapperConfig::default()).unwrap();
    let (router, mut pose_rx, mut faults) = crate::session::test_router(mapper.clone());
    let transport = LcmTransport::new_real("memq://b2-native-direct").unwrap();
    let odom = transport
        .subscribe_owned(
            LcmTopic::new("/b2/native/odom").unwrap(),
            Some(LcmMessageType::new("nav_msgs.Odometry").unwrap()),
        )
        .await
        .unwrap();
    let clouds = transport
        .subscribe_owned(
            LcmTopic::new("/b2/native/cloud").unwrap(),
            Some(LcmMessageType::new("sensor_msgs.PointCloud2").unwrap()),
        )
        .await
        .unwrap();
    let (_odom_tx, odom_rx) = mpsc::unbounded_channel();
    let (_cloud_tx, cloud_rx) = mpsc::unbounded_channel();
    let (stop, stopped) = watch::channel(false);
    let native = native_bound();
    let ingress = Ingress {
        router,
        native: Some(native),
        self_filter: None,
        transport: transport.clone(),
        auxiliary: Default::default(),
    };
    let task = tokio::spawn(ingress.run(odom, clouds, odom_rx, cloud_rx, stopped));
    let message = schema::Odometry {
        header: schema::Header {
            stamp_sec: 2,
            frame_id: "odom".into(),
            ..Default::default()
        },
        pose: schema::Pose {
            position: [3., 4., 0.],
            orientation_xyzw: [0., 0., 0., 1.],
        },
        ..Default::default()
    };
    transport
        .publish(
            RawLcmFrameBuilder::new(
                LcmTopic::new("/b2/native/odom").unwrap(),
                LcmMessageType::new("nav_msgs.Odometry").unwrap(),
            )
            .payload(schema::encode_odometry_body(&message).unwrap())
            .build(),
        )
        .await
        .unwrap();
    tokio::time::timeout(std::time::Duration::from_secs(2), pose_rx.changed())
        .await
        .unwrap()
        .unwrap();
    let output = pose_rx.borrow().clone().unwrap();
    assert_eq!(output.timestamp_ns, 2_000_000_000);
    assert_eq!(
        (output.position.unwrap().x, output.frame_id),
        (3., "map".into())
    );
    assert!(faults.try_recv().is_err());
    stop.send_replace(true);
    task.await.unwrap();
    mapper.shutdown().await.unwrap();
}

fn native_bound() -> crate::native_lio_owner::BoundNativeLioRuntime {
    crate::native_lio_owner::BoundNativeLioRuntime {
        channels: crate::native_lio_owner::NativePrivateChannels {
            source_odometry: binding("/b2/native/odom", "nav_msgs.Odometry").unwrap(),
            source_cloud: binding("/b2/native/cloud", "sensor_msgs.PointCloud2").unwrap(),
            input_cloud: binding("/b2/input/cloud", "sensor_msgs.PointCloud2").unwrap(),
            input_imu: binding("/b2/input/imu", "sensor_msgs.Imu").unwrap(),
            input_ack: None,
            input_frame: None,
        },
        runtime: Box::new(nuwax_lio_backends::native::NativeRuntimeSpec {
            backend: NativeBackend::FastLio2,
            cwd: "/".into(),
            executable: "/bin/true".into(),
            args: Vec::new(),
            env: Default::default(),
        }),
        body_from_sensor: crate::native_lio_owner::BodyFromSensorTransform::from_json(
            r#"{"translation_m":[0,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}"#,
        )
        .unwrap(),
        pair_max_skew: std::time::Duration::from_millis(25),
    }
}

fn test_lio(
    mapper: Arc<crate::mapper::Mapper>,
    native: Option<BoundNativeLioRuntime>,
) -> (Lio, mpsc::UnboundedReceiver<(u64, crate::session::SessionIssue)>) {
    let (router, _, faults) = crate::session::test_router(mapper);
    let (odometry_input, _) = mpsc::unbounded_channel();
    let (cloud_input, _) = mpsc::unbounded_channel();
    let transport = LcmTransport::new_real("memq://b2-input-boundary").unwrap();
    (
        Lio {
            router,
            info: proto::RuntimeInfo {
                pipeline: "lio_3d".into(),
                map_frame_id: "map".into(),
                odom_frame_id: "odom".into(),
                base_frame_id: "base_link".into(),
                ..Default::default()
            },
            native: native.clone(),
            planar: None,
            runtime: Mutex::new(None),
            input_runtime: parking_lot::RwLock::new(Some(InputRuntime {
                transport,
                native,
                planar: None,
                odometry_input,
                cloud_input,
            })),
            input: Mutex::new(Default::default()),
            native_self_filter: None,
            sensor_frame: "laser".into(),
            body_from_sensor: None,
            auxiliary: Default::default(),
        },
        faults,
    )
}
fn raw_request(stamp: u64) -> proto::PublishPointCloudRequest {
    let mut points = [1_f32, 0., 0.]
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect::<Vec<_>>();
    points.extend(0_u32.to_le_bytes());
    proto::PublishPointCloudRequest {
        points,
        point_step: 16,
        point_count: 1,
        frame_id: "laser".into(),
        timestamp_ns: stamp,
        fields: [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("offset_ns", 12, 6)]
            .into_iter()
            .map(|(name, offset, datatype)| proto::PointCloudField {
                name: name.into(),
                offset,
                datatype,
                count: 1,
            })
            .collect(),
        imu_samples: vec![proto::LioImuSample {
            timestamp_ns: stamp,
            frame_id: "laser".into(),
            accel_mps2_z: 9.81,
            ..Default::default()
        }],
        ..Default::default()
    }
}
#[tokio::test]
async fn imu_gap_reports_session_fault_once() {
    let mapper = crate::mapper::Mapper::start(42, crate::mapper::MapperConfig::default()).unwrap();
    let (lio, mut faults) = test_lio(mapper.clone(), Some(native_bound()));
    lio.publish_point_cloud(raw_request(1)).await.unwrap();
    for stamp in [500_000_002, 500_000_003, 1_000_000_000] {
        assert_eq!(
            lio.publish_point_cloud(raw_request(stamp))
                .await
                .unwrap_err()
                .code(),
            tonic::Code::FailedPrecondition
        );
    }
    let (id, crate::session::SessionIssue::Fault(reason)) = faults.try_recv().unwrap() else {
        panic!("惯性输入断档应报告会话故障");
    };
    assert_eq!(id, 42);
    assert!(reason.contains("惯性源时间断档"));
    assert!(faults.try_recv().is_err());
    mapper.shutdown().await.unwrap();
}
#[tokio::test]
async fn unknown_sensor_frame_is_rejected_without_map_mutation() {
    let mapper = crate::mapper::Mapper::start(43, crate::mapper::MapperConfig::default()).unwrap();
    for native in [None, Some(native_bound())] {
        let (lio, mut faults) = test_lio(mapper.clone(), native);
        for frame in ["map", "unknown"] {
            let mut request = raw_request(100);
            request.frame_id = frame.into();
            request.imu_samples = if lio.native.is_some() {
                request.imu_samples
            } else {
                Vec::new()
            };
            assert_eq!(
                lio.publish_point_cloud(request).await.unwrap_err().code(),
                tonic::Code::InvalidArgument
            );
        }
        assert!(faults.try_recv().is_err());
        assert!(lio.router.recent_frames().is_empty());
    }
    assert!(mapper.snapshot().await.unwrap().occupancy.is_none());
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn input_rpc_does_not_wait_for_the_session_runtime_lock() {
    let mapper = crate::mapper::Mapper::start(44, crate::mapper::MapperConfig::default()).unwrap();
    let (lio, _) = test_lio(mapper.clone(), None);
    let control = lio.runtime.lock().await;
    let result = tokio::time::timeout(
        std::time::Duration::from_millis(100),
        lio.publish_laser_scan(proto::PublishLaserScanRequest::default()),
    )
    .await
    .unwrap();
    assert_eq!(result.unwrap_err().code(), tonic::Code::FailedPrecondition);
    let result = tokio::time::timeout(
        std::time::Duration::from_millis(100),
        lio.publish_odom(proto::PublishOdomRequest {
            pose: Some(pose(0., 0., 100)),
            ..Default::default()
        }),
    )
    .await
    .unwrap();
    // 输入端点已关闭的错误必须能直接返回，不等会话控制锁释放。
    assert_eq!(result.unwrap_err().code(), tonic::Code::Unavailable);
    drop(control);
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn odom_pointcloud_before_pose_is_rejected_without_mutation() {
    let mapper = crate::mapper::Mapper::start(45, crate::mapper::MapperConfig::default()).unwrap();
    let (router, mut poses, mut faults) = crate::session::test_router(mapper.clone());
    let transport = LcmTransport::new_real("memq://b2-before-pose").unwrap();
    let odom = transport
        .subscribe_owned(
            LcmTopic::new("/b2/before/odom").unwrap(),
            Some(LcmMessageType::new("nav_msgs.Odometry").unwrap()),
        )
        .await
        .unwrap();
    let clouds = transport
        .subscribe_owned(
            LcmTopic::new("/b2/before/cloud").unwrap(),
            Some(LcmMessageType::new("sensor_msgs.PointCloud2").unwrap()),
        )
        .await
        .unwrap();
    let (odom_tx, odom_rx) = mpsc::unbounded_channel();
    let (cloud_tx, cloud_rx) = mpsc::unbounded_channel();
    let (stop, stopped) = watch::channel(false);
    let task = tokio::spawn(
        Ingress {
            router: router.clone(),
            native: None,
            self_filter: None,
            transport,
            auxiliary: Default::default(),
        }
        .run(odom, clouds, odom_rx, cloud_rx, stopped),
    );
    cloud_tx.send((45, cloud("odom", 100))).unwrap();
    assert!(mapper.snapshot().await.unwrap().occupancy.is_none());
    odom_tx.send(pose(2., 0., 200)).unwrap();
    tokio::time::timeout(std::time::Duration::from_secs(2), poses.changed())
        .await
        .unwrap()
        .unwrap();
    let mut mapped = mapper.pose_rx();
    cloud_tx.send((45, cloud("odom", 200))).unwrap();
    tokio::time::timeout(std::time::Duration::from_secs(2), mapped.changed())
        .await
        .unwrap()
        .unwrap();
    stop.send_replace(true);
    task.await.unwrap();
    assert_eq!(
        mapper
            .snapshot()
            .await
            .unwrap()
            .last_pose
            .unwrap()
            .timestamp_ns,
        200
    );
    let frames = router.recent_frames();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].odom_pose.timestamp_ns, 200);
    assert!(faults.try_recv().is_err());
    mapper.shutdown().await.unwrap();
}
