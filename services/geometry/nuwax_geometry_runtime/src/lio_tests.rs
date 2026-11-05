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
